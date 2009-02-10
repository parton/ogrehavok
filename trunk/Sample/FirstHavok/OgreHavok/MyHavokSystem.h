#pragma once

#include "HavokSystem.h"
#include <Ogre.h>
#include <ExampleApplication.h>
#include <ExampleFrameListener.h>


/////////////////////////////////////////////////////////////////////////////////////////////////
//帮助函数
//将Havok 的hkVector4转换成Ogre的Vector3
Ogre::Vector3 hkVector4ToOgre(hkVector4& vec)
{
	Ogre::Vector3 p = Vector3(vec(0),vec(1),vec(2));
	return p;
}

//将Havok的hkQuaternion转换成Ogre的四元数
Ogre::Quaternion hkQuatToOgre(hkQuaternion& quat)
{	
	hkVector4 imgpart = quat.getImag();
	hkReal realpart = quat.getReal();

	Ogre::Quaternion q = Ogre::Quaternion(realpart,imgpart(0),imgpart(1),imgpart(2));
	return q;
}
hkVector4 OgreTohkVector4(Ogre::Vector3& vec)
{
	hkVector4 v;
	v(0) = vec.x;
	v(1) = vec.y;
	v(2) = vec.z;

	return v;
}

/////////////////////////////////////////////////////////////////////////////////////////////////
//类Body定义,包装了Ogre的SceneNode和Havok的RigidBody

/*
用于创建OgreHavokBody的描述符
*/

#define BODY_TYPE_BOX		0
#define BODY_TYPE_SPHERE	1

class OGREHAVOKBODY_DESC{
public:
	hkpWorld*	world;
	Ogre::SceneManager* sceneMgr;
	//描述Havok刚体
	hkReal		mass;			//Rigidbody质量
	int       bodyType;			//body类型,有Box,Sphere
	bool	  dynamic;			//是否是Dynamic的
	hkVector4 pos;				//Body创建位置
	hkVector4 size;				//只用于Box,box的尺寸
	hkReal    radius;			//如果body类型是Shpere或Capsule,这个是半径
	Ogre::Vector3 scale;		//OgreNode的缩放
	Ogre::String meshName;		//Ogre Mesh的名字
	Ogre::String bodyName;		//body名
public:
	//默认构造函数
	OGREHAVOKBODY_DESC():world(HK_NULL),sceneMgr(NULL),mass(5.0f),bodyType(BODY_TYPE_BOX),dynamic(true),
						pos(hkVector4(0.0f,0.0f,0.0f)),size(hkVector4(1.0f,1.0f,1.0f)),radius(1.0f),
						scale(Ogre::Vector3(1,1,1))
	{
		
	}
	//复制构造函数
	OGREHAVOKBODY_DESC(OGREHAVOKBODY_DESC& desc)
	{
		world=desc.world;
		sceneMgr = desc.sceneMgr;
		mass=	desc.mass;
		bodyType=desc.bodyType;
		dynamic = desc.dynamic;
		pos = desc.pos;
		size = desc.size;
		radius = desc.radius;
		scale = desc.scale;
		meshName = desc.meshName;
		bodyName = desc.bodyName;
	}

};

/*
用于包装Havok和Ogre
*/

/////////////////////////////////////////////////////////////////////////////////////////////////
//类Body定义,包装了Ogre的SceneNode和Havok的RigidBody
class OgreHavokBody
{
public:
	/*构造函数
	havokWorld		hkpWorld指针
	sceneMgr		Ogre SceneManager指针
	pos				创建hkpRigidBody时的初始位置
	size			创建hkpRigidBody时的尺寸
	scale			Ogre场景节点的缩放
	meshName		Ogre mesh文件名
	*/
	OgreHavokBody(OGREHAVOKBODY_DESC& desc)
	{
		m_BodyType =		desc.bodyType;
		m_hkpWorld =		desc.world;
		m_SceneMgr =		desc.sceneMgr;
		if(m_BodyType==BODY_TYPE_BOX){
			m_Size = desc.size;
		}else if(m_BodyType==BODY_TYPE_SPHERE){
			m_Radius = desc.radius;
		}
		
		m_Position =		desc.pos;
		m_Scale	   =		desc.scale;
		m_MeshName =		desc.meshName;
		m_BodyName =		desc.bodyName;
		m_Mass     =		desc.mass;
		m_bDynamic =		desc.dynamic;

		if(m_BodyType==BODY_TYPE_BOX){
			_createhkpBoxRigidBody();
		}else if(m_BodyType==BODY_TYPE_SPHERE){
			_createhkpSphereRigidBody();
		}
		_createOgreNode();
	};

	~OgreHavokBody()
	{
		m_hkpWorld->removeEntity(m_HavokRigid);
		m_SceneMgr->destroySceneNode(m_OgreNode->getName());
	}

	//内部使用类,创建Havok 物理对象
	void _createhkpBoxRigidBody()
	{
		m_hkpWorld->markForWrite();
		
		hkpConvexShape* shape = new hkpBoxShape(m_Size,0.0f);
		hkpRigidBodyCinfo boxInfo;

		if(m_bDynamic){
			//创建动态的刚体
			boxInfo.m_mass = m_Mass;
			hkpMassProperties massProperties;
			hkpInertiaTensorComputer::computeBoxVolumeMassProperties(m_Size,boxInfo.m_mass,massProperties);

			boxInfo.m_mass = massProperties.m_mass;
			boxInfo.m_centerOfMass = massProperties.m_centerOfMass;
			boxInfo.m_inertiaTensor= massProperties.m_inertiaTensor;
			boxInfo.m_solverDeactivation = boxInfo.SOLVER_DEACTIVATION_MEDIUM;
			boxInfo.m_shape = shape;
			boxInfo.m_motionType = hkpMotion::MOTION_BOX_INERTIA;
			boxInfo.m_position = m_Position;
		}else{
			//静态的
			boxInfo.m_shape = shape;
			boxInfo.m_mass = 0.0f;
			boxInfo.m_motionType = hkpMotion::MOTION_FIXED;
			boxInfo.m_position =m_Position;
			boxInfo.m_qualityType = HK_COLLIDABLE_QUALITY_FIXED;
		}

		//创建刚体
		m_HavokRigid = new hkpRigidBody(boxInfo);
		m_hkpWorld->addEntity(m_HavokRigid);

		m_hkpWorld->unmarkForWrite();
		//引用计数减一
		m_HavokRigid->removeReference();
		shape->removeReference();
	}
	//内部使用类,创建Sphere刚体
	void _createhkpSphereRigidBody()
	{
		m_hkpWorld->markForWrite();
		hkpConvexShape* shape = new hkpSphereShape(m_Radius);
		hkpRigidBodyCinfo sphereInfo;
		
		if(m_bDynamic){
			sphereInfo.m_mass = m_Mass;

			hkpMassProperties massProperties;
			hkpInertiaTensorComputer::computeSphereVolumeMassProperties(m_Radius,m_Mass,massProperties);

			sphereInfo.m_mass = massProperties.m_mass;
			sphereInfo.m_centerOfMass = massProperties.m_centerOfMass;
			sphereInfo.m_inertiaTensor = massProperties.m_inertiaTensor;
			sphereInfo.m_solverDeactivation = sphereInfo.SOLVER_DEACTIVATION_MEDIUM;
			sphereInfo.m_shape = shape;
			sphereInfo.m_motionType = hkpMotion::MOTION_SPHERE_INERTIA;
			sphereInfo.m_position = m_Position;
		}else{
			sphereInfo.m_shape=shape;
			sphereInfo.m_mass = 0.0f;
			sphereInfo.m_motionType = hkpMotion::MOTION_FIXED;
			sphereInfo.m_position = m_Position;
			sphereInfo.m_qualityType = HK_COLLIDABLE_QUALITY_FIXED;
		}
		//创建Havok hkpRigidBody
		m_HavokRigid = new hkpRigidBody(sphereInfo);
		m_hkpWorld->addEntity(m_HavokRigid);

		m_HavokRigid->removeReference();
		shape->removeReference();
	}

	//内部使用类,创建Ogre场景节点
	void _createOgreNode()
	{
		Ogre::String str = m_BodyName + "_Entity";
		Entity* ent = m_SceneMgr->createEntity(str,m_MeshName);
		//创建节点
		m_OgreNode = m_SceneMgr->getRootSceneNode()->createChildSceneNode();
		m_OgreNode->scale(m_Scale);
		m_OgreNode->attachObject(ent);
		Ogre::Vector3 pos = hkVector4ToOgre(m_Position);
		//设置Ogre节点的初始位置
		m_OgreNode->setPosition(pos);
	}
	//同步Havok物理对象和Ogre场景节点
	void update()
	{
		m_hkpWorld->markForRead();
		hkVector4 pos = m_HavokRigid->getPosition();
		Ogre::Vector3 position = hkVector4ToOgre(pos);
		m_OgreNode->setPosition(position);

		hkQuaternion quat = m_HavokRigid->getRotation();
		Ogre::Quaternion q = hkQuatToOgre(quat);
		m_OgreNode->setOrientation(q);
		m_hkpWorld->unmarkForRead();
	}

	//得到Havok刚体hkpRigidBody
	hkpRigidBody* getRigidBody(){ return m_HavokRigid;}

private:
	//body形状类型
	int						m_BodyType;
	//Ogre场景节点
	Ogre::SceneNode*		m_OgreNode;
	//ogre entity
	Ogre::Entity*			m_Entity;		
	//Havok刚体
	hkpRigidBody*			m_HavokRigid;
	//Havok World
	hkpWorld*				m_hkpWorld;
	//havok rigid body mass
	hkReal					m_Mass;
	//是否Dynamic
	bool					m_bDynamic;
	//Ogre SceneManager
	Ogre::SceneManager*		m_SceneMgr;
	//Ogre Mesh文件
	Ogre::String			m_MeshName;
	//这个对象的名字
	Ogre::String			m_BodyName;
	//body pos
	hkVector4				m_Position;
	//Body size
	hkVector4				m_Size;
	//Sphere时半径
	hkReal					m_Radius;
	//场景节点创建时指定的缩放
	Ogre::Vector3			m_Scale;
	hkQuaternion			m_Quat;
};

/////////////////////////////////////////////////////////////////////////////////////////////////
class MyHavokSystem:public HavokSystem
{
public:
	//构造函数
	MyHavokSystem();
	virtual void createPhysicsScene();
	hkpRigidBody* createHavokBox(hkVector4 &pos,hkVector4 &size);
	//获取Ground刚体的位置
	const Ogre::Vector3 getGroundPos();		
	//获取Ground刚体的尺寸
	const Ogre::Vector3 getGroundSize();
	//获得Ground的旋转
	const Ogre::Quaternion getGroundQuat();
	

	hkpRigidBody*	m_Box;		//一个模拟的刚体
};

void MyHavokSystem::createPhysicsScene()
{
	//setGroundSize(500,1,500);
	HavokSystem::createPhysicsScene();
	/*
	hkQuaternion rotation(hkVector4(0.0f,0.0f,1.0f),50.0f);
	m_Ground->setRotation(rotation);
	*/
}

hkpRigidBody* MyHavokSystem::createHavokBox(hkVector4 &pos,hkVector4 &size)
{
	m_World->markForWrite();
	hkpConvexShape* shape = new hkpBoxShape(size,0.05f);

	hkpRigidBodyCinfo boxInfo;
	boxInfo.m_mass = 10.0f;
	hkpMassProperties massProperties;
	hkpInertiaTensorComputer::computeBoxVolumeMassProperties(size,boxInfo.m_mass,massProperties);

	boxInfo.m_mass = massProperties.m_mass;
	boxInfo.m_centerOfMass = massProperties.m_centerOfMass;
	boxInfo.m_inertiaTensor= massProperties.m_inertiaTensor;
	boxInfo.m_solverDeactivation = boxInfo.SOLVER_DEACTIVATION_MEDIUM;
	boxInfo.m_shape = shape;
	boxInfo.m_motionType = hkpMotion::MOTION_BOX_INERTIA;
	boxInfo.m_position = pos;

	//创建刚体
	hkpRigidBody* body = new hkpRigidBody(boxInfo);
	m_World->addEntity(body);

	m_World->unmarkForWrite();
	shape->removeReference();

	return body;
}

const Ogre::Vector3 MyHavokSystem::getGroundPos()
{
	Ogre::Vector3 pos;
	pos.x = m_GroundPos(0);
	pos.y = m_GroundPos(1);
	pos.z = m_GroundPos(2);
	return pos;
}

const Ogre::Vector3 MyHavokSystem::getGroundSize()
{
	Ogre::Vector3 pos;
	pos.x = m_GroundSize(0);
	pos.y = m_GroundSize(1);
	pos.z = m_GroundSize(2);

	return pos;
}

const Ogre::Quaternion MyHavokSystem::getGroundQuat()
{
	m_World->markForRead();
	hkQuaternion hkQuat = m_Ground->getRotation();
	m_World->unmarkForRead();

	hkVector4 imgpart = hkQuat.getImag();
	hkReal realpart = hkQuat.getReal();

	Ogre::Quaternion quat = Ogre::Quaternion(realpart,imgpart(0),imgpart(1),imgpart(2));
	
	return quat;
}	

MyHavokSystem::MyHavokSystem()
{

}

/***********************************************************************************************/

using namespace Ogre;



//定义类HavokFrameListener
class HavokFrameListener:public ExampleFrameListener
{
protected:
	MyHavokSystem* mHavokSystem;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Entity* mBoxEnt;
	int				count;
	hkArray<OgreHavokBody*>		mArrayBody;

public:
	HavokFrameListener(RenderWindow* win, Camera* cam,MyHavokSystem* havokSystem,Ogre::SceneManager* sceneMgr):ExampleFrameListener( win, cam )
	{
		mHavokSystem = havokSystem;
		mSceneMgr=sceneMgr;
		count=0;

		//创建body
		
		//mBody = new OgreHavokBody(mHavokSystem->m_World,mSceneMgr,hkVector4(0,200,50),hkVector4(5,5,5),Vector3(5,5,5),meshname,bodyname);
	}

	//创建BOX
	OgreHavokBody* createBox(Ogre::Vector3& pos,hkReal mass = 1.0f)
	{
		Ogre::String meshname = "cube.1m.mesh";
		Ogre::String bodyname = "myBody"+Ogre::StringConverter::toString(count);
		hkVector4 p = OgreTohkVector4(pos);
		OGREHAVOKBODY_DESC desc;
		desc.world = mHavokSystem->m_World;
		desc.sceneMgr = mSceneMgr;
		desc.bodyName = bodyname;
		desc.meshName = meshname;
		desc.bodyType = BODY_TYPE_BOX;
		desc.mass = mass;
		desc.pos = p;
		//desc.dynamic = false;
		//desc.radius = 5;
		desc.size=hkVector4(1,1,1);
		desc.scale = Ogre::Vector3(2,2,2);
		OgreHavokBody* body = new OgreHavokBody(desc);
		mArrayBody.pushBack(body);
		count++;

		return body;
	}
	bool frameStarted( const FrameEvent& evt )
    {
        if( ExampleFrameListener::frameStarted( evt ) == false )
		return false;
		
		mHavokSystem->simulate();
		//取得box节点

		if(mKeyboard->isKeyDown(OIS::KC_SPACE)&& mTimeUntilNextToggle<=0)
		{
			Ogre::Vector3 pos = mCamera->getPosition();
			Ogre::Vector3 dir = mCamera->getDirection();
			dir.normalise();
			dir = dir*400;
			hkVector4 hkDir = OgreTohkVector4(dir);

			OgreHavokBody* body = createBox(pos,10.0f);

			mHavokSystem->m_World->markForWrite();
			body->getRigidBody()->setLinearVelocity(hkDir);
			mHavokSystem->m_World->unmarkForWrite();

			mTimeUntilNextToggle=0.5;
		}
		if(mKeyboard->isKeyDown(OIS::KC_1)&&mTimeUntilNextToggle<=0)
		{
			Ogre::Vector3 pos = Vector3(0,30,0);
			
			for(int i=1;i<10;i++){
				Vector3 newposx =Vector3(pos.x+i*2,pos.y,pos.z);
				createBox(newposx);
				for(int j=1;j<10;j++){
					Vector3 newposy = Vector3(newposx.x,newposx.y+j*2,newposx.z);
					createBox(newposy);

					for(int k=1;k<10;k++){
						Vector3 newposz = Vector3(newposy.x,newposy.y,newposy.z+k*2);
						createBox(newposz);
					}
				}
			}
			
			//createBox(pos);
			mTimeUntilNextToggle = 0.5;
		}

		if(mKeyboard->isKeyDown(OIS::KC_0)&&mTimeUntilNextToggle<=0)
		{
			hkArray<OgreHavokBody*>::iterator iter;
			for(iter = mArrayBody.begin();iter!=mArrayBody.end();iter++){
				delete (*iter);
			}

			mArrayBody.clear();
			mTimeUntilNextToggle=0.5;
		}
		hkArray<OgreHavokBody*>::iterator iter;
		for(iter = mArrayBody.begin();iter!=mArrayBody.end();iter++){
			(*iter)->update();
		}

		mDebugText = "Total body count: "+ Ogre::StringConverter::toString(mArrayBody.getSize());
		return true;
	}


};


class HavokApplication:public ExampleApplication
{
public:
    HavokApplication()
	{
		mHavokSystem = new MyHavokSystem();
	}

protected:
	virtual void createFrameListener(void)
    {
        mFrameListener= new HavokFrameListener(mWindow, mCamera,mHavokSystem,mSceneMgr);
        mRoot->addFrameListener(mFrameListener);
    }

	void createScene(void)
    {
		//mSceneMgr->setShadowTechnique( Ogre::SHADOWTYPE_STENCIL_ADDITIVE );
		// Set ambient light
        mSceneMgr->setAmbientLight(ColourValue(0.5, 0.5, 0.5));

        // Create a skybox
        //mSceneMgr->setSkyBox(true, "CloudySky", 50 );
		mCamera->getViewport()->setBackgroundColour(ColourValue(0.75,0.75,0.94));

		//创建灯光
		Ogre::Light* l = mSceneMgr->createLight("MainLight");
		l->setType(Ogre::Light::LT_POINT);
		//l->setDirection(Vector3(1,1,-1));
		l->setPosition(0,500,-10);

		l=mSceneMgr->createLight("DirLight");
		l->setType(Ogre::Light::LT_DIRECTIONAL);
		l->setDirection(1,-1,-1);
		l->setPosition(300,300,300);


		mHavokSystem->setGroundSize(100,5,100);
		//启动物理引擎
		mHavokSystem->setup();	
		//在Ogre中创建地面
		Ogre::Entity* entGround = mSceneMgr->createEntity("ground","floor200x200.mesh");
		//Ogre::Entity* entBox    = mSceneMgr->createEntity("box","cube.1m.mesh");

		Ogre::SceneNode* groundNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("groundnode");
		//Ogre::String name = "firstbox";

		//Ogre::SceneNode* boxNode = mSceneMgr->getRootSceneNode()->createChildSceneNode(name);

		groundNode->scale(1,0.5,1);
		//boxNode->scale(10,10,10);

		groundNode->attachObject(entGround);
		//boxNode->attachObject(entBox);

		groundNode->setPosition(mHavokSystem->getGroundPos());
		
		
		mHavokSystem->m_World->lock();
		Ogre::Quaternion q = mHavokSystem->getGroundQuat();
		mHavokSystem->m_World->unlock();
		
		groundNode->setOrientation(q);

		mCamera->setPosition(10,23,15);
	
	}

protected:
	MyHavokSystem*			mHavokSystem;
};
