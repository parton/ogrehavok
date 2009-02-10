#pragma once

//ͷ�ļ�����
/******************************************************************************************/
#include <windows.h>
// Math and base include
#include <Common/Base/hkBase.h>
#include <Common/Base/System/hkBaseSystem.h>
#include <Common/Base/Memory/hkThreadMemory.h>
#include <Common/Base/Memory/Memory/Pool/hkPoolMemory.h>
#include <Common/Base/System/Error/hkDefaultError.h>
#include <Common/Base/Monitor/hkMonitorStream.h>

// Dynamics includes
#include <Physics/Collide/hkpCollide.h>										
#include <Physics/Collide/Agent/ConvexAgent/SphereBox/hkpSphereBoxAgent.h>	
#include <Physics/Collide/Shape/Convex/Box/hkpBoxShape.h>					
#include <Physics/Collide/Shape/Convex/Sphere/hkpSphereShape.h>				
#include <Physics/Collide/Dispatch/hkpAgentRegisterUtil.h>					

#include <Physics/Collide/Query/CastUtil/hkpWorldRayCastInput.h>			
#include <Physics/Collide/Query/CastUtil/hkpWorldRayCastOutput.h>			

#include <Physics/Dynamics/World/hkpWorld.h>								
#include <Physics/Dynamics/Entity/hkpRigidBody.h>							
#include <Physics/Utilities/Dynamics/Inertia/hkpInertiaTensorComputer.h>	

#include <Common/Base/Thread/Job/ThreadPool/Cpu/hkCpuJobThreadPool.h>
#include <Common/Base/Thread/Job/ThreadPool/Spu/hkSpuJobThreadPool.h>
#include <Common/Base/Thread/JobQueue/hkJobQueue.h>

// Visual Debugger includes
#include <Common/Visualize/hkVisualDebugger.h>
#include <Physics/Utilities/VisualDebugger/hkpPhysicsContext.h>				

// Classlists
#define INCLUDE_HAVOK_PHYSICS_CLASSES
#define HK_CLASSES_FILE <Common/Serialize/Classlist/hkClasses.h>
#include <Common/Serialize/Util/hkBuiltinTypeRegistry.cxx>

// Generate a custom list to trim memory requirements
#define HK_COMPAT_FILE <Common/Compat/hkCompatVersions.h>
#include <Common/Compat/hkCompat_None.cxx>

//Keycode
//#include <Common/Base/keycode.cxx>

#include <stdio.h>

/************************************************************************************************/



class HavokSystem
{
public:
	HavokSystem(void);
	

	~HavokSystem(void);
	//����hkpWorld
	virtual bool createHavokWorld(hkReal worldsize);
	//��ʼ��VDB
	virtual bool InitVDB();
	//����������
	virtual void createPhysicsScene();
	void		 setGroundSize(hkReal x,hkReal y,hkReal z);
	void		 setGroundPos(hkReal x,hkReal y,hkReal z);

	//step simulation
	virtual void simulate();

	void		 setup();

	//Physics
	hkpWorld*					m_World;

protected:
	//��Ա����
	hkPoolMemory*				m_MemoryManager;
	hkThreadMemory*				m_ThreadMemory;
	char*						m_StackBuffer;
	int							m_StackSize;
	//���߳����
	hkJobThreadPool*			m_ThreadPool;
	int							m_TotalNumThreadUsed;
	hkJobQueue*					m_JobQueue;
	
	//VDB���
	hkArray<hkProcessContext*>	m_Contexts;
	hkpPhysicsContext*			m_Context;
	hkVisualDebugger*			m_Vdb;
	hkpRigidBody*				m_Ground;			//����
	hkVector4					m_GroundSize;		
	hkVector4					m_GroundPos;		//����λ��
	//ģ�����
	bool						m_bPause;
	//�Ƿ�����Debugger
	bool						m_bEnableVDB;
};


//��������
static void HK_CALL errorReport(const char* msg,void*)
{
	printf("%s",msg);
}

HavokSystem::HavokSystem(void)
{
	m_MemoryManager = new hkPoolMemory();
	m_ThreadMemory = new hkThreadMemory(m_MemoryManager);
	hkBaseSystem::init(m_MemoryManager,m_ThreadMemory,errorReport);
	
	m_MemoryManager->removeReference();
	//��ʼ����ջ
	{
		m_StackSize = 0x100000;
		m_StackBuffer = hkAllocate<char>(m_StackSize,HK_MEMORY_CLASS_BASE);
		hkThreadMemory::getInstance().setStackArea(m_StackBuffer,m_StackSize);
	}
	//��ʼ�����߳�
	int m_TotalNumThreadsUsed;

	hkHardwareInfo hwInfo;
	hkGetHardwareInfo(hwInfo);
	m_TotalNumThreadsUsed	= hwInfo.m_numThreads;

	hkCpuJobThreadPoolCinfo gThreadPoolCinfo;
	gThreadPoolCinfo.m_numThreads = m_TotalNumThreadsUsed-1;
	gThreadPoolCinfo.m_timerBufferPerThreadAllocation = 200000;
	m_ThreadPool	= new hkCpuJobThreadPool(gThreadPoolCinfo);

	hkJobQueueCinfo info;
	info.m_jobQueueHwSetup.m_numCpuThreads = m_TotalNumThreadsUsed;
	m_JobQueue = new hkJobQueue(info);

	hkMonitorStream::getInstance().resize(200000);

	//����Ĭ�ϵĵ���ߴ磬�������ĵĻ�����setup֮ǰ����setGroundSize()���óߴ�
	m_GroundSize.set(300.0f,2.0f,300.0f);
	//����Ĭ�ϵĵ���λ�ã���setup֮ǰ����setGroundPos()�޸�
	m_GroundPos.set(0.0f,0.0f,0.0f);

	//Ĭ�ϲ�ʹ�ã֣ģ�
	//m_bEnableVDB = false;
}


HavokSystem::~HavokSystem(void)
{
	m_World->removeReference();
	m_Vdb->removeReference();
	m_Context->removeReference();
	delete m_JobQueue;
	m_ThreadPool->removeReference();
	m_ThreadMemory->setStackArea(0,0);
	hkDeallocate(m_StackBuffer);
	m_ThreadMemory->removeReference();

	hkBaseSystem::quit();
}

bool HavokSystem::createHavokWorld(hkReal worldsize)
{
	hkpWorldCinfo worldInfo;
	worldInfo.m_simulationType = hkpWorldCinfo::SIMULATION_TYPE_MULTITHREADED;
	worldInfo.m_broadPhaseBorderBehaviour = hkpWorldCinfo::BROADPHASE_BORDER_REMOVE_ENTITY;
	//����world�ߴ�
	worldInfo.setBroadPhaseWorldSize(worldsize);
	//worldInfo.m_gravity = hkVector4(0.0f,-16.0f,0.0f);

	m_World = new hkpWorld(worldInfo);
	m_World->m_wantDeactivation = false;
	m_World->markForWrite();
	//ע����ײ����
	hkpAgentRegisterUtil::registerAllAgents(m_World->getCollisionDispatcher());
	m_World->registerWithJobQueue(m_JobQueue);

	m_World->unmarkForWrite();
	return true;
}

bool HavokSystem::InitVDB()
{

	m_World->markForWrite();
	m_Context = new hkpPhysicsContext();
	hkpPhysicsContext::registerAllPhysicsProcesses();
	m_Context->addWorld(m_World);
	m_Contexts.pushBack(m_Context);

	m_World->unmarkForWrite();

	m_Vdb = new hkVisualDebugger(m_Contexts);
	m_Vdb->serve();

	return true;
}

void HavokSystem::createPhysicsScene()
{
	m_World->markForWrite();
	//����Ground
	//hkVector4 groundRadii(300.0f,2.0f,200.0f);
	hkpConvexShape* shape = new hkpBoxShape(m_GroundSize,0.05f);

	hkpRigidBodyCinfo ci;

	ci.m_shape = shape;
	ci.m_motionType = hkpMotion::MOTION_FIXED;
	ci.m_position = m_GroundPos;
	ci.m_qualityType = HK_COLLIDABLE_QUALITY_FIXED;

	m_Ground = new hkpRigidBody(ci);
	m_World->addEntity(m_Ground);
	shape->removeReference();
	m_World->unmarkForWrite();
}

void HavokSystem::simulate()
{
	hkStopwatch stopWatch;
	stopWatch.start();
	hkReal lastTime = stopWatch.getElapsedSeconds();

	hkReal timestep = 1.0f/60.0f;

	m_World->stepMultithreaded(m_JobQueue,m_ThreadPool,timestep);

		m_Context->syncTimers(m_ThreadPool);
		m_Vdb->step();
	

	hkMonitorStream::getInstance().reset();
	m_ThreadPool->clearTimerData();

	while(stopWatch.getElapsedSeconds()<lastTime+timestep);
	lastTime += timestep;
	//step the graphics display here

	
}
//���õ���ĳߴ�
void HavokSystem::setGroundSize(hkReal x,hkReal y,hkReal z)
{
	m_GroundSize.set(x,y,z);
}
//���õ����λ��
void HavokSystem::setGroundPos(hkReal x, hkReal y, hkReal z)
{
	m_GroundSize.set(x,y,z);
}

void HavokSystem::setup()
{
	//����hkpWorld
	createHavokWorld(3000.0f);
	//��ʼ��Զ�̵���
	InitVDB();
	//����������
	createPhysicsScene();
}