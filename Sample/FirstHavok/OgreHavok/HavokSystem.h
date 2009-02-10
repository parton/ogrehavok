#pragma once

//头文件包含
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
	//创建hkpWorld
	virtual bool createHavokWorld(hkReal worldsize);
	//初始化VDB
	virtual bool InitVDB();
	//创建物理场景
	virtual void createPhysicsScene();
	void		 setGroundSize(hkReal x,hkReal y,hkReal z);
	void		 setGroundPos(hkReal x,hkReal y,hkReal z);

	//step simulation
	virtual void simulate();

	void		 setup();

	//Physics
	hkpWorld*					m_World;

protected:
	//成员变量
	hkPoolMemory*				m_MemoryManager;
	hkThreadMemory*				m_ThreadMemory;
	char*						m_StackBuffer;
	int							m_StackSize;
	//多线程相关
	hkJobThreadPool*			m_ThreadPool;
	int							m_TotalNumThreadUsed;
	hkJobQueue*					m_JobQueue;
	
	//VDB相关
	hkArray<hkProcessContext*>	m_Contexts;
	hkpPhysicsContext*			m_Context;
	hkVisualDebugger*			m_Vdb;
	hkpRigidBody*				m_Ground;			//地面
	hkVector4					m_GroundSize;		
	hkVector4					m_GroundPos;		//地面位置
	//模拟控制
	bool						m_bPause;
	//是否启用Debugger
	bool						m_bEnableVDB;
};


//错误处理函数
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
	//初始化堆栈
	{
		m_StackSize = 0x100000;
		m_StackBuffer = hkAllocate<char>(m_StackSize,HK_MEMORY_CLASS_BASE);
		hkThreadMemory::getInstance().setStackArea(m_StackBuffer,m_StackSize);
	}
	//初始化多线程
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

	//设置默认的地面尺寸，如果想更改的话，在setup之前调用setGroundSize()设置尺寸
	m_GroundSize.set(300.0f,2.0f,300.0f);
	//设置默认的地面位置，在setup之前调用setGroundPos()修改
	m_GroundPos.set(0.0f,0.0f,0.0f);

	//默认不使用ＶＤＢ
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
	//设置world尺寸
	worldInfo.setBroadPhaseWorldSize(worldsize);
	//worldInfo.m_gravity = hkVector4(0.0f,-16.0f,0.0f);

	m_World = new hkpWorld(worldInfo);
	m_World->m_wantDeactivation = false;
	m_World->markForWrite();
	//注册碰撞代理
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
	//创建Ground
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
//设置地面的尺寸
void HavokSystem::setGroundSize(hkReal x,hkReal y,hkReal z)
{
	m_GroundSize.set(x,y,z);
}
//设置地面的位置
void HavokSystem::setGroundPos(hkReal x, hkReal y, hkReal z)
{
	m_GroundSize.set(x,y,z);
}

void HavokSystem::setup()
{
	//创建hkpWorld
	createHavokWorld(3000.0f);
	//初始化远程调试
	InitVDB();
	//创建物理场景
	createPhysicsScene();
}