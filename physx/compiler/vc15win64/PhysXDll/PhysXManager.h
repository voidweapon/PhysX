#pragma once
#ifndef PHYSXMANAGER
#define PHYSXMANAGER

#include<set> 
#include "PxPhysicsAPI.h"
#include "PsArray.h"
#include "extensions/PxExtensionsAPI.h"
#include "ControlledScene.h"

using namespace physx;

class PhysXManager : PxSimulationFilterCallback
{
public:
	PhysXManager();
	~PhysXManager();

	/// <summary>
	/// 初始化物理引擎
	/// </summary>
	bool onInit(bool* collisionTable);

	void setSleepThreshold(float threshold);
	PxReal getSleepThreshold();

	void setBounceThreshold(PxReal threshold);
	PxReal getBounceThreshold();


	void setDefaultContactOffset(PxReal contactOffset);
	PxReal getDefaultContactOffset();


	void setDefaultSolverIterations(PxReal minPositionIters);
	PxReal getDefaultSolverIterations();

	void setDefaultSolverVelocityIterations(PxReal minVelocityIters);
	PxReal getDefaultSolverVelocityIterations();

	void setFrictionType(PxFrictionType::Enum type);
	PxFrictionType::Enum getFrictionType();

	/// <summary>
	/// 关闭物理引擎
	/// </summary>
	void onShutdown();

	/// <summary>
	/// 创建物理场景
	/// </summary>
	ControlledScene* createScene();

	/// <summary>
	/// 销毁物理场景
	/// </summary>
	void destroyScene(ControlledScene* scene);

public://impelement PxSimulationFilterCallback
	PxFilterFlags	pairFound(PxU32 pairID,
		PxFilterObjectAttributes attributes0, PxFilterData filterData0, const PxActor* a0, const PxShape* s0,
		PxFilterObjectAttributes attributes1, PxFilterData filterData1, const PxActor* a1, const PxShape* s1,
		PxPairFlags& pairFlags);

	void			pairLost(PxU32 pairID,
		PxFilterObjectAttributes attributes0,
		PxFilterData filterData0,
		PxFilterObjectAttributes attributes1,
		PxFilterData filterData1,
		bool objectRemoved);

	bool			statusChange(PxU32& pairID, PxPairFlags& pairFlags, PxFilterFlags& filterFlags);

protected:
	void togglePvdConnection();
	void createPvdConnection();


	physx::PxPvd*                           mPvd;
	physx::PxPvdTransport*                  mTransport;
	physx::PxPvdInstrumentationFlags        mPvdFlags;

protected: // configuration
	PxU32									mNbThreads;
private:
	PxFoundation*							mFoundation;
	PxPhysics*								mPhysics;
	PxCooking*								mCooking;
	PxMaterial*								mMaterial;
	PxDefaultCpuDispatcher*					mCpuDispatcher;

	PxReal									mSleepThreshold;
	PxReal									mBounceThreshold;
	PxReal									mDefaultContactOffset;
	PxReal									mDefaultSolverIterations;
	PxReal									mDefaultSolverVelocityIterations;
	PxFrictionType::Enum					mDefaultFrictionType;

private: // Runtime
	std::set<ControlledScene*>				mSetScene;
};

#endif