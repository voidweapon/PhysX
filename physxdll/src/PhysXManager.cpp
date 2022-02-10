#include"PhysXManager.h"

PxDefaultAllocator gDefaultAllocatorCallback;
PxDefaultErrorCallback gDefaultErrorCallback;

PxFilterFlags contactReportFilterShader(PxFilterObjectAttributes attributes0, PxFilterData filterData0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1,
	PxPairFlags& pairFlags, const void* constantBlock, PxU32 constantBlockSize)
{
	PX_UNUSED(constantBlockSize);
	PX_UNUSED(constantBlock);

	PxFilterObjectType::Enum type0 = PxGetFilterObjectType(attributes0);
	PxFilterObjectType::Enum type1 = PxGetFilterObjectType(attributes1);
	bool isKinematic0 = PxFilterObjectIsKinematic(attributes0);
	bool isKinematic1 = PxFilterObjectIsKinematic(attributes1);

	/*
	*	Kill
	*	Static Collider - Static Collider			
	*	Static Collider - Static Trigger Collider
	*	Static Trigger Collider - Static Collider
	*	Static Trigger Collider - Static Trigger Collider
	*/
	if (type0 == PxFilterObjectType::Enum::eRIGID_STATIC && type1 == PxFilterObjectType::Enum::eRIGID_STATIC)
	{
		return PxFilterFlag::eKILL;
	}

	// let triggers through
	if (PxFilterObjectIsTrigger(attributes0) || PxFilterObjectIsTrigger(attributes1))
	{
		pairFlags = PxPairFlag::eTRIGGER_DEFAULT;
		return PxFilterFlag::eDEFAULT;
	}

	/*
	*	Kill 
	*	Static Collider - Kinematic Rigidbody Collider
	*	Kinematic Rigidbody Collider - Static Collider
	*	Kinematic Rigidbody Collider - Kinematic Rigidbody Collider
	*/
	if ((type0 == PxFilterObjectType::Enum::eRIGID_STATIC && isKinematic1) //Static Collider - Kinematic Rigidbody Collider
		|| (isKinematic0 && type1 == PxFilterObjectType::Enum::eRIGID_STATIC) //Kinematic Rigidbody Collider - Static Collider
		|| (type0 == PxFilterObjectType::Enum::eRIGID_DYNAMIC && type1 == PxFilterObjectType::Enum::eRIGID_DYNAMIC && isKinematic0 && isKinematic1) //Kinematic Rigidbody Collider - Kinematic Rigidbody Collider
		)
	{
		return PxFilterFlag::eKILL;
	}


	if (PxGetGroupCollisionFlag(filterData0.word0, filterData1.word0))
	{
		pairFlags = PxPairFlag::eCONTACT_DEFAULT;
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_FOUND;
		pairFlags |= PxPairFlag::eNOTIFY_TOUCH_LOST;

		return PxFilterFlag::eDEFAULT | PxFilterFlag::eCALLBACK;
	}
	
	return PxFilterFlag::eSUPPRESS;
}

PhysXManager::PhysXManager():
	mNbThreads(1),
	mFoundation(NULL),
	mPhysics(NULL),
	mCooking(NULL),
	mCpuDispatcher(NULL),
	mPvd(NULL),
	mTransport(NULL),
	mSleepThreshold(0.005),
	mBounceThreshold(2),
	mDefaultContactOffset(0.01),
	mDefaultSolverIterations(6),
	mDefaultSolverVelocityIterations(1),
	mDefaultFrictionType(PxFrictionType::ePATCH)

{
	
}

PhysXManager::~PhysXManager()
{
	
}

bool PhysXManager::onInit(bool* collisionTable)
{
	bool recordMemoryAllocations = false;
	
	mFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gDefaultAllocatorCallback, gDefaultErrorCallback);

	createPvdConnection();

	PxTolerancesScale scale;

	mPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *mFoundation, scale, recordMemoryAllocations, mPvd);

	PxCookingParams params(scale);
	params.meshWeldTolerance = 0.001f;
	params.meshPreprocessParams = PxMeshPreprocessingFlags(PxMeshPreprocessingFlag::eWELD_VERTICES);
	//params.buildGPUData = true; //Enable GRB data being produced in cooking.
	mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *mFoundation, params);

	mMaterial = mPhysics->createMaterial(0.5f, 0.5f, 0);

	mCpuDispatcher = PxDefaultCpuDispatcherCreate(mNbThreads);

	int index = 0;
	for (size_t i = 0; i < 32; i++)
	{
		for (size_t j = 0; j < 32; j++)
		{
			PxSetGroupCollisionFlag(i, j, collisionTable[index++]);
		}
	}

	return true;
}

void PhysXManager::onShutdown()
{
	for (auto itr = mSetScene.begin(); itr != mSetScene.end(); itr++)
	{
		(*itr)->release();
	}
	mSetScene.clear();

	mCpuDispatcher->release();
	mCpuDispatcher = NULL;

	mCooking->release();
	mCooking = NULL;

	mMaterial->release();
	mMaterial = NULL;

	mPhysics->release();
	mPhysics = NULL;

	if (mPvd) 
	{
		mPvd->release();
		mPvd = NULL;
	}

	if (mTransport)
	{
		mTransport->release();
		mTransport = NULL;
	}

	mFoundation->release();
	mFoundation = NULL;
}


void PhysXManager::setSleepThreshold(float threshold)
{
	mSleepThreshold = threshold;
}

PxReal PhysXManager::getSleepThreshold()
{
	return mSleepThreshold;
}

void PhysXManager::setBounceThreshold(PxReal threshold)
{
	mBounceThreshold = threshold;
}
PxReal PhysXManager::getBounceThreshold()
{
	return mBounceThreshold;
}


void PhysXManager::setDefaultContactOffset(PxReal contactOffset)
{
	mDefaultContactOffset = contactOffset;
}
PxReal PhysXManager::getDefaultContactOffset()
{
	return mDefaultContactOffset;
}


void PhysXManager::setDefaultSolverIterations(PxReal minPositionIters)
{
	mDefaultSolverIterations = minPositionIters;
}
PxReal PhysXManager::getDefaultSolverIterations()
{
	return mDefaultSolverIterations;
}

void PhysXManager::setDefaultSolverVelocityIterations(PxReal minVelocityIters)
{
	mDefaultSolverVelocityIterations = minVelocityIters;
}
PxReal PhysXManager::getDefaultSolverVelocityIterations()
{
	return mDefaultSolverVelocityIterations;
}

void PhysXManager::setFrictionType(PxFrictionType::Enum type)
{
	mDefaultFrictionType = type;
}
PxFrictionType::Enum PhysXManager::getFrictionType()
{
	return mDefaultFrictionType;
}


void PhysXManager::togglePvdConnection()
{
	if (!mPvd) return;
	if (mPvd->isConnected())
		mPvd->disconnect();
	else
		mPvd->connect(*mTransport, mPvdFlags);
}

void PhysXManager::createPvdConnection()
{
#if PX_SUPPORT_PVD

	mTransport = physx::PxDefaultPvdSocketTransportCreate("127.0.0.1", 5425, 10);
	if (mTransport == NULL)
		return;

	mPvdFlags = physx::PxPvdInstrumentationFlag::eDEBUG /*| PxPvdInstrumentationFlag::ePROFILE*/;
	mPvd = physx::PxCreatePvd(*mFoundation);
	mPvd->connect(*mTransport, mPvdFlags);
#endif
}

ControlledScene* PhysXManager::createScene()
{

	PxSceneDesc sceneDesc(mPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);

	sceneDesc.cpuDispatcher = mCpuDispatcher;
	sceneDesc.filterShader = contactReportFilterShader;
	sceneDesc.broadPhaseType = PxBroadPhaseType::eSAP;
	sceneDesc.filterCallback = this;
	//sceneDesc.filterShader = PxDefaultSimulationFilterShader;

	sceneDesc.flags |= PxSceneFlag::eENABLE_ENHANCED_DETERMINISM;
	sceneDesc.flags |= PxSceneFlag::eENABLE_PCM;
	sceneDesc.flags |= PxSceneFlag::eENABLE_STABILIZATION;
	sceneDesc.flags |= PxSceneFlag::eENABLE_ACTIVE_ACTORS;
	sceneDesc.flags |= PxSceneFlag::eENABLE_CCD;
	sceneDesc.sceneQueryUpdateMode = PxSceneQueryUpdateMode::eBUILD_ENABLED_COMMIT_DISABLED;

	PxScene* newScene = mPhysics->createScene(sceneDesc);
	if (!newScene) 
	{
		return NULL;
	}

	newScene->setBounceThresholdVelocity(mBounceThreshold);
	newScene->setFrictionType(mDefaultFrictionType);

	ControlledScene* sceneController = new ControlledScene(mPhysics, newScene, mMaterial);
	sceneController->mSleepThreshold = &(this->mSleepThreshold);
	sceneController->mDefaultContactOffset = &(this->mDefaultContactOffset);
	sceneController->mDefaultSolverIterations = &(this->mDefaultSolverIterations);
	sceneController->mDefaultSolverVelocityIterations = &(this->mDefaultSolverVelocityIterations);
	mSetScene.insert(sceneController);

	return sceneController;
}

void PhysXManager::destroyScene(ControlledScene* scene)
{
	if (!scene)
	{
		return;
	}
	scene->release();

	mSetScene.erase(scene);
}

PxFilterFlags	PhysXManager::pairFound(PxU32 pairID,
	PxFilterObjectAttributes attributes0, PxFilterData filterData0, const PxActor* a0, const PxShape* s0,
	PxFilterObjectAttributes attributes1, PxFilterData filterData1, const PxActor* a1, const PxShape* s1,
	PxPairFlags& pairFlags) 
{
	PX_UNUSED(pairID);
	PX_UNUSED(filterData0);
	PX_UNUSED(filterData1);
	PX_UNUSED(s0);
	PX_UNUSED(s1);

	PxFilterObjectType::Enum type0 = PxGetFilterObjectType(attributes0);
	PxFilterObjectType::Enum type1 = PxGetFilterObjectType(attributes1);
	int detectionMode0 = 0;
	int detectionMode1 = 0;
	bool isContinuousDynamic0 = false;
	bool isContinuousDynamic1 = false;


	if (type0 == PxFilterObjectType::eRIGID_DYNAMIC) 
	{
		PxActor* _a0 = const_cast<PxActor*>(a0);
		PxRigidBody* rigidBody0 = static_cast<PxRigidBody*>(_a0);
		PxRigidBodyFlags rigidBodyFlags0 = rigidBody0->getRigidBodyFlags();
		if (rigidBodyFlags0 & PxRigidBodyFlag::eENABLE_CCD) 
		{
			detectionMode0 = 1;
		}
		else if(rigidBodyFlags0 & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
		{
			detectionMode0 = 2;
		}
	}
	if (type1 == PxFilterObjectType::eRIGID_DYNAMIC)
	{
		PxActor* _a1 = const_cast<PxActor*>(a1);
		PxRigidBody* rigidBody1 = static_cast<PxRigidBody*>(_a1);
		PxRigidBodyFlags rigidBodyFlags1 = rigidBody1->getRigidBodyFlags();
		if (rigidBodyFlags1 & PxRigidBodyFlag::eENABLE_CCD)
		{
			detectionMode1 = 1;
		}
		else if (rigidBodyFlags1 & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD)
		{
			detectionMode1 = 2;
		}
	}

	if (detectionMode0 == 2 || detectionMode1 == 2)
	{
		//Speculative Continuous - Speculative Continuous
		return PxFilterFlag::eDEFAULT;
	}

	if (detectionMode0 == 1 && detectionMode1 == 1)
	{
		if (isContinuousDynamic0 || isContinuousDynamic1)
		{
			//Continuous Dynamic - Continuous Dynamic
			//Continuous Dynamic - Continuous
			pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_CCD;
		}
		//else
		//{
			// Continuous - Continuous
		//}

	}
	else if (detectionMode0 == 1 || detectionMode1 == 1)
	{
		if (type0 == PxFilterObjectType::eRIGID_DYNAMIC && type1 == PxFilterObjectType::eRIGID_DYNAMIC)
		{
			//Continuous Dynamic - Discrete Dynamic Collider
		}
		else
		{
			pairFlags |= PxPairFlag::eDETECT_CCD_CONTACT;
			pairFlags |= PxPairFlag::eNOTIFY_TOUCH_CCD;
		}
	}

	return PxFilterFlag::eDEFAULT;
}

void PhysXManager::pairLost(PxU32 pairID,
	PxFilterObjectAttributes attributes0,
	PxFilterData filterData0,
	PxFilterObjectAttributes attributes1,
	PxFilterData filterData1,
	bool objectRemoved) 
{
	PX_UNUSED(pairID);
	PX_UNUSED(attributes0);
	PX_UNUSED(filterData0);
	PX_UNUSED(attributes1);
	PX_UNUSED(filterData1);
	PX_UNUSED(objectRemoved);
}

bool PhysXManager::statusChange(PxU32& pairID, PxPairFlags& pairFlags, PxFilterFlags& filterFlags) 
{
	PX_UNUSED(pairID);
	PX_UNUSED(pairFlags);
	PX_UNUSED(filterFlags);

	return false;
}