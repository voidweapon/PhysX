#include "ControlledScene.h"

ControlledScene::ControlledScene(PxPhysics* physics, PxScene* scene, PxMaterial* defaultMaterial):
	mPhysics(physics),
	mScene(scene),
	mDefaultMaterial(defaultMaterial)
{
	mScene->setSimulationEventCallback(this);
}

ControlledScene::~ControlledScene()
{
	mPhysics = nullptr;
	mScene = nullptr;
	mDefaultMaterial = nullptr;
}

#pragma region Simulation Event
void ControlledScene::onTrigger(PxTriggerPair* pairs, PxU32 count)
{
	for (size_t i = 0; i < count; i++)
	{
		PxTriggerPair triggerPair = pairs[i];
		PxTrigger trigger;
		trigger.triggerActor = triggerPair.triggerActor;
		trigger.otherActor = triggerPair.otherActor;

		if (triggerPair.status == PxPairFlag::eNOTIFY_TOUCH_FOUND) 
		{
			trigger.status = 0;
		}	
		else if (triggerPair.status == PxPairFlag::eNOTIFY_TOUCH_LOST)
		{
			trigger.status = 1;
		}

		mListTrigger.push_back(trigger);
	}
}

void ControlledScene::onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs)
{
	for (size_t i = 0; i < nbPairs; i++)
	{
		PxContactPair contractPair = pairs[i];
		PxCollision collision;
		collision.actor0 = contractPair.shapes[0]->getActor();
		collision.actor1 = contractPair.shapes[1]->getActor();
		collision.shape0 = contractPair.shapes[0];
		collision.shape1 = contractPair.shapes[1];
		collision.contactCount = contractPair.contactCount;
		collision.contactPair = (&pairs[i]);
		//collision.status = contractPair.events;
		if (contractPair.events & PxPairFlag::eNOTIFY_TOUCH_FOUND)
		{
			collision.status = 0;
		}	
		else if (contractPair.events & PxPairFlag::eNOTIFY_TOUCH_LOST)
		{
			collision.status = 1;
		}
		else if (contractPair.events & PxPairFlag::eNOTIFY_TOUCH_PERSISTS)
		{
			collision.status = 2;
		}
		mListCollision.push_back(collision);
	}
}
#pragma endregion

void ControlledScene::release()
{
	if (!mScene)
	{
		return;
	}
	mScene->release();
	mScene = nullptr;
}

void ControlledScene::stepPhysicsScene(PxReal dt)
{
	if (!mScene)
	{
		return;
	}

	__clearCache();

	mScene->simulate(dt);
	mScene->fetchResults(true);
}

#pragma region RigidActor

void ControlledScene::__initShape(PxShape* shape, int layer, bool isTrigger)
{
	PxFilterData simulateFilter;
	simulateFilter.word0 = layer;
	PxFilterData queryFilter;
	queryFilter.word0 = 1 << layer;

	shape->setQueryFilterData(queryFilter);
	shape->setSimulationFilterData(simulateFilter);

	if (isTrigger)
	{
		shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, false);
		shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, true);

	}
	else
	{
		shape->setFlag(PxShapeFlag::eTRIGGER_SHAPE, false);
		shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
	}
	shape->setFlag(PxShapeFlag::eSCENE_QUERY_SHAPE, true);
}

PxRigidActor* ControlledScene::createRigidbody(PxTransform tm, bool isStatic)
{
	PxRigidActor* actor = nullptr;
	if (isStatic) 
	{
		PxRigidStatic* staticRd = mPhysics->createRigidStatic(tm);
		mScene->addActor(*staticRd);
		actor = staticRd;
	}
	else
	{
		PxRigidDynamic* dynamic = mPhysics->createRigidDynamic(tm);
		mScene->addActor(*dynamic);
		dynamic->setSleepThreshold(*(this->mSleepThreshold));
		dynamic->setContactReportThreshold(*(this->mDefaultContactOffset));
		dynamic->setSolverIterationCounts(*(this->mDefaultSolverIterations), *(this->mDefaultSolverVelocityIterations));
		actor = dynamic;
	}
	return actor;
}

PxRigidActor* ControlledScene::changeRigidbodyStatic(PxRigidActor* actor, bool isStatic)
{
	auto tm = actor->getGlobalPose();
	PxRigidActor* newActor = nullptr;

	if (isStatic)
	{
		PxRigidStatic* staticRd = mPhysics->createRigidStatic(tm);
		mScene->addActor(*staticRd);
		newActor = staticRd;
	}
	else
	{
		PxRigidDynamic* dynamic = mPhysics->createRigidDynamic(tm);
		mScene->addActor(*dynamic);
		dynamic->setSleepThreshold(*(this->mSleepThreshold));
		dynamic->setContactReportThreshold(*(this->mDefaultContactOffset));
		dynamic->setSolverIterationCounts(*(this->mDefaultSolverIterations), *(this->mDefaultSolverVelocityIterations));
		newActor = dynamic;
	}


	PxU32 count = actor->getNbShapes();
	for (size_t i = 0; i < count; i++)
	{
		PxShape* shape;
		actor->getShapes(&shape, 1, i);

		shape->acquireReference();
		actor->detachShape(*shape);
		newActor->attachShape(*shape);
	}

	mScene->removeActor(*actor);
	actor->release();

	return newActor;
}

PxShape* ControlledScene::addBoxCollider(PxRigidActor* actor, int layer, PxVec3 center, PxVec3 size, bool isTrigger)
{
	PxMaterial* material = mDefaultMaterial;
	size *= 0.5f;
	PxShape* shape = mPhysics->createShape(PxBoxGeometry(size.x, size.y, size.z), *material, true);

	PxTransform relativePose(center);
	shape->setLocalPose(relativePose);

	this->__initShape(shape, layer, isTrigger);

	actor->attachShape(*shape);
	shape->release();

	return shape;
}

PxShape* ControlledScene::addCapsuleCollider(PxRigidActor* actor, int layer, PxVec3 center, PxReal radius, PxReal heigh, int direction, bool isTrigger)
{
	PxMaterial* material = mDefaultMaterial;
	heigh = (heigh - radius * 2.0f) * 0.5f;
	PxShape* shape =  PxRigidActorExt::createExclusiveShape(*actor, PxCapsuleGeometry(radius, heigh), *material);


	if (direction == 0)
	{
		//X-axes
		PxTransform relativePose_X(center);
		shape->setLocalPose(relativePose_X);
	}
	else if(direction == 1)
	{
		//Y-axes
		PxTransform relativePose_Y(center, PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
		shape->setLocalPose(relativePose_Y);
	}
	else if (direction == 2)
	{
		//Z-axes
		PxTransform relativePose_Z(center, PxQuat(PxHalfPi, PxVec3(0, 1, 0)));
		shape->setLocalPose(relativePose_Z);
	}

	this->__initShape(shape, layer, isTrigger);

	return shape;
}

PxShape* ControlledScene::addSphereCollider(PxRigidActor* actor, int layer, PxVec3 center, PxReal radius, bool isTrigger)
{
	PxMaterial* material = mDefaultMaterial;
	PxShape* shape = PxRigidActorExt::createExclusiveShape(*actor, PxSphereGeometry(radius), *material);

	PxTransform relativePose(center);
	shape->setLocalPose(relativePose);

	this->__initShape(shape, layer, isTrigger);

	return shape;
}


void ControlledScene::destroyRigidActor(PxRigidActor* rigidActor)
{
	if (!mScene || !rigidActor)
	{
		return;
	}
	mScene->removeActor(*rigidActor);
	rigidActor->release();
}


void ControlledScene::removeCollider(PxRigidActor* actor, PxShape* shape)
{
	actor->detachShape(*shape);
}

#pragma endregion


#pragma region Change Single RigidActror Transform
void ControlledScene::setColliderPosition(PxRigidActor* rigidActor, PxVec3 pos)
{
	if (!mScene || !rigidActor)
	{
		return;
	}
	PxTransform tm = rigidActor->getGlobalPose();
	tm.p = pos;
	rigidActor->setGlobalPose(tm);
}
void ControlledScene::setColliderQuaternion(PxRigidActor* rigidActor, PxQuat qua)
{
	if (!mScene || !rigidActor)
	{
		return;
	}

	PxTransform tm = rigidActor->getGlobalPose();
	tm.q = qua;
	rigidActor->setGlobalPose(tm);
}
#pragma endregion

#pragma region Sync Data
void ControlledScene::syncTransforms(PxActorSync* tranforms, uint32_t count)
{
	for (size_t i = 0; i < count; i++)
	{
		PxRigidActor* actor = tranforms[i].actor;
		PxTransform tm = actor->getGlobalPose();
		tm.p = tranforms[i].tm.p;
		tm.q = tranforms[i].tm.q;
		actor->setGlobalPose(tm);
	}
}
void ControlledScene::fetchTransforms(PxActorSync* tranforms, uint32_t &count)
{
	PxU32 activeCount;
	PxActor** activeActors = mScene->getActiveActors(activeCount);
	uint32_t rigidActorCount = 0;
	for (size_t i = 0; i < activeCount; i++)
	{
		PxActor* actor = activeActors[i];
		PxRigidActor* rigidActor = (PxRigidActor*)actor;
		if (!rigidActor)
		{
			continue;
		}

		PxActorSync actorSync;
		actorSync.actor = rigidActor;
		PxTransform tm = rigidActor->getGlobalPose();
		actorSync.tm = PxTransformP();
		actorSync.tm.p = tm.p;
		actorSync.tm.q = tm.q;

		tranforms[rigidActorCount++] = actorSync;
	}
	count = rigidActorCount;
}


void ControlledScene::fetchTriggers(PxTrigger* triggers, uint32_t &count)
{
	count = mListTrigger.size();
	uint32_t index = 0;
	for (size_t i = 0; i < count; i++)
	{
		triggers[i] = mListTrigger[i];
	}
}

void ControlledScene::fetchContacts(PxCollision* collisions, uint32_t &count)
{
	count = mListCollision.size();
	uint32_t index = 0;
	for (size_t i = 0; i < count; i++)
	{
		collisions[i] = mListCollision[i];
	}
}
#pragma endregion


#pragma region Raycast
bool ControlledScene::raycast(PxVec3&  origin, PxVec3&  direction, PxReal maxDistance, PxI32 layerMask)
{
	PxRaycastBuffer rayHit;

	PxHitFlags hitFlags = (PxHitFlags)(PxHitFlag::eDEFAULT);
	PxFilterData filterData;
	filterData.word0 = layerMask;
	filterData.word1 = 0;
	PxQueryFilterData Queryfilter(filterData, PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC | PxQueryFlag::ePREFILTER);
	
	bool hit =  mScene->raycast(origin, direction, maxDistance, rayHit, hitFlags, Queryfilter, this);

	if (hit)
	{
		PxRaycastHit hitInfo = rayHit.getAnyHit(0);
		PxVec3 pos = hitInfo.position;
	}

	return hit;
}

bool ControlledScene::raycast(PxVec3 origin, PxVec3 direction, PxRaycastHit& hitInfo, float maxDistance, int layerMask)
{
	PxRaycastBuffer rayHit;

	PxHitFlags hitFlags = (PxHitFlags)(PxHitFlag::eDEFAULT);
	PxFilterData filterData;
	filterData.word0 = layerMask;
	filterData.word1 = 0;
	PxQueryFilterData Queryfilter(filterData, PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC | PxQueryFlag::ePREFILTER);

	bool hit = mScene->raycast(origin, direction, maxDistance, rayHit, hitFlags, Queryfilter, this);

	if (hit)
	{
		hitInfo = rayHit.getAnyHit(0);
	}

	return hit;
}

int ControlledScene::raycastNonAlloc(PxVec3 origin, PxVec3 direction, PxRaycastBuffer& rayHit, float maxDistance, int layerMask)
{
	PxHitFlags hitFlags = (PxHitFlags)(PxHitFlag::eDEFAULT);
	PxFilterData filterData;
	filterData.word0 = layerMask;
	filterData.word1 = 1;
	PxQueryFilterData Queryfilter(filterData, PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC | PxQueryFlag::ePREFILTER);

	bool hit = mScene->raycast(origin, direction, maxDistance, rayHit, hitFlags, Queryfilter, this);

	return  rayHit.getNbAnyHits();
}


bool ControlledScene::sphereCast(PxVec3 origin, float radius, PxVec3 direction, PxRaycastHit& hitInfo, float maxDistance, int layerMask)
{
	/*
	* Issue #471 https://github.com/NVIDIAGameWorks/PhysX/issues/471
	* Don't use = operate or copy constructor for create Geometry for overlap
	*/
	PxSphereGeometry sphere(radius);
	PxTransform tm(origin);
	PxSweepBuffer rayHit;
	bool ret = this->__sweep(sphere, tm, direction, rayHit, maxDistance, layerMask, false);
	
	if (ret)
	{
		this->__sweepBufferToPxRaycastHit(rayHit, &hitInfo);
	}

	return ret;
}
int ControlledScene::sphereCast(PxVec3 origin, float radius, PxVec3 direction, PxSweepBuffer& hitInfo, float maxDistance, int layerMask)
{	
	/*
	* Issue #471 https://github.com/NVIDIAGameWorks/PhysX/issues/471
	* Don't use = operate or copy constructor for create Geometry for overlap
	*/
	PxSphereGeometry sphere(radius);
	PxTransform tm(origin);
	this->__sweep(sphere, tm, direction, hitInfo, maxDistance, layerMask, true);

	return hitInfo.getNbAnyHits();
}

bool ControlledScene::boxCast(PxVec3 center, PxVec3 halfExtents, PxVec3 direction, PxRaycastHit& hitInfoOut, PxQuat orientation, float maxDistance, int layerMask)
{
	/*
	* Issue #471 https://github.com/NVIDIAGameWorks/PhysX/issues/471
	* Don't use = operate or copy constructor for create Geometry for overlap
	*/
	PxBoxGeometry box(halfExtents);
	PxTransform tm(center, orientation);
	PxSweepBuffer rayHit;
	bool ret = this->__sweep(box, tm, direction, rayHit, maxDistance, layerMask, false);
	
	if (ret)
	{
		this->__sweepBufferToPxRaycastHit(rayHit, &hitInfoOut);
	}

	return ret;
}
int ControlledScene::boxCastNonAlloc(PxVec3 center, PxVec3 halfExtents, PxVec3 direction, PxQuat orientation, PxSweepBuffer& hitInfo,float maxDistance, int layerMask)
{
	/*
	* Issue #471 https://github.com/NVIDIAGameWorks/PhysX/issues/471
	* Don't use = operate or copy constructor for create Geometry for overlap
	*/
	PxBoxGeometry box(halfExtents);
	PxTransform tm(center);
	this->__sweep(box, tm, direction, hitInfo, maxDistance, layerMask, true);

	return hitInfo.getNbAnyHits();
}

bool ControlledScene::capsuleCast(PxVec3 point1, PxVec3 point2, float radius, PxVec3 direction, PxRaycastHit& hitInfoOut, float maxDistance, int layerMask)
{
	/*
	* Issue #471 https://github.com/NVIDIAGameWorks/PhysX/issues/471
	* Don't use = operate or copy constructor for create Geometry for overlap
	*/
	PxVec3 shapeDir = point1 - point2;
	PxReal halfHeigh = shapeDir.magnitude() - 2.0f* radius;
	PxCapsuleGeometry capsule(radius, halfHeigh);
	PxVec3 up = PxVec3(0.0f,1.0f,0.0f);
	PxQuat pose = FromToRotation(up, shapeDir);
	PxTransform tm(0.5f * (point1 + point2), pose);

	PxSweepBuffer rayHit;
	bool ret = this->__sweep(capsule, tm, direction, rayHit, maxDistance, layerMask, false);
	
	if (ret)
	{
		 this->__sweepBufferToPxRaycastHit(rayHit, &hitInfoOut);
	}

	return ret;
}
int ControlledScene::capsuleCastNonAlloc(PxVec3 point1, PxVec3 point2, float radius, PxVec3 direction, PxSweepBuffer& hitInfo, float maxDistance, int layerMask)
{
	/* 
	* Issue #471 https://github.com/NVIDIAGameWorks/PhysX/issues/471
	* Don't use = operate or copy constructor for create Geometry for overlap
	*/
	PxVec3 shapeDir = point1 - point2;
	PxReal halfHeigh = shapeDir.magnitude() - 2.0f* radius;
	PxCapsuleGeometry capsule(radius, halfHeigh);
	PxVec3 up = PxVec3(0.0f,1.0f,0.0f);
	PxQuat pose = FromToRotation(up, shapeDir);
	PxTransform tm(0.5f * (point1 + point2), pose);

	PxSweepBuffer rayHit;
	this->__sweep(capsule, tm, direction, hitInfo, maxDistance, layerMask, true);

	return hitInfo.getNbAnyHits();
}

bool ControlledScene::overlapSphere(PxVec3 origin, float radius, PxOverlapBuffer& result, int layerMask)
{
	/* 
	* Issue #471 https://github.com/NVIDIAGameWorks/PhysX/issues/471
	* Don't use = operate or copy constructor for create Geometry for overlap
	*/
	PxSphereGeometry sphere(radius);
	PxTransform tm(origin);
	return this->__overlap(sphere, tm, result, layerMask, false);
}
bool ControlledScene::overlapBoxNonAlloc(PxVec3 center, PxVec3 halfExtents, PxQuat orientation, PxOverlapBuffer& result, int layerMask)
{
	/*
	* Issue #471 https://github.com/NVIDIAGameWorks/PhysX/issues/471
	* Don't use = operate or copy constructor for create Geometry for overlap
	*/
	PxBoxGeometry box(halfExtents);
	PxTransform tm(center, orientation);
	return this->__overlap(box, tm, result, layerMask, true);
}
bool ControlledScene::overlapCapsuleNonAlloc(PxVec3 point0, PxVec3 point1, PxReal radius, PxOverlapBuffer& result, int layerMask)
{
	/*
	* Issue #471 https://github.com/NVIDIAGameWorks/PhysX/issues/471
	* Don't use = operate or copy constructor for create Geometry for overlap
	*/
	PxVec3 shapeDir = point0 - point1;
	PxReal halfHeigh = shapeDir.magnitude() - 2.0f* radius;
	PxCapsuleGeometry capsule(radius, halfHeigh);
	PxVec3 up = PxVec3(0.0f,1.0f,0.0f);
	PxQuat pose = FromToRotation(up, shapeDir);
	PxTransform tm(0.5f * (point0 + point1), pose);
	return this->__overlap(capsule, tm, result, layerMask, true);
}


bool ControlledScene::__sweep(PxGeometry& geometry, PxTransform& pose, PxVec3 direction, PxSweepBuffer& rayHit, float maxDistance, int layerMask, bool castAll)
{
	PxHitFlags hitFlags = (PxHitFlags)(PxHitFlag::eDEFAULT);
	PxFilterData filterData;
	filterData.word0 = layerMask;
	filterData.word1 = castAll ? 1 : 0;
	PxQueryFlags flags = PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC | PxQueryFlag::ePREFILTER;
	if (!castAll)
	{
		flags |= PxQueryFlag::eANY_HIT;
	}
	PxQueryFilterData Queryfilter(filterData, flags);

	return mScene->sweep(geometry, pose, direction, maxDistance, rayHit, hitFlags, Queryfilter, this);
}


bool ControlledScene::__overlap(PxGeometry& geometry, PxTransform& pose, PxOverlapBuffer& result, int layerMask, bool castAll)
{

	PxFilterData filterData;
	filterData.word0 = layerMask;
	//overlap can't ues PxQueryHitType::eBLOCK
	//filterData.word1 = castAll ? 1 : 0;
	//PxQueryFlags flags = PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC | PxQueryFlag::ePREFILTER;
	PxQueryFlags flags = PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC;
	if (!castAll)
	{
		flags |= PxQueryFlag::eANY_HIT;
	}
	PxQueryFilterData Queryfilter(filterData, flags);

	return mScene->overlap(geometry, pose, result, Queryfilter, this);
}

void ControlledScene::__sweepBufferToPxRaycastHit(PxSweepBuffer& rayHit, PxRaycastHit* hitInfo)
{
	auto hit = rayHit.block;
	hitInfo->actor = hit.actor;
	hitInfo->shape = hit.shape;
	hitInfo->faceIndex = hit.faceIndex;
	hitInfo->flags = hit.flags;
	hitInfo->position = hit.position;
	hitInfo->normal = hit.normal;
	hitInfo->distance = hit.distance;

}


PxQueryHitType::Enum ControlledScene::preFilter(
	const PxFilterData& filterData, const PxShape* shape, const PxRigidActor* actor, PxHitFlags& queryFlags)
{
	auto shapeFilterData = shape->getQueryFilterData();
	if (shapeFilterData.word0 & filterData.word0)
	{
		if (filterData.word1 == 0)
		{
			return PxQueryHitType::eBLOCK;
		}
		return PxQueryHitType::eTOUCH;
	}
	return PxQueryHitType::eNONE;
}

PxQueryHitType::Enum ControlledScene::postFilter(const PxFilterData& filterData, const PxQueryHit& hit)
{
	return PxQueryHitType::eTOUCH;
}

#pragma endregion

void ControlledScene::__clearCache()
{
	mListCollision.clear();
	mListTrigger.clear();
}
