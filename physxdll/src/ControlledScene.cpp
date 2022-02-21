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
	mPhysics = NULL;
	mScene = NULL;
	mDefaultMaterial = NULL;
}

//=================================== Simulation Event ======================================
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
	PX_UNUSED(pairHeader);

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
//===================================

void ControlledScene::release()
{
	if (!mScene)
	{
		return;
	}
	mScene->release();
	mScene = NULL;
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

//=================================== RigidActor ===================================

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
	PxRigidActor* actor = NULL;
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
	PxTransform tm = actor->getGlobalPose();
	PxRigidActor* newActor = NULL;

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

PxShape* ControlledScene::addBoxCollider(PxRigidActor* actor, int layer, PxVec3P center, PxVec3P size, bool isTrigger)
{
	PxMaterial* material = mDefaultMaterial;
	PxShape* shape = mPhysics->createShape(PxBoxGeometry(size.x * 0.5f, size.y * 0.5f, size.z * 0.5f), *material, true);

	PxTransform relativePose(center.x, center.y, center.z);
	shape->setLocalPose(relativePose);

	this->__initShape(shape, layer, isTrigger);

	actor->attachShape(*shape);
	shape->release();

	return shape;
}

PxShape* ControlledScene::addCapsuleCollider(PxRigidActor* actor, int layer, PxVec3P center, PxReal radius, PxReal heigh, int direction, bool isTrigger)
{
	PxMaterial* material = mDefaultMaterial;
	heigh = (heigh - radius * 2.0f) * 0.5f;
	PxShape* shape =  PxRigidActorExt::createExclusiveShape(*actor, PxCapsuleGeometry(radius, heigh), *material);


	if (direction == 0)
	{
		//X-axes
		PxTransform relativePose_X(center.x, center.y, center.z);
		shape->setLocalPose(relativePose_X);
	}
	else if(direction == 1)
	{
		//Y-axes
		PxTransform relativePose_Y(center.x, center.y, center.z, PxQuat(PxHalfPi, PxVec3(0, 0, 1)));
		shape->setLocalPose(relativePose_Y);
	}
	else if (direction == 2)
	{
		//Z-axes
		PxTransform relativePose_Z(center.x, center.y, center.z, PxQuat(PxHalfPi, PxVec3(0, 1, 0)));
		shape->setLocalPose(relativePose_Z);
	}

	this->__initShape(shape, layer, isTrigger);

	return shape;
}

PxShape* ControlledScene::addSphereCollider(PxRigidActor* actor, int layer, PxVec3P center, PxReal radius, bool isTrigger)
{
	PxMaterial* material = mDefaultMaterial;
	PxShape* shape = PxRigidActorExt::createExclusiveShape(*actor, PxSphereGeometry(radius), *material);

	PxTransform relativePose(center.x, center.y, center.z);
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

//===================================


//=================================== Change Single RigidActror Transform ===================================
void ControlledScene::setColliderPosition(PxRigidActor* rigidActor, PxVec3P pos)
{
	if (!mScene || !rigidActor)
	{
		return;
	}
	PxTransform tm = rigidActor->getGlobalPose();
	tm.p.x = pos.x;
	tm.p.y = pos.y;
	tm.p.z = pos.z;
	rigidActor->setGlobalPose(tm);
}
void ControlledScene::setColliderQuaternion(PxRigidActor* rigidActor, PxQuatP qua)
{
	if (!mScene || !rigidActor)
	{
		return;
	}

	PxTransform tm = rigidActor->getGlobalPose();
	tm.q.x = qua.x;
	tm.q.y = qua.y;
	tm.q.z = qua.z;
	tm.q.w = qua.w;
	rigidActor->setGlobalPose(tm);
}
//===================================

//=================================== Sync Data ===================================
void ControlledScene::syncTransforms(PxActorSync* tranforms, uint32_t count)
{
	for (size_t i = 0; i < count; i++)
	{
		PxRigidActor* actor = tranforms[i].actor;
		PxTransform tm = actor->getGlobalPose();
		PxVec3P p = tranforms[i].tm.p;
		PxQuatP q = tranforms[i].tm.q;
		tm.p.x = p.x;
		tm.p.y = p.y;
		tm.p.z = p.z;

		tm.q.x = q.x;
		tm.q.y = q.y;
		tm.q.z = q.z;
		tm.q.w = q.w;

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

		PxVec3P p;
		p.x = tm.p.x;
		p.y = tm.p.y;
		p.z = tm.p.z;

		PxQuatP q;
		q.x = tm.q.x;
		q.y = tm.q.y;
		q.z = tm.q.z;
		q.w = tm.q.w;

		actorSync.tm.p = p;
		actorSync.tm.q = q;

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
//===================================

//=================================== Raycast ===================================
bool ControlledScene::raycast(PxVec3&  origin, PxVec3&  direction, PxReal maxDistance, PxI32 layerMask)
{
	PxRaycastBuffer rayHit;

	PxHitFlags hitFlags = (PxHitFlags)(PxHitFlag::eDEFAULT);
	PxFilterData filterData;
	filterData.word0 = layerMask;
	filterData.word1 = 0;
	PxQueryFilterData Queryfilter(filterData, PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC | PxQueryFlag::ePREFILTER);
	
	bool hit =  mScene->raycast(origin, direction, maxDistance, rayHit, hitFlags, Queryfilter, this);

	return hit;
}

bool ControlledScene::raycast(PxVec3& origin, PxVec3& direction, PxRaycastHit& hitInfo, float maxDistance, int layerMask)
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

PxU32 ControlledScene::raycastNonAlloc(PxVec3& origin, PxVec3& direction, PxRaycastBuffer& rayHit, float maxDistance, int layerMask)
{
	PxHitFlags hitFlags = (PxHitFlags)(PxHitFlag::eDEFAULT);
	PxFilterData filterData;
	filterData.word0 = layerMask;
	filterData.word1 = 1;
	PxQueryFilterData Queryfilter(filterData, PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC | PxQueryFlag::ePREFILTER);

	bool hit = mScene->raycast(origin, direction, maxDistance, rayHit, hitFlags, Queryfilter, this);

	return  rayHit.getNbAnyHits();
}


bool ControlledScene::sphereCast(PxVec3& origin, float radius, PxVec3& direction, PxRaycastHit& hitInfo, float maxDistance, int layerMask)
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
PxU32 ControlledScene::sphereCast(PxVec3& origin, float radius, PxVec3& direction, PxSweepBuffer& hitInfo, float maxDistance, int layerMask)
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

bool ControlledScene::boxCast(PxVec3& center, PxVec3& halfExtents, PxVec3& direction, PxRaycastHit& hitInfoOut, PxQuat& orientation, float maxDistance, int layerMask)
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
PxU32 ControlledScene::boxCastNonAlloc(PxVec3& center, PxVec3& halfExtents, PxVec3& direction, PxQuat& orientation, PxSweepBuffer& hitInfo,float maxDistance, int layerMask)
{
	/*
	* Issue #471 https://github.com/NVIDIAGameWorks/PhysX/issues/471
	* Don't use = operate or copy constructor for create Geometry for overlap
	*/
	PxBoxGeometry box(halfExtents);
	PxTransform tm(center, orientation);
	this->__sweep(box, tm, direction, hitInfo, maxDistance, layerMask, true);

	return hitInfo.getNbAnyHits();
}

bool ControlledScene::capsuleCast(PxVec3& point1, PxVec3& point2, float radius, PxVec3& direction, PxRaycastHit& hitInfoOut, float maxDistance, int layerMask)
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
PxU32 ControlledScene::capsuleCastNonAlloc(PxVec3& point1, PxVec3& point2, float radius, PxVec3& direction, PxSweepBuffer& hitInfo, float maxDistance, int layerMask)
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

PxU32 ControlledScene::overlapSphere(PxVec3& origin, float radius, PxOverlapBuffer& result, int layerMask)
{
	/* 
	* Issue #471 https://github.com/NVIDIAGameWorks/PhysX/issues/471
	* Don't use = operate or copy constructor for create Geometry for overlap
	*/
	PxSphereGeometry sphere(radius);
	PxTransform tm(origin);
	return this->__overlap(sphere, tm, result, layerMask, true);
}
PxU32 ControlledScene::overlapBoxNonAlloc(PxVec3& center, PxVec3& halfExtents, PxQuat& orientation, PxOverlapBuffer& result, int layerMask)
{
	/*
	* Issue #471 https://github.com/NVIDIAGameWorks/PhysX/issues/471
	* Don't use = operate or copy constructor for create Geometry for overlap
	*/
	PxBoxGeometry box(halfExtents);
	PxTransform tm(center, orientation);
	return this->__overlap(box, tm, result, layerMask, true);
}
PxU32 ControlledScene::overlapCapsuleNonAlloc(PxVec3& point0, PxVec3& point1, PxReal radius, PxOverlapBuffer& result, int layerMask)
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


bool ControlledScene::__sweep(PxGeometry& geometry, PxTransform& pose, PxVec3& direction, PxSweepBuffer& rayHit, float maxDistance, int layerMask, bool castAll)
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


PxU32 ControlledScene::__overlap(PxGeometry& geometry, PxTransform& pose, PxOverlapBuffer& result, int layerMask, bool castAll)
{

	PxFilterData filterData(layerMask, 0,0,0);
	filterData.word0 = layerMask;
	//overlap can't ues PxQueryHitType::eBLOCK
	filterData.word1 = castAll ? 1 : 0;
	//PxQueryFlags flags = PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC | PxQueryFlag::ePREFILTER;
	PxQueryFlags flags = PxQueryFlag::eNO_BLOCK | PxQueryFlag::eSTATIC | PxQueryFlag::eDYNAMIC | PxQueryFlag::ePREFILTER;
	//if (!castAll)
	//{
	//	flags |= PxQueryFlag::eANY_HIT;
	//}
	PxQueryFilterData Queryfilter(filterData, flags);

	bool ret = false;
	if (mScene->overlap(geometry, pose, result, Queryfilter, this)) 
	{
		ret = true;
	}

	return result.nbTouches;
}

void ControlledScene::__sweepBufferToPxRaycastHit(PxSweepBuffer& rayHit, PxRaycastHit* hitInfo)
{
	physx::PxSweepHit hit = rayHit.block;
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
	PX_UNUSED(actor);
	PX_UNUSED(queryFlags);
	physx::PxFilterData shapeFilterData = shape->getQueryFilterData();
	if (shapeFilterData.word0 & filterData.word0)
	{
		if (filterData.word1 == 0)
		{
			return PxQueryHitType::eBLOCK;
		}
		else
		{
			return PxQueryHitType::eTOUCH;
		}

	}
	else
	{
		return PxQueryHitType::eNONE;
	}
}

PxQueryHitType::Enum ControlledScene::postFilter(const PxFilterData& filterData, const PxQueryHit& hit)
{
	PX_UNUSED(filterData);
	PX_UNUSED(hit);
	return PxQueryHitType::eTOUCH;
}

//===================================

void ControlledScene::__clearCache()
{
	mListCollision.clear();
	mListTrigger.clear();
}
