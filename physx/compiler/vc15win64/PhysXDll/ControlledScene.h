#pragma once

#ifndef CONTROLLEDSCENE
#define CONTROLLEDSCENE


#include <map>
#include <vector>
#include "PxPhysicsAPI.h"
#include "PxSimulationEventCallback.h"
#include "extensions/PxExtensionsAPI.h"
#include "PhysXPInvokeHelper.h"



using namespace physx;

class ControlledScene:  
	public PxSimulationEventCallback,
	public PxQueryFilterCallback
{
public:
	ControlledScene(PxPhysics* physics, PxScene* scene);
	~ControlledScene();
	PxReal*									mSleepThreshold;
	PxReal*									mDefaultContactOffset;
	PxReal*									mDefaultSolverIterations;
	PxReal*									mDefaultSolverVelocityIterations;


public: // Implement PxSimulationEventCallback
	void onConstraintBreak(PxConstraintInfo* constraints, PxU32 count) { PX_UNUSED(constraints); PX_UNUSED(count); }
	void onWake(PxActor** actors, PxU32 count) { PX_UNUSED(actors); PX_UNUSED(count); }
	void onSleep(PxActor** actors, PxU32 count) { PX_UNUSED(actors); PX_UNUSED(count); }
	void onAdvance(const PxRigidBody*const*, const PxTransform*, const PxU32) {}

	void onTrigger(PxTriggerPair* pairs, PxU32 count);
	void onContact(const PxContactPairHeader& pairHeader, const PxContactPair* pairs, PxU32 nbPairs);

public: // Implement PxQueryFilterCallback
	PxQueryHitType::Enum preFilter(
		const PxFilterData& filterData, const PxShape* shape, const PxRigidActor* actor, PxHitFlags& queryFlags);

	PxQueryHitType::Enum postFilter(const PxFilterData& filterData, const PxQueryHit& hit);
public:
	void release();
	void stepPhysicsScene(PxReal dt);

	PxRigidActor* createRigidbody(PxTransform tm, bool isStatic);
	PxRigidActor* changeRigidbodyStatic(PxRigidActor* actor, bool isStatic);
	void destroyRigidActor(PxRigidActor* rigidActor);

	PxShape* addBoxCollider(PxRigidActor* actor, int layer, PxVec3 center, PxVec3 size, bool isTrigger);
	PxShape* addCapsuleCollider(PxRigidActor* actor, int layer, PxVec3 center, PxReal radius, PxReal heigh, int direction, bool isTrigger);
	PxShape* addSphereCollider(PxRigidActor* actor, int layer, PxVec3 center, PxReal radius, bool isTrigger);

	void removeCollider(PxRigidActor* actor, PxShape* shape);

	void setColliderPosition(PxRigidActor* rigidActor, PxVec3 pos);
	void setColliderQuaternion(PxRigidActor* rigidActor, PxQuat qua);

	/**
	\param[int] array
	\param[int] array count
	*/
	void syncTransforms(PxActorSync* tranforms, uint32_t count);

	/**
	\param[out] array
	\param[out] array count
	*/
	void fetchTransforms(PxActorSync* tranforms, uint32_t &count);

	/**
	\param[out] array
	\param[out] array count
	*/
	void fetchContacts(PxCollision* collisions, uint32_t &count);
	/**
	\param[out] array
	\param[out] array count
	*/
	void fetchTriggers(PxTrigger* triggers, uint32_t &count);

	bool raycast(PxVec3& origin, PxVec3& direction, PxReal maxDistance, PxI32 layerMask);
	bool raycast(PxVec3 origin, PxVec3 direction, PxRaycastHit& hitInfo, float maxDistance, int layerMask);
	int raycastNonAlloc(PxVec3 origin, PxVec3 direction, PxRaycastBuffer& rayHit, float maxDistance, int layerMask);

	bool sphereCast(PxVec3 origin, float radius, PxVec3 direction, PxRaycastHit& hitInfo, float maxDistance, int layerMask);
	int sphereCast(PxVec3 origin, float radius, PxVec3 direction, PxSweepBuffer& hitInfo, float maxDistance, int layerMask);

	bool boxCast(PxVec3 center, PxVec3 halfExtents, PxVec3 direction, PxRaycastHit& hitInfoOut, PxQuat orientation, float maxDistance, int layerMask);
	int boxCastNonAlloc(PxVec3 center, PxVec3 halfExtents, PxVec3 direction, PxQuat orientation, PxSweepBuffer& hitInfo,float maxDistance, int layerMask);

	bool capsuleCast(PxVec3 point1, PxVec3 point2, float radius, PxVec3 direction, PxRaycastHit& hitInfoOut, float maxDistance, int layerMask);
	int capsuleCastNonAlloc(PxVec3 point1, PxVec3 point2, float radius, PxVec3 direction, PxSweepBuffer& hitInfo, float maxDistance, int layerMask);

	bool overlapSphere(PxVec3 origin, float radius, PxOverlapBuffer& result, int layerMask);
	bool overlapBoxNonAlloc(PxVec3 center, PxVec3 halfExtents, PxQuat orientation, PxOverlapBuffer& result, int layerMask);
	bool overlapCapsuleNonAlloc(PxVec3 point0, PxVec3 point1, PxReal radius, PxOverlapBuffer& result, int layerMask);

private:
	void __clearCache();
	void __initShape(PxShape* shape, int layer, bool isTrigger);
	bool __sweep(PxGeometry& geometry, PxTransform& pose, PxVec3 direction, PxSweepBuffer& rayHit, float maxDistance, int layerMask);
	bool __overlap(PxGeometry& geometry, PxTransform& pose, PxOverlapBuffer& result, int layerMask);
	void __sweepBufferToPxRaycastHit(PxSweepBuffer& rayHit, PxRaycastHit& hitInfo);
private:
	std::vector<PxCollision>		mListCollision;
	std::vector<PxTrigger>			mListTrigger;
	PxPhysics*						mPhysics;
	PxScene*						mScene;
};

#endif