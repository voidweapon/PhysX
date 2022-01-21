#pragma once

#ifndef PHYSXDLL
#define PHYSXDLL

#include"PhysXManager.h"

#define DllExport  __declspec(dllexport)

using namespace physx;

extern "C" 
{
#pragma region Physics
	DllExport bool initPhysics(bool* collisionTable);
	DllExport void cleanupPhysics();
	DllExport void sleepThreshold(float threshold);
	DllExport void bounceThreshold(float threshold);
	DllExport void defaultContactOffset(float contactOffset);
	DllExport void defaultSolverIterations(float minPositionIters);
	DllExport void defaultSolverVelocityIterations(float minVelocityIters);
	DllExport void frictionType(int type);
#pragma endregion


#pragma region Scene
	DllExport ControlledScene* createPhysicsScene();

	DllExport void destroyPhysicsScene(ControlledScene* scene);

	DllExport void stepPhysicsScene(ControlledScene* scene, float dt);
#pragma endregion


#pragma region Actor
	DllExport PxRigidActor* createRigidbody(ControlledScene* scene, PxTransformP tm, bool isStatic);
	DllExport void destroyRigidActor(ControlledScene* scene, PxRigidActor* actor);
	DllExport PxRigidActor* changeRigidbodyStatic(ControlledScene* scene, PxRigidActor* actor, bool isStatic);
#pragma endregion


#pragma region set Actor Property
	DllExport void setActorPosition(PxRigidActor* actor, PxVec3 pos);
	DllExport void setActorQuaternion(PxRigidActor* actor, PxQuat qua);
	DllExport void setMass(PxRigidActor* actor, PxReal mass);
	DllExport void setDrag(PxRigidActor* actor, PxReal drag);
	DllExport void setAngularDrag(PxRigidActor* actor, PxReal angularDrag);
	DllExport void setAngularVelocity(PxRigidActor* actor, PxVec3 angularVelocity);
	DllExport void getAngularVelocity(PxRigidActor* actor, PxVec3& angularVelocity);
	DllExport void setUseGravity(PxRigidActor* actor, bool useGravity);
	DllExport void setIsKinematic(PxRigidActor* actor, bool isKinematic);
	DllExport void setVelocity(PxRigidActor* actor, PxVec3 velocity);
	DllExport void getVelocity(PxRigidActor* actor, PxVec3& velocity);
	DllExport void setConstraints(PxRigidActor* actor, int constraints);
	DllExport void setCollisionDetectionMode(PxRigidActor* actor, int collisionDetectionMode);
	DllExport void setLayer(PxRigidActor* actor, int layer);

#pragma region Collider
	DllExport void removeCollider(PxRigidActor* actor, PxShape* shape);
	DllExport PxShape* addBoxCollider(ControlledScene* scene, PxRigidActor* actor, int layer, PxVec3 center, PxVec3 size, bool isTrigger);
	DllExport PxShape* addCapsuleCollider(ControlledScene* scene, PxRigidActor* actor, int layer, PxVec3 center, float radius, float heigh, int direction, bool isTrigger);
	DllExport PxShape* addSphereCollider(ControlledScene* scene, PxRigidActor* actor, int layer, PxVec3 center, float radius, bool isTrigger);
#pragma endregion


#pragma region Sync Data / Event
	DllExport void syncTransforms(ControlledScene* scene, PxActorSync* tranforms, uint32_t count);

	DllExport void fetchTransforms(ControlledScene* scene, PxActorSync* tranforms, uint32_t &count);

	DllExport void fetchContacts(ControlledScene* scene, PxCollision* collisions, uint32_t &count);

	DllExport void fetchTriggers(ControlledScene* scene, PxTrigger* triggers, uint32_t &count);
#pragma endregion


#pragma region Scene Query
	DllExport bool raycast(ControlledScene* scene, PxVec3 origin, PxVec3 direction, float maxDistance, int layerMask);
	DllExport bool raycast2(ControlledScene* scene, PxVec3 origin, PxVec3 direction, PxRaycastHitP& hitInfoOut, float maxDistance, int layerMask);
	DllExport int raycastNonAlloc(ControlledScene* scene, PxVec3 origin, PxVec3 direction, float maxDistance, int layerMask, PxRaycastHitP* hitInfoOut, int maxCount);
	
	DllExport bool sphereCast(ControlledScene* scene, PxVec3 origin, float radius, PxVec3 direction, PxRaycastHitP& hitInfoOut, float maxDistance, int layerMask);
	DllExport int sphereCastNonAlloc(ControlledScene* scene, PxVec3 origin, float radius, PxVec3 direction, float maxDistance, int layerMask, PxRaycastHitP* hitInfoOut, int maxCount);

	DllExport bool boxCast(ControlledScene* scene, PxVec3 center, PxVec3 halfExtents, PxVec3 direction, PxRaycastHitP& hitInfoOut, PxQuat orientation, float maxDistance, int layerMask);
	DllExport int boxCastNonAlloc(ControlledScene* scene, PxVec3 center, PxVec3 halfExtents, PxVec3 direction, PxQuat orientation, float maxDistance, int layerMask, PxRaycastHitP* hitInfoOut, int maxCount);

	DllExport bool capsuleCast(ControlledScene* scene, PxVec3 point1, PxVec3 point2, float radius, PxVec3 direction, PxRaycastHitP& hitInfoOut, float maxDistance, int layerMask);
	DllExport int capsuleCastNonAlloc(ControlledScene* scene, PxVec3 point1, PxVec3 point2, float radius, PxVec3 direction, float maxDistance, int layerMask, PxRaycastHitP* hitInfoOut, int maxCount);

	DllExport int overlapSphereNonAlloc(ControlledScene* scene, PxVec3 origin, float radius, int layerMask, PxActorShapeP* result, int maxCount);
	DllExport int overlapBoxNonAlloc(ControlledScene* scene, PxVec3 center, PxVec3 halfExtents,  PxQuat orientation, int mask, PxActorShapeP* results, int maxCount);
	DllExport int overlapCapsuleNonAlloc(ControlledScene* scene, PxVec3 point0, PxVec3 point1,  float radius, int mask, PxActorShapeP* results, int maxCount);
#pragma endregion


#pragma region Force
	DllExport void addForce(void* actor, PxVec3 force, int mode);
#pragma endregion

}
#endif