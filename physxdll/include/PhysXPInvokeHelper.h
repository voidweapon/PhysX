#ifndef PINVOKE_HELPER
#define PINVOKE_HELPER

#include "extensions/PxExtensionsAPI.h"
#include "PxSimulationEventCallback.h"

using namespace physx;

struct PxVec3P
{
public:
	float x;
	float y;
	float z;
};


struct PxQuatP
{
public:
	float x;
	float y;
	float z;
	float w;
};

struct PxTransformP
{
public:
	PxQuatP q;
	PxVec3P p;
};

struct PxActorSync
{
public:
	PxRigidActor* actor;
	PxTransformP tm;
};

struct PxCollision
{
public:
	const PxContactPair* contactPair;
	PxRigidActor* actor0;
	PxRigidActor* actor1;
	PxShape* shape0;
	PxShape* shape1;
	int status;
	int contactCount;

};

struct PxTrigger
{
public:
	PxRigidActor* triggerActor;
	PxRigidActor* otherActor;
	int status;
};

struct PxRaycastHitP
{
public:
	PxRigidActor* actor;
	PxShape* shape;

	uint32_t faceIndex;

	uint16_t flags;
	PxVec3 position;
	PxVec3 normal;
	float distance;

	float u;
	float v;
};

struct  PxActorShapeP
{
public:
	PxRigidActor* actor;
	PxShape* shape;
};

PxVec3 CopyToPxVec3(PxVec3P& from, PxVec3 to);
PxQuat CopyToPxQuat(PxQuatP& from, PxQuat to);

PxTransformP ConvertToP(PxTransform &pTm);
PxTransform ConvertToPx(PxTransformP &pTm);

PxRaycastHitP ConvertToHitP(PxRaycastHit &hitInfo);
PxRaycastHitP ConvertToHitP(PxSweepHit &hitInfo);
PxActorShapeP ConvertToASP(PxOverlapHit& hitInfo);
PxQuat FromToRotation(PxVec3 &a, PxVec3 &b);
#endif