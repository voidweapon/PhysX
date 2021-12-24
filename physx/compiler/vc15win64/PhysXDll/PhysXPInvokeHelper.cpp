#include "PhysXPInvokeHelper.h"

PxTransformP ConvertToP(PxTransform &pTm)
{
	PxTransformP tm;
	tm.q = pTm.q;
	tm.p = pTm.p;
	return tm;
}

PxTransform ConvertToPx(PxTransformP &pTm)
{
	PxTransform tm(pTm.p, pTm.q);
	return tm;
}

PxRaycastHitP ConvertToHitP(PxRaycastHit &hitInfo)
{
	PxRaycastHitP hit;
	hit.actor = hitInfo.actor;
	hit.shape = hitInfo.shape;
	hit.faceIndex = hitInfo.faceIndex;
	hit.flags = (uint16_t)hitInfo.flags;
	hit.position = hitInfo.position;
	hit.normal = hitInfo.normal;
	hit.distance = hitInfo.distance;
	hit.u = hitInfo.u;
	hit.v = hitInfo.v;
	return hit;
}


PxRaycastHitP ConvertToHitP(PxSweepHit &hitInfo)
{
	PxRaycastHitP hit;
	hit.actor = hitInfo.actor;
	hit.shape = hitInfo.shape;
	hit.faceIndex = hitInfo.faceIndex;
	hit.flags = (uint16_t)hitInfo.flags;
	hit.position = hitInfo.position;
	hit.normal = hitInfo.normal;
	hit.distance = hitInfo.distance;
	return hit;
}

PxActorShapeP ConvertToASP(PxOverlapHit& hitInfo)
{
	PxActorShapeP hit;
	hit.actor = hitInfo.actor;
	hit.shape = hitInfo.shape;

	return hit;
}