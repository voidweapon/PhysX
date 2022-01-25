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

PxQuat FromToRotation(PxVec3 &a, PxVec3 &b)
{
	PxVec3 start = a.getNormalized();
	PxVec3 dest = b.getNormalized();
	PxReal cosTheta = start.dot(dest);
	PxVec3 rotationAxis;
	PxQuat quaternion;
	if (cosTheta < -1 + 0.001f)
	{
		rotationAxis = PxVec3(0.0f, 0.0f, 1.0f).cross(start);
		if (rotationAxis.magnitudeSquared() < 0.01f)
		{
			rotationAxis = PxVec3(1.0f, 0.0f, 0.0f).cross(start);
		}
		rotationAxis.normalize();
		quaternion = PxQuat(PxPi, rotationAxis);
		quaternion.normalize();
		return quaternion;
	}

	rotationAxis = start.cross(dest);
	PxReal s = PxSqrt((1 + cosTheta) * 2);
	PxReal invs = 1 / s;
	
	quaternion = PxQuat(rotationAxis.x * invs, rotationAxis.y * invs, rotationAxis.z * invs, s * 0.5f);
	quaternion.normalize();
	return quaternion;
}