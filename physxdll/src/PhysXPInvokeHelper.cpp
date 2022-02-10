#include "PhysXPInvokeHelper.h"

PxVec3 CopyToPxVec3(PxVec3P& from, PxVec3 to)
{
	to.x = from.x;
	to.y = from.y;
	to.z = from.z;

	return to;
}
PxQuat CopyToPxQuat(PxQuatP& from, PxQuat to)
{
	to.x = from.x;
	to.y = from.y;
	to.z = from.z;
	to.w = from.w;

	return to;
}

PxTransformP ConvertToP(PxTransform &pTm)
{
	PxTransformP tm;

	PxVec3P p;
	p.x = tm.p.x;
	p.y = tm.p.y;
	p.z = tm.p.z;

	PxQuatP q;
	q.x = tm.q.x;
	q.y = tm.q.y;
	q.z = tm.q.z;
	q.w = tm.q.w;

	tm.q = q;
	tm.p = p;

	return tm;
}

PxTransform ConvertToPx(PxTransformP &pTm)
{
	PxTransform tm(PxIDENTITY::PxIdentity);

	PxVec3P p = pTm.p;
	PxQuatP q = pTm.q;
	tm.p.x = p.x;
	tm.p.y = p.y;
	tm.p.z = p.z;

	tm.q.x = q.x;
	tm.q.y = q.y;
	tm.q.z = q.z;
	tm.q.w = q.w;

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