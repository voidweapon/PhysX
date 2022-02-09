#include "PhysXDll.h"


extern "C" 
{
	PhysXManager* _manager = NULL;

	PxTransformP DefaultTm;

	PxRaycastHit raycastCache[65535];
	PxSweepHit sweepCache[65535];
	PxOverlapHit overlapCache[65535];

	bool initPhysics(bool* collisionTable)
	{
		DefaultTm = PxTransformP();
		DefaultTm.p = PxVec3();
		DefaultTm.q = PxQuat();
		DefaultTm.p.x = 0.0f;
		DefaultTm.p.y = 0.0f;
		DefaultTm.p.z = 0.0f;
		DefaultTm.q.x = 0.0f;
		DefaultTm.q.y = 0.0f;
		DefaultTm.q.z = 0.0f;
		DefaultTm.q.w = 0.0f;

		_manager = new PhysXManager();

		return _manager->onInit(collisionTable);
	}

	void sleepThreshold(float threshold)
	{
		if (!_manager) return;

		_manager->setSleepThreshold(threshold);
	}

	void bounceThreshold(float threshold)
	{
		if (!_manager) return;
		_manager->setBounceThreshold(threshold);
	}

	void defaultContactOffset(float contactOffset)
	{
		if (!_manager) return;
		_manager->setDefaultContactOffset(contactOffset);
	}

	void defaultSolverIterations(float minPositionIters)
	{
		if (!_manager) return;
		_manager->setDefaultSolverIterations(minPositionIters);
	 }
	void defaultSolverVelocityIterations(float minVelocityIters)
	{
		if (!_manager) return;
		_manager->setDefaultSolverVelocityIterations(minVelocityIters);
	}
	void frictionType(int type)
	{
		if (!_manager) return;
		_manager->setFrictionType((PxFrictionType::Enum) type);
	}

	void cleanupPhysics()
	{
		if (_manager) 
		{
			_manager->onShutdown();
			delete _manager;
		}
		_manager = NULL;
	}

	ControlledScene* createPhysicsScene()
	{
		if (!_manager) return nullptr;

		return _manager->createScene();
	}

	void destroyPhysicsScene(ControlledScene* scene)
	{
		if (!_manager) return;

		_manager->destroyScene(scene);
	}

	void stepPhysicsScene(ControlledScene* scene, float dt)
	{
		if (!_manager) return;

		scene->stepPhysicsScene(dt);
	}

	PxRigidActor* createRigidbody(ControlledScene* scene, PxTransformP tm, bool isStatic)
	{
		if (!_manager) return nullptr;
		return scene->createRigidbody(ConvertToPx(tm), isStatic);
	}

	void destroyRigidActor(ControlledScene* scene, PxRigidActor* actor)
	{
		if (!_manager) return;
		scene->destroyRigidActor(actor);
	}

	PxRigidActor* changeRigidbodyStatic(ControlledScene* scene,  PxRigidActor* actor, bool isStatic)
	{
		if (!_manager) return nullptr;
		return scene->changeRigidbodyStatic(actor, isStatic);
	}

	void removeCollider(PxRigidActor* actor, PxShape* shape)
	{
		if (!_manager) return;

		actor->detachShape(*shape);
	}

	PxShape* addBoxCollider(ControlledScene* scene, PxRigidActor* actor, int layer, PxVec3 center ,PxVec3 size,  bool isTrigger)
	{
		if (!_manager) return nullptr;

		return scene->addBoxCollider(actor, layer, center, size, isTrigger);
	}

	PxShape* addCapsuleCollider(ControlledScene* scene, PxRigidActor* actor, int layer, PxVec3 center, float radius, float heigh, int direction, bool isTrigger)
	{
		if (!_manager) return nullptr;

		return scene->addCapsuleCollider(actor, layer, center, radius, heigh, direction, isTrigger);
	}

	PxShape* addSphereCollider(ControlledScene* scene, PxRigidActor* actor, int layer, PxVec3 center, float radius, bool isTrigger)
	{
		if (!_manager) return nullptr;

		return scene->addSphereCollider(actor, layer, center, radius, isTrigger);
	}

	void setActorPosition(PxRigidActor* actor, PxVec3 pos)
	{
		if (!_manager) return;

		PxTransform tm = actor->getGlobalPose();
		tm.p = pos;
		actor->setGlobalPose(tm);
	}

	void setActorQuaternion(PxRigidActor* actor, PxQuat qua)
	{
		if (!_manager) return;

		PxTransform tm = actor->getGlobalPose();
		tm.q = qua;
		actor->setGlobalPose(tm);
	}

	void setMass(PxRigidActor* actor, PxReal mass)
	{
		if (!_manager) return;
		PxRigidBody* rigidbody = (PxRigidBody*)(actor);
		if (!rigidbody) return;

		PxRigidBodyExt::updateMassAndInertia(*rigidbody, mass);
	}
	void setDrag(PxRigidActor* actor, PxReal drag)
	{
		if (!_manager) return;
		PxRigidBody* rigidbody = (PxRigidBody*)(actor);
		if (!rigidbody) return;

		rigidbody->setLinearDamping(drag);
	}
	void setAngularDrag(PxRigidActor* actor, PxReal angularDrag)
	{
		if (!_manager) return;
		PxRigidBody* rigidbody = (PxRigidBody*)(actor);
		if (!rigidbody) return;

		rigidbody->setAngularDamping(angularDrag);
	}
	void setAngularVelocity(PxRigidActor* actor, PxVec3 angularVelocity)
	{
		if (!_manager) return;
		PxRigidBody* rigidbody = (PxRigidBody*)(actor);
		if (!rigidbody) return;

		rigidbody->setAngularVelocity(angularVelocity);
	}
	void getAngularVelocity(PxRigidActor* actor, PxVec3& angularVelocity)
	{
		angularVelocity = PxVec3();
		if (!_manager) return;
		PxRigidBody* rigidbody = (PxRigidBody*)(actor);
		if (!rigidbody) return;

		angularVelocity = rigidbody->getAngularVelocity();
	}
	void setUseGravity(PxRigidActor* actor, bool useGravity)
	{
		if (!_manager) return;
		PxRigidBody* rigidbody = (PxRigidBody*)(actor);
		if (!rigidbody) return;

		rigidbody->setActorFlag(PxActorFlag::eDISABLE_GRAVITY, !useGravity);
	}
	void setIsKinematic(PxRigidActor* actor, bool isKinematic)
	{
		if (!_manager) return;
		PxRigidBody* rigidbody = (PxRigidBody*)(actor);
		if (!rigidbody) return;
		rigidbody->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, isKinematic);
	}
	void setVelocity(PxRigidActor* actor, PxVec3 velocity)
	{
		if (!_manager) return;
		PxRigidBody* rigidbody = (PxRigidBody*)(actor);
		if (!rigidbody) return;
		rigidbody->setLinearVelocity(velocity);
	}
	void getVelocity(PxRigidActor* actor, PxVec3& velocity)
	{
		velocity = PxVec3();

		if (!_manager) return;
		PxRigidBody* rigidbody = (PxRigidBody*)(actor);
		if (!rigidbody) return;

		velocity = rigidbody->getLinearVelocity();
	}
	void setConstraints(PxRigidActor* actor, int constraints)
	{
		if (!_manager) return;
		PxRigidDynamic* rigidbody = (PxRigidDynamic*)(actor);
		if (!rigidbody) return;

		if (constraints == 1)
		{
			//None
			rigidbody->setRigidDynamicLockFlags(PxRigidDynamicLockFlags());
		}
		else if(constraints == 128)
		{
			//FreezeAll
			PxRigidDynamicLockFlags all = 
				PxRigidDynamicLockFlag::eLOCK_ANGULAR_X | PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y
				| PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z | PxRigidDynamicLockFlag::eLOCK_LINEAR_X
				| PxRigidDynamicLockFlag::eLOCK_LINEAR_Y | PxRigidDynamicLockFlag::eLOCK_LINEAR_Z;
			rigidbody->setRigidDynamicLockFlags(all);
		}
		else if (constraints == 14)
		{
			PxRigidDynamicLockFlags freezePosition = PxRigidDynamicLockFlag::eLOCK_LINEAR_X
				| PxRigidDynamicLockFlag::eLOCK_LINEAR_Y | PxRigidDynamicLockFlag::eLOCK_LINEAR_Z;
			rigidbody->setRigidDynamicLockFlags(freezePosition);
		}
		else if (constraints == 112)
		{
			PxRigidDynamicLockFlags freezeRotation = PxRigidDynamicLockFlag::eLOCK_ANGULAR_X 
				| PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y| PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z;
			rigidbody->setRigidDynamicLockFlags(freezeRotation);
		}
		else
		{
			PxRigidDynamicLockFlags lock;
			if (constraints & 2)
			{
				lock |= PxRigidDynamicLockFlag::eLOCK_LINEAR_X;
			}
			if (constraints & 4)
			{
				lock |= PxRigidDynamicLockFlag::eLOCK_LINEAR_Y;
			}
			if (constraints & 8)
			{
				lock |= PxRigidDynamicLockFlag::eLOCK_LINEAR_Z;
			}

			if (constraints & 16)
			{
				lock |= PxRigidDynamicLockFlag::eLOCK_ANGULAR_X;
			}
			if (constraints & 32)
			{
				lock |= PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y;
			}
			if (constraints & 64)
			{
				lock |= PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z;
			}
			rigidbody->setRigidDynamicLockFlags(lock);
		}
	}

	void setCollisionDetectionMode(PxRigidActor* actor, int collisionDetectionMode)
	{
		if (!_manager) return;
		PxRigidDynamic* rigidbody = (PxRigidDynamic*)(actor);
		if (!rigidbody) return;

		if (collisionDetectionMode == 0)
		{
			//Discrete
			rigidbody->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, false);
			rigidbody->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD_FRICTION, false);
			rigidbody->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, false);
		}
		else if (collisionDetectionMode == 1)
		{
			//Continuous
			//Continuous collision detection is on for colliding with static mesh geometry.
			rigidbody->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			rigidbody->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD_FRICTION, true);
			rigidbody->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, false);
		}
		else if (collisionDetectionMode == 2)
		{
			//ContinuousDynamic
			//Continuous collision detection is on for colliding with static and dynamic geometry.
			rigidbody->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, true);
			rigidbody->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD_FRICTION, true);
			rigidbody->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, false);
		}
		else
		{
			//ContinuousSpeculative
			//Speculative continuous collision detection is on for static and dynamic geometries
			rigidbody->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD, false);
			rigidbody->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_CCD_FRICTION, true);
			rigidbody->setRigidBodyFlag(PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD, true);
		}
	}
	void setLayer(PxRigidActor* actor, int layer)
	{
		if (!_manager) return;
		PxRigidDynamic* rigidbody = (PxRigidDynamic*)(actor);
		if (!rigidbody) return;

		PxU32 count = rigidbody->getNbShapes();
		for (size_t i = 0; i < count; i++)
		{
			PxShape* shape;
			rigidbody->getShapes(&shape, 1, i);
			PxFilterData simulateFilter;
			simulateFilter.word0 = layer;
			PxFilterData queryFilter;
			queryFilter.word0 = 1 << layer;

			shape->setQueryFilterData(queryFilter);
			shape->setSimulationFilterData(simulateFilter);
		}
	}


	void addForce(void* actor, PxVec3 force, int mode)
	{
		if (!_manager) return;

		PxRigidActor* pxActor = (PxRigidActor*)(actor);
		PxRigidDynamic* rigidbody = (PxRigidDynamic*)(actor);
		if (!rigidbody) return;

		rigidbody->addForce(force, (PxForceMode::Enum)mode);
	}

	void syncTransforms(ControlledScene* scene, PxActorSync* tranforms, uint32_t count)
	{
		if (!_manager) return;
		scene->syncTransforms(tranforms, count);
	}

	void fetchTransforms(ControlledScene* scene, PxActorSync* tranforms, uint32_t &count)
	{
		count = 0;
		if (!_manager) return;
		scene->fetchTransforms(tranforms, count);
	}

	void fetchContacts(ControlledScene* scene, PxCollision* collisions, uint32_t &count)
	{
		count = 0;
		if (!_manager) return;
		scene->fetchContacts(collisions, count);
	}

	void fetchTriggers(ControlledScene* scene, PxTrigger* triggers, uint32_t &count)
	{
		count = 0;
		if (!_manager) return;
		scene->fetchTriggers(triggers, count);
	}


	bool raycast(ControlledScene* scene, PxVec3 origin, PxVec3 direction, float maxDistance, int layerMask)
	{
		if (!_manager) return false;

		return scene->raycast(origin, direction, maxDistance, layerMask);
	}

	bool raycast2(ControlledScene* scene, PxVec3 origin, PxVec3 direction, PxRaycastHitP& hitInfoOut, float maxDistance, int layerMask)
	{
		if (!_manager) return false;

		PxRaycastHit hitInfo;
		bool hit =  scene->raycast(origin, direction, hitInfo, maxDistance, layerMask);

		hitInfoOut = ConvertToHitP(hitInfo);

		return hit;
	}

	int raycastNonAlloc(ControlledScene* scene, PxVec3 origin, PxVec3 direction, float maxDistance, int layerMask, PxRaycastHitP* hitInfoOut, int maxCount)
	{
		if (!_manager) return 0;

		if (maxCount > 65535)
		{
			maxCount = 65535;
		}

		PxRaycastBuffer rayHit(&raycastCache[0], maxCount);
		PxU32 count = scene->raycastNonAlloc(origin, direction, rayHit, maxDistance, layerMask);

		for (PxU32 i = 0; i < count; i++)
		{
			PxRaycastHit hitInfo = rayHit.getAnyHit(i);
			hitInfoOut[i] = ConvertToHitP(hitInfo);
		}

		return count;
	}

	bool sphereCast(ControlledScene* scene, PxVec3 origin, float radius, PxVec3 direction, PxRaycastHitP& hitInfoOut, float maxDistance, int layerMask)
	{
		if (!_manager) return false;

		PxRaycastHit hitInfo;
		bool ret = scene->sphereCast(origin, radius, direction, hitInfo, maxDistance, layerMask);

		hitInfoOut = ConvertToHitP(hitInfo);

		return ret;
	}
	int sphereCastNonAlloc(ControlledScene* scene, PxVec3 origin, float radius, PxVec3 direction,  float maxDistance, int layerMask, PxRaycastHitP* hitInfoOut, int maxCount)
	{
		if (!_manager) return 0;
		if (maxCount > 65535)
		{
			maxCount = 65535;
		}

		PxSweepBuffer rayHit(&sweepCache[0], maxCount);
		PxU32 count = scene->sphereCast(origin, radius, direction, rayHit, maxDistance, layerMask);

		for (PxU32 i = 0; i < count; i++)
		{
			PxSweepHit hitInfo = rayHit.getAnyHit(i);
			hitInfoOut[i] = ConvertToHitP(hitInfo);
		}

		return count;
	}

	bool boxCast(ControlledScene* scene, PxVec3 center, PxVec3 halfExtents, PxVec3 direction, PxRaycastHitP& hitInfoOut, PxQuat orientation, float maxDistance, int layerMask)
	{
		if (!_manager) return false;

		PxRaycastHit hitInfo;
		bool ret = scene->boxCast(center, halfExtents, direction, hitInfo, orientation, maxDistance, layerMask);

		hitInfoOut = ConvertToHitP(hitInfo);

		return ret;
	}
	int boxCastNonAlloc(ControlledScene* scene, PxVec3 center, PxVec3 halfExtents, PxVec3 direction, PxQuat orientation, float maxDistance, int layerMask, PxRaycastHitP* hitInfoOut, int maxCount)
	{
		if (!_manager) return 0;
		if (maxCount > 65535)
		{
			maxCount = 65535;
		}
		PxSweepBuffer rayHit(&sweepCache[0], maxCount);
		PxU32 count = scene->boxCastNonAlloc(center, halfExtents, direction, orientation, rayHit, maxDistance, layerMask);

		for (PxU32 i = 0; i < count; i++)
		{
			PxSweepHit hitInfo = rayHit.getAnyHit(i);
			hitInfoOut[i] = ConvertToHitP(hitInfo);
		}

		return count;
	}

	bool capsuleCast(ControlledScene* scene, PxVec3 point1, PxVec3 point2, float radius, PxVec3 direction, PxRaycastHitP& hitInfoOut, float maxDistance, int layerMask)
	{
		if (!_manager) return false;
				PxRaycastHit hitInfo;

		bool ret = scene->capsuleCast(point1, point2, radius, direction, hitInfo, maxDistance, layerMask);

		hitInfoOut = ConvertToHitP(hitInfo);

		return ret;
	}
	int capsuleCastNonAlloc(ControlledScene* scene, PxVec3 point1, PxVec3 point2, float radius, PxVec3 direction, float maxDistance, int layerMask, PxRaycastHitP* hitInfoOut, int maxCount)
	{
		if (!_manager) return 0;
		if (maxCount > 65535)
		{
			maxCount = 65535;
		}

		PxSweepBuffer rayHit(&sweepCache[0], maxCount);
		PxU32 count = scene->capsuleCastNonAlloc(point1, point2, radius, direction, rayHit, maxDistance, layerMask);

		for (PxU32 i = 0; i < count; i++)
		{
			PxSweepHit hitInfo = rayHit.getAnyHit(i);
			hitInfoOut[i] = ConvertToHitP(hitInfo);
		}
		return count;
	}

	int overlapSphereNonAlloc(ControlledScene* scene, PxVec3 origin, float radius, int layerMask, PxActorShapeP* result, int maxCount)
	{
		if (!_manager) return 0;

		if (maxCount > 65535)
		{
			maxCount = 65535;
		}

		PxOverlapBuffer hitResult(overlapCache, maxCount);
		scene->overlapSphere(origin, radius, hitResult, layerMask);

		size_t count = hitResult.nbTouches;
		for (size_t i = 0; i < count; i++)
		{
			PxOverlapHit hitInfo = hitResult.getAnyHit(i);
			result[i] = ConvertToASP(hitInfo);
		}

		return count;
	}

	int overlapBoxNonAlloc(ControlledScene* scene, PxVec3 center, PxVec3 halfExtents,  PxQuat orientation, int mask, PxActorShapeP* results, int maxCount)
	{
		if (!_manager) return 0;

		if (maxCount > 65535)
		{
			maxCount = 65535;
		}

		PxOverlapBuffer hitResult(overlapCache, maxCount);
		scene->overlapBoxNonAlloc(center, halfExtents, orientation, hitResult, mask);

		size_t count = hitResult.nbTouches;
		for (size_t i = 0; i < count; i++)
		{
			PxOverlapHit hitInfo = hitResult.getAnyHit(i);
			results[i] = ConvertToASP(hitInfo);
		}

		return count;
	}
	int overlapCapsuleNonAlloc(ControlledScene* scene, PxVec3 point0, PxVec3 point1,  float radius, int mask, PxActorShapeP* results, int maxCount)
	{
		if (!_manager) return 0;

		if (maxCount > 65535)
		{
			maxCount = 65535;
		}

		PxOverlapBuffer hitResult(overlapCache, maxCount);
		scene->overlapCapsuleNonAlloc(point0, point1, radius, hitResult, mask);

		size_t count = hitResult.nbTouches;
		for (size_t i = 0; i < count; i++)
		{
			PxOverlapHit hitInfo = hitResult.getAnyHit(i);
			results[i] = ConvertToASP(hitInfo);
		}

		return count;
	}
}

