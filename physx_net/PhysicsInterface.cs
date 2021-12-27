using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using UnityEngine;

namespace ET
{
    public static class PhysicsInterface
    {
        private const string PhysXDLL = "PhysXDll";

        //============================================================================================================================
        #region Physics
        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern bool initPhysics([In] bool[] coliisionTable);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void cleanupPhysics();

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void sleepThreshold(float threshold);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void bounceThreshold(float threshold);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void defaultContactOffset(float contactOffset);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void defaultSolverIterations(float minPositionIters);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void defaultSolverVelocityIterations(float minVelocityIters);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void frictionType(int type);
        #endregion

        //============================================================================================================================
        #region Scene
        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createPhysicsScene();

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void destroyPhysicsScene(IntPtr scene);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void stepPhysicsScene(IntPtr scene, float dt);
        #endregion

        //============================================================================================================================
        #region Actor
        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr createRigidbody(IntPtr scene, PxTransformP tm, bool isTrigger);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void destroyRigidActor(IntPtr scene, IntPtr actor);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr changeRigidbodyStatic(IntPtr scene, IntPtr actor, bool isStatic);
        #endregion

        //============================================================================================================================
        #region Actor Property
        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void setActorPosition(IntPtr collider, PxVec3 pos);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void setActorQuaternion(IntPtr collider, PxQuat qua);
        #endregion

        //============================================================================================================================
        #region Collider

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void removeCollider(IntPtr actor, IntPtr shape);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr addBoxCollider(IntPtr scene, IntPtr actor, int layer, PxVec3 center, PxVec3 size, bool isTrigger);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr addCapsuleCollider(IntPtr scene, IntPtr actor, int layer, float radius, float height, int direction, bool isTrigger);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern IntPtr addSphereCollider(IntPtr scene, IntPtr actor, int layer, PxVec3 center, float radius, bool isTrigger);
        #endregion


        //============================================================================================================================
        #region Sync Data / Event
        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void syncTransforms(IntPtr scene, [In] PxActorSync[] tranforms, uint count);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void fetchTransforms(IntPtr scene, [Out] PxActorSync[] tranforms, ref uint count);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void fetchContacts(IntPtr scene, [Out] PxCollision[] collisions, ref uint count);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void fetchTriggers(IntPtr scene, [Out] PxTrigger[] triggers, ref uint count);
        #endregion

        //============================================================================================================================
        #region Scene Query
        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern bool raycast(IntPtr scene, PxVec3 origin, PxVec3 direction, float maxDistance, int layerMask);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern bool raycast2(IntPtr scene, PxVec3 origin, PxVec3 direction, ref PxRaycastHitP hitInfo, float maxDistance, int layerMask);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern int raycastNonAlloc(IntPtr scene, PxVec3 origin, PxVec3 direction, float maxDistance, int layerMask, [Out] PxRaycastHitP[] hitInfoOut, int maxCount);

        

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern bool sphereCast(IntPtr scene, PxVec3 origin, float radius, PxVec3 direction, ref PxRaycastHitP hitInfoOut, float maxDistance, int layerMask);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern int sphereCastNonAlloc(IntPtr scene, PxVec3 origin, float radius, PxVec3 direction, float maxDistance, int layerMask, [Out] PxRaycastHitP[] hitInfoOut, int maxCount);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
	    private static extern bool boxCast(IntPtr scene, PxVec3 center, PxVec3 halfExtents, PxVec3 direction, ref PxRaycastHitP hitInfoOut, PxQuat orientation, float maxDistance, int layerMask);
        
        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern int boxCastNonAlloc(IntPtr scene, PxVec3 center, PxVec3 halfExtents, PxVec3 direction, PxQuat orientation, float maxDistance, int layerMask, [Out] PxRaycastHitP[] hitInfoOut, int maxCount);

        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
	    private static extern bool capsuleCast(IntPtr scene, PxVec3 point1, PxVec3 point2, float radius, PxVec3 direction, ref PxRaycastHitP hitInfoOut, float maxDistance, int layerMask);
        
        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern int capsuleCastNonAlloc(IntPtr scene, PxVec3 point1, PxVec3 point2, float radius, PxVec3 direction, float maxDistance, int layerMask, [Out] PxRaycastHitP[] hitInfoOut, int maxCount);



        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern int overlapSphereNonAlloc(IntPtr scene, PxVec3 origin, float radius, int layerMask, [Out] PxActorShapeP[] result, int maxCount);
    	
        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern int overlapBoxNonAlloc(IntPtr scene, PxVec3 center, PxVec3 halfExtents,  PxQuat orientation, int mask, [Out] PxActorShapeP[] results, int maxCount);
	    
        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern int overlapCapsuleNonAlloc(IntPtr scene, PxVec3 point0, PxVec3 point1,  float radius, int mask, [Out] PxActorShapeP[] results, int maxCount);

        #endregion


        //============================================================================================================================
        #region Force
        [DllImport(PhysXDLL, CallingConvention = CallingConvention.Cdecl)]
        private static extern void addForce(IntPtr actor, PxVec3 force, int mode);
        #endregion

        #region Physics

        /// <summary>
        /// 初始化物理引擎
        /// </summary>
        /// <returns></returns>
        public static bool InitPhysics(bool[,] coliisionTable)
        {
            bool[] table = new bool[32 * 32];
            int index = 0;
            for (int i = 0; i < 32; i++)
            {
                for (int j = 0; j < 32; j++)
                {
                    table[index++] = coliisionTable[i, j];
                }
            }
            return initPhysics(table);
        }

        /// <summary>
        /// 清理物理引擎
        /// </summary>
        public static void CleanupPhysics()
        {
            cleanupPhysics();
        }

        public static void SetSleepThreshold(float threshold)
        {
            sleepThreshold(threshold);
        }

        public static  void SetBounceThreshold(float threshold)
        {
            bounceThreshold(threshold);
        }

        public static void DefaultContactOffset(float contactOffset)
        {
            defaultContactOffset(contactOffset);
        }
        public static void DefaultSolverIterations(float minPositionIters)
        {
            defaultSolverIterations(minPositionIters);
        }
        public static void DefaultSolverVelocityIterations(float minVelocityIters)
        {
            defaultSolverVelocityIterations(minVelocityIters);
        }
        public static void FrictionType(int type)
        {
            frictionType(type);
        }
        #endregion

        /// <summary>
        /// 创建物理世界
        /// </summary>
        /// <param name="id">物理世界Id 需要唯一</param>
        /// <returns></returns>
        public static IntPtr CreatePhysicsScene()
        {
            return createPhysicsScene();
        }

        /// <summary>
        /// 销毁物理世界
        /// </summary>
        /// <param name="id">物理世界Id</param>
        public static void DestroyPhysicsScene(IntPtr scene)
        {
            destroyPhysicsScene(scene);
        }



        /// <summary>
        /// 调用物理世界的Update
        /// </summary>
        /// <param name="id"></param>
        /// <param name="dt"></param>
        public static void StepPhysicsScene(IntPtr scene, float dt)
        {
            stepPhysicsScene(scene, dt);
        }

        public static IntPtr CreateRigidbody(IntPtr scene, Vector3 pos, Quaternion qua, bool isStatic = false)
        {
            PxTransformP pTm = new PxTransformP()
            {
                q = qua.ToPx(),
                p = pos.ToPx(),
            };

            return createRigidbody(scene, pTm, isStatic);
        }

        /// <summary>
        /// 删除actorr
        ///一起移除所有Shape
        /// </summary>
        /// <param name="scene"></param>
        /// <param name="actor"></param>
        public static void DestroyRigidActor(IntPtr scene, IntPtr actor)
        {
            destroyRigidActor(scene, actor);
        }


        public static IntPtr ChangeRigidbodyStatic(IntPtr scene, IntPtr actor, bool isStatic)
        {
            return changeRigidbodyStatic(scene, actor, isStatic);
        }


        #region Collider
        /// <summary>
        /// 删除collider
        /// </summary>
        /// <param name="actor"></param>
        /// <param name="shape"></param>
        public static void RemoveCollider(IntPtr actor, IntPtr shape)
        {
            removeCollider(actor, shape);
        }

        public static IntPtr AddBoxCollider(IntPtr scene, IntPtr actor, int layer, Vector3 center, Vector3 size, bool isTrigger = false)
        {
            return addBoxCollider(scene, actor, layer, center.ToPx(), size.ToPx(), isTrigger);
        }

        public static IntPtr AddCapsuleCollider(IntPtr scene, IntPtr actor, int layer, float radius, float height, int direction, bool isTrigger = false)
        {
            return addCapsuleCollider(scene, actor, layer, radius, height, direction, isTrigger);
        } 

        public static IntPtr AddSphereCollider(IntPtr scene, IntPtr actor, int layer, Vector3 center, float radius, bool isTrigger = false)
        {
            return addSphereCollider(scene, actor, layer, center.ToPx(), radius, isTrigger);
        }
        #endregion

        /// <summary>
        /// 设置世界坐标
        /// </summary>
        /// <param name="collider"></param>
        /// <param name="pos"></param>
        public static void SetActorPosition(long collider, Vector3 pos)
        {
            IntPtr intPtr = (IntPtr)collider;
            setActorPosition(intPtr, pos.ToPx());
        }

        /// <summary>
        /// 设置世界坐标
        /// </summary>
        /// <param name="collider"></param>
        /// <param name="pos"></param>
        public static void SetActorQuaternion(long collider, Quaternion qua)
        {
            IntPtr intPtr = (IntPtr)collider;
            setActorQuaternion(intPtr, qua.ToPx());
        }

        public static void AddForce(IntPtr actor, Vector3 force, ForceMode mode)
        {
            addForce(actor, force.ToPx(), (int)mode);
        }

        #region Scene Query
        public static bool Raycast(IntPtr scene, Vector3 origin, Vector3 direction, float maxDistance, int layerMask)
        {
            return raycast(scene, origin.ToPx(), direction.ToPx(), maxDistance, layerMask);
        }

        public static bool Raycast2(IntPtr scene, Vector3 origin, Vector3 direction, float maxDistance, int layerMask)
        {
            PxRaycastHitP hitInfo = default;
            bool hit = raycast2(scene, origin.ToPx(), direction.ToPx(), ref hitInfo, maxDistance, layerMask);

            return hit;
        }

        public static int RaycastNonAlloc(IntPtr scene, Vector3 origin, Vector3 direction, PxRaycastHitP[] hitInfo, float maxDistance, int layerMask)
        {
            int maxCount = hitInfo.Length;
            int hitCount = raycastNonAlloc(scene, origin.ToPx(), direction.ToPx(), maxDistance, layerMask, hitInfo, maxCount);
            return hitCount;
        }


        public static bool SphereCast(IntPtr scene, Vector3 origin, float radius, Vector3 direction, out PxRaycastHitP hitInfo, float maxDistance, int layerMask)
        {
            hitInfo = default;
            return sphereCast(scene, origin.ToPx(), radius, direction.ToPx(), ref hitInfo, maxDistance, layerMask);
        }

        public static int SphereCastNonAlloc(IntPtr scene, Vector3 origin, float radius, Vector3 direction, PxRaycastHitP[] results, float maxDistance, int layerMask)
        {
            int maxCount = results.Length;
            int hitCount = sphereCastNonAlloc(scene, origin.ToPx(), radius, direction.ToPx(), maxDistance, layerMask, results, maxCount);
            return hitCount;
        }


	    public static bool BoxCast(IntPtr scene, Vector3 center, Vector3 halfExtents, Vector3 direction, ref PxRaycastHitP hitInfo, Quaternion orientation, float maxDistance, int layerMask)
        {
            hitInfo = default;
            return boxCast(scene, center.ToPx(), halfExtents.ToPx(), direction.ToPx(), ref hitInfo, orientation.ToPx(), maxDistance, layerMask);
        }
        

        public static int BoxCastNonAlloc(IntPtr scene, Vector3 center, Vector3 halfExtents, Vector3 direction, PxRaycastHitP[] results, Quaternion orientation, float maxDistance, int layerMask)
        {
            int maxCount = results.Length;
            int hitCount = boxCastNonAlloc(scene, center.ToPx(), halfExtents.ToPx(), direction.ToPx(), orientation.ToPx(), maxDistance, layerMask, results, maxCount);
            return hitCount;
        }

	    public static bool CapsuleCast(IntPtr scene, Vector3 point1, Vector3 point2, float radius, Vector3 direction, ref PxRaycastHitP hitInfo, float maxDistance, int layerMask)
        {
            hitInfo = default;
            return capsuleCast(scene, point1.ToPx(), point1.ToPx(), radius, direction.ToPx(), ref hitInfo, maxDistance, layerMask);
        }
        

        public static int CapsuleCastNonAlloc(IntPtr scene, Vector3 point1, Vector3 point2, float radius, Vector3 direction, PxRaycastHitP[] results, float maxDistance, int layerMask)
        {
            int maxCount = results.Length;
            int hitCount = capsuleCastNonAlloc(scene, point1.ToPx(), point1.ToPx(), radius, direction.ToPx(), maxDistance, layerMask, results, maxCount);
            return hitCount;
        }



        public static int OverlapSphereNonAlloc(IntPtr scene, Vector3 position, float radius, PxActorShapeP[] results, int layerMask)
        {
            int maxCount = results.Length;
            int hitCount = overlapSphereNonAlloc(scene, position.ToPx(), radius, layerMask, results, maxCount);
            return hitCount;
        }

        public static int OverlapBoxNonAlloc(IntPtr scene, Vector3 center, Vector3 halfExtents, PxActorShapeP[] results, Quaternion orientation, int layerMask)
        {
            int maxCount = results.Length;
            int hitCount = overlapBoxNonAlloc(scene, center.ToPx(), halfExtents.ToPx(), orientation.ToPx(), layerMask, results, maxCount);
            return hitCount;
        }
	    
        public static int OverlapCapsuleNonAlloc(IntPtr scene, Vector3 point0, Vector3 point1,  float radius, PxActorShapeP[] results, int layerMask)
        {
            int maxCount = results.Length;
            int hitCount = overlapCapsuleNonAlloc(scene, point0.ToPx(), point1.ToPx(), radius, layerMask, results, maxCount);
            return hitCount;
        }
        #endregion


        #region Sync Date / Event
        public static void SyncTransforms(IntPtr scene, PxActorSync[] tranforms, uint count)
        {
            syncTransforms(scene, tranforms, count);
        }

        public static uint FetchTransforms(IntPtr scene, PxActorSync[] tranforms)
        {
            uint count = 0;
            fetchTransforms(scene, tranforms, ref count);
            return count;
        }

        public static uint FetchCollisions(IntPtr scene, PxCollision[] collisions)
        {
            uint count = 0;
            fetchContacts(scene, collisions, ref count);
            return count;
        }

        public static uint FetchTriggers(IntPtr scene, PxTrigger[] triggers)
        {
            uint count = 0;
            fetchTriggers(scene, triggers, ref count);
            return count;
        } 
        #endregion


        /// <summary>
        /// position: offset [0-2]
        /// quaternoin: offset[3-6]
        /// </summary>
        private static float[] sCache = new float[65535];

        public static PxVec3 ToPx(this Vector3 self)
        {
            return new PxVec3() 
            {
                x = self.x,
                y = self.y,
                z = self.z,
            };
        }

        public static PxQuat ToPx(this Quaternion self)
        {
            return new PxQuat()
            {
                x = self.x,
                y = self.y,
                z = self.z,
                w = self.w,
            };
        }

        public static Vector3 ToUnity(this PxVec3 self)
        {
            return new Vector3()
            {
                x = self.x,
                y = self.y,
                z = self.z,
            };
        }

        public static Quaternion ToUnity(this PxQuat self)
        {
            return new Quaternion()
            {
                x = self.x,
                y = self.y,
                z = self.z,
                w = self.w,
            };
        }
    }



    [StructLayout(LayoutKind.Sequential)]
    public struct PxVec3
    {
        public float x;
        public float y;
        public float z;
    };


    [StructLayout(LayoutKind.Sequential)]
    public struct PxQuat
    {
        public float x;
        public float y;
        public float z;
        public float w;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct PxTransformP
    {
        public PxQuat q;
        public PxVec3 p;
    }


    [StructLayout(LayoutKind.Sequential)]
    struct PxContactPoint
    {
        public PxVec3 position;
        public PxVec3 normal;
        public PxVec3 impulse;
        public float separation;
    };

    [StructLayout(LayoutKind.Sequential)]
    public struct PxActorSync
    {
        public IntPtr actor;
        public PxTransformP tm;
    }


    [StructLayout(LayoutKind.Sequential)]
    public struct PxContactPairPoint
    {
        public PxVec3 position;
        public float separation;
        public PxVec3 normal;
        public uint internalFaceIndex0;
        public PxVec3 impulse;
        public uint internalFaceIndex1;
    }


    [StructLayout(LayoutKind.Sequential)]
    public struct PxCollision
    {
        public IntPtr contactPair;
        public IntPtr actor0;
        public IntPtr actor1;
        public IntPtr shape0;
        public IntPtr shape1;
        public int status;
        public int contactCount;
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct PxTrigger
    {
        public IntPtr triggerActor;
        public IntPtr otherActor;
        public int status;
    };


    [StructLayout(LayoutKind.Sequential)]
    public struct PxActorShapeP
    {
        public IntPtr actor;
        public IntPtr shape;
    }


    [StructLayout(LayoutKind.Sequential)]
    public struct PxRaycastHitP
    {
        public IntPtr actor;
        public IntPtr shape;

        public uint faceIndex;

        public ushort flags;
        public PxVec3 position;
        public PxVec3 normal;
        public float distance;

        public float u;
        public float v;
    }
}
