package core.physics;

import com.bulletphysics.collision.broadphase.AxisSweep3;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.BroadphasePair;
import com.bulletphysics.collision.broadphase.HashedOverlappingPairCache;
import com.bulletphysics.collision.dispatch.*;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.collision.shapes.*;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.ObjectArrayList;
import core.math.Vec3f;
import dc.ChunkNode;
import dc.entities.MeshBuffer;
import dc.utils.Aabb;

import javax.vecmath.Vector3f;
import java.nio.ByteBuffer;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Level;
import java.util.logging.Logger;

import static core.physics.PhysicsOperationType.PhysicsOp_RayCast;
import static core.physics.PhysicsOperationType.PhysicsOp_WorldUpdate;

public class JBulletPhysics implements Physics {
    final public static Logger logger = Logger.getLogger(JBulletPhysics.class.getName());
    // i.e. 1 voxel is PHYSICS_SCALE meters
    //final float PHYSICS_SCALE = 0.05f;
    final float PHYSICS_SCALE = 1f;
    private DynamicsWorld dynamicsWorld;
    private BroadphaseInterface broadphase;
    private Aabb g_worldBounds;
    private boolean g_physicsQuit = false;
    private ConcurrentLinkedQueue<Runnable> g_operationQueue;
    private Thread g_physicsThread;
    private ExecutorService executorService;
    private volatile boolean g_rayCastPending = false;
    private Vec3f collisionPos;
    private Vec3f collisionNorm;
    private float closestHitFraction = Float.MAX_VALUE;
    private Player g_player;
    private int PHYSICS_GROUP_WORLD = 1;
    private int PHYSICS_GROUP_ACTOR = 2;
    private short PHYSICS_GROUP_PLAYER = 4;
    private short PHYSICS_FILTER_ALL = (short) (PHYSICS_GROUP_WORLD | PHYSICS_GROUP_ACTOR | PHYSICS_GROUP_PLAYER);
    private short PHYSICS_FILTER_NOT_PLAYER = (short) (PHYSICS_FILTER_ALL & ~PHYSICS_GROUP_PLAYER);
    private int maxChunkSize;
    private boolean playerCollision;

    private void EnqueuePhysicsOperation(PhysicsOperationType opType, Runnable op) {
        try {
            boolean res = g_operationQueue.add(op);
            if (!res) {
                int t = 3;
            }
        } catch (Throwable e) {
            logger.log(Level.SEVERE, e.toString());
        }
    }

    @Override
    public Vec3f getCollisionNorm() {
        return collisionNorm;
    }

    @Override
    public Vec3f getCollisionPos() {
        return collisionPos;
    }

    private Vec3f Scale_WorldToPhysics(Vec3f worldValue) {
        return new Vec3f(worldValue.X * PHYSICS_SCALE, worldValue.Y * PHYSICS_SCALE, worldValue.Z * PHYSICS_SCALE);
    }

    private javax.vecmath.Vector3f Scale_WorldToPhysics(javax.vecmath.Vector3f worldValue) {
        return new javax.vecmath.Vector3f(worldValue.x * PHYSICS_SCALE, worldValue.y * PHYSICS_SCALE, worldValue.z * PHYSICS_SCALE);
    }

    private javax.vecmath.Vector3f Scale_PhysicsToWorld(javax.vecmath.Vector3f worldValue) {
        return new javax.vecmath.Vector3f(worldValue.x / PHYSICS_SCALE, worldValue.y / PHYSICS_SCALE, worldValue.z / PHYSICS_SCALE);
    }

    public JBulletPhysics(Aabb g_worldBounds, int maxChunkSize, boolean playerCollision) {
        executorService = Executors.newFixedThreadPool(6, new ThreadFactory() {
                    private AtomicInteger count = new AtomicInteger();
                    @Override
                    public Thread newThread(Runnable r) {
                        Thread thread = new Thread(r);
                        thread.setName("JBulletPhysics " + count.getAndIncrement());
                        return thread;
                    }
                }
        );
        g_operationQueue = new ConcurrentLinkedQueue<>();
        Physics_Initialise(g_worldBounds);
        g_player = new Player();
        collisionPos = new Vec3f();
        collisionNorm = new Vec3f();
        this.maxChunkSize = maxChunkSize;
        this.playerCollision = playerCollision;
    }

    private void Physics_Initialise(Aabb worldBounds) {
        g_worldBounds = worldBounds;
        CollisionConfiguration collisionCfg = new DefaultCollisionConfiguration();
        CollisionDispatcher dispatcher = new CollisionDispatcher(collisionCfg);
        HashedOverlappingPairCache pairCache = new HashedOverlappingPairCache();

        Vector3f worldMin = new Vector3f(g_worldBounds.min.x, g_worldBounds.min.y, g_worldBounds.min.z);
        Vector3f worldMax = new Vector3f(g_worldBounds.max.x, g_worldBounds.max.y, g_worldBounds.max.z);
        int maxHandles = 3500;

        broadphase = new AxisSweep3(Scale_WorldToPhysics(worldMin), Scale_WorldToPhysics(worldMax), maxHandles, pairCache);
        ConstraintSolver solver = new SequentialImpulseConstraintSolver();
        dynamicsWorld = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionCfg);
        dynamicsWorld.setGravity(new javax.vecmath.Vector3f(0, -9.8f, 0));
        dynamicsWorld.getSolverInfo().numIterations = 5;

        g_physicsQuit = false;
        g_physicsThread = new Thread(this::PhysicsThreadFunction);
        g_physicsThread.start();
    }

    @Override
    public void Physics_UpdateWorldNodeMainMesh(boolean updateMain, ChunkNode chunkNode) {
        if ((updateMain && chunkNode.renderMesh == null) || (!updateMain && chunkNode.seamMesh == null)|| chunkNode.size > maxChunkSize) {
            return;
        }
        MeshBuffer meshBuffer = updateMain ? chunkNode.renderMesh.meshBuffer : chunkNode.seamMesh.meshBuffer;
        EnqueuePhysicsOperation(PhysicsOp_WorldUpdate, () -> UpdateCollisionNode(updateMain, chunkNode.worldNode, meshBuffer));
    }

    public void RemoveMeshData(PhysicsMeshData meshData){
        EnqueuePhysicsOperation(PhysicsOp_WorldUpdate, () -> RemovePhysicsMeshData(meshData));
    }

    // call after node is update (CSG operation)
    private void UpdateCollisionNode(boolean updateMain, WorldCollisionNode node, MeshBuffer meshBuffer) {
        executorService.submit(() -> {
            if (meshBuffer != null) {
                try {
                    PhysicsMeshData newMesh = addMeshToWorldImpl(meshBuffer);
                    EnqueuePhysicsOperation(PhysicsOp_WorldUpdate, () -> ReplaceCollisionNodeMesh(updateMain, node, newMesh));
                } catch (Throwable e) {
                    logger.log(Level.SEVERE, e.toString());
                }
            }
        });
    }

    private PhysicsMeshData addMeshToWorldImpl(MeshBuffer meshBuffer) {
        ByteBuffer indicatesByteBuffer = ByteBuffer.allocate(Integer.BYTES * meshBuffer.getNumIndicates());
        indicatesByteBuffer.asIntBuffer().put(meshBuffer.getIndicates());
        meshBuffer.getIndicates().flip();
        indicatesByteBuffer.rewind();

        ByteBuffer verticesByteBuffer = ByteBuffer.allocate(Float.BYTES * 3 * meshBuffer.getNumVertices());
        for (int i = 0; i < meshBuffer.getNumVertices(); i++) {
            int index = i * 9;
            float x = meshBuffer.getVertices().get(index + 0) * PHYSICS_SCALE;
            float y = meshBuffer.getVertices().get(index + 1) * PHYSICS_SCALE;
            float z = meshBuffer.getVertices().get(index + 2) * PHYSICS_SCALE;
            verticesByteBuffer.putFloat(x);
            verticesByteBuffer.putFloat(y);
            verticesByteBuffer.putFloat(z);
        }
        verticesByteBuffer.flip();

        IndexedMesh indexedMesh = new IndexedMesh();
        indexedMesh.vertexBase = verticesByteBuffer;
        indexedMesh.vertexStride = 3 * Float.BYTES;
        indexedMesh.numVertices = meshBuffer.getNumVertices();
        indexedMesh.triangleIndexBase = indicatesByteBuffer;
        indexedMesh.triangleIndexStride = 3 * Float.BYTES;
        indexedMesh.numTriangles = meshBuffer.getNumIndicates() / 3;
        indexedMesh.indexType = ScalarType.INTEGER;

        PhysicsMeshData meshData = new PhysicsMeshData();
        meshData.buffer = new TriangleIndexVertexArray();
        meshData.buffer.addIndexedMesh(indexedMesh);
        meshData.shape = new BvhTriangleMeshShape(meshData.buffer, true, true);
        meshData.body = new RigidBody(0, null, meshData.shape);
        meshData.body.setFriction(0.9f);

        meshData.body.setCollisionFlags(meshData.body.getCollisionFlags() | CollisionFlags.STATIC_OBJECT);
        return meshData;
    }

    private void ReplaceCollisionNodeMesh(boolean replaceMainMesh, WorldCollisionNode node, PhysicsMeshData newMesh) {
        PhysicsMeshData oldMesh;
        if (replaceMainMesh) {
            oldMesh = node.mainMesh;
            node.mainMesh = newMesh;
        } else {
            oldMesh = node.seamMesh;
            node.seamMesh = newMesh;
        }

        if (newMesh != null) {
            dynamicsWorld.addRigidBody(newMesh.body);
        }
        if (oldMesh != null) {
            RemoveMeshData(oldMesh);
        }
    }

    private void RemovePhysicsMeshData(PhysicsMeshData meshData) {
        if(meshData.body!=null) {
            dynamicsWorld.removeRigidBody(meshData.body);
        }
        meshData.body = null;
        meshData.shape = null;
        meshData.buffer = null;
    }

    private void CastRayImpl(Vec3f start, Vec3f end) {
        Vec3f rayStart = Scale_WorldToPhysics(new Vec3f(start.X, start.Y, start.Z));
        Vec3f rayEnd = Scale_WorldToPhysics(new Vec3f(end.X, end.Y, end.Z));

        CollisionWorld.ClosestRayResultCallback rayCallback = new CollisionWorld.ClosestRayResultCallback(rayStart.convert(), rayEnd.convert());
        if (dynamicsWorld.getNumCollisionObjects() > 0) {
            int t = 3;
        }
        dynamicsWorld.rayTest(rayStart.convert(), rayEnd.convert(), rayCallback);

        if (rayCallback.hasHit()) {
            RigidBody body = RigidBody.upcast(rayCallback.collisionObject);
            if (body != null && body.hasContactResponse()) {
                collisionPos.set(rayCallback.hitPointWorld);
                collisionNorm.set(rayCallback.hitNormalWorld);
                collisionNorm.normalize();
                closestHitFraction = rayCallback.closestHitFraction;
            }
        }
        g_rayCastPending = false;
    }

    @Override
    public void Physics_CastRay(Vec3f start, Vec3f end) {
        if (!g_rayCastPending) {
            g_rayCastPending = true;
            EnqueuePhysicsOperation(PhysicsOp_RayCast, () -> CastRayImpl(start, end));
        }
    }

    private void PhysicsThreadFunction() {
        long prevTime = System.nanoTime();
        float startTime = (float) prevTime / 1000.f;
        while (!g_physicsQuit) {
            Runnable task;
            while (g_operationQueue.size() > 0) {
                try {
                    if ((task = g_operationQueue.poll()) != null) {
                        task.run();
                    }
                } catch (Throwable e) {
                    logger.log(Level.SEVERE, e.toString());
                }
            }

            long deltaTime = System.nanoTime() - prevTime;
            float dt = deltaTime / 1000.f;
            float updatePeriod = 1 / 60.f;
            if (dt < updatePeriod) {
                continue;
            }
            prevTime = System.nanoTime();
            float elapsedTime = ((float) prevTime / 1000.f) - startTime;
            dynamicsWorld.stepSimulation(dt);
            if(playerCollision) {
                UpdatePlayer(dt, elapsedTime);
            }
        }
    }

    @Override
    public void Physics_Shutdown() {
        g_operationQueue.clear();
        g_physicsQuit = true;
        try {
            g_physicsThread.join();
        } catch (InterruptedException e) {
            logger.log(Level.SEVERE, e.toString());
        }

        if(g_player.body!=null) {
            dynamicsWorld.removeRigidBody(g_player.body);
        }
        if(g_player.ghost!=null) {
            dynamicsWorld.removeCollisionObject(g_player.ghost);
        }
        dynamicsWorld.destroy();
    }

    @Override
    public void Physics_SpawnPlayer(Vec3f origin) {
        EnqueuePhysicsOperation(PhysicsOp_WorldUpdate, () -> SpawnPlayerImpl(origin));
    }

    @Override
    public Vec3f Physics_GetPlayerPosition() {
        // TODO not thread safe
        if (g_player.body!=null) {
            Vector3f origin = g_player.body.getWorldTransform(new Transform()).origin;
            Vec3f position = new Vec3f(origin.x, origin.y, origin.z);

		    float eyeOffset = (PLAYER_HEIGHT / 2.f) - 1.f;
            position.Y += eyeOffset;
            return position;
        }
        return new Vec3f(0.f);
    }

    @Override
    public void Physics_SetPlayerVelocity(Vec3f velocity) {
        EnqueuePhysicsOperation(PhysicsOp_WorldUpdate, () -> g_player.velocity = velocity);
    }

    @Override
    public void Physics_PlayerJump() {
        EnqueuePhysicsOperation(PhysicsOp_WorldUpdate, () -> g_player.jump = true);
    }

    @Override
    public void Physics_TogglePlayerNoClip() {
        EnqueuePhysicsOperation(PhysicsOp_WorldUpdate, () -> g_player.noclip = !g_player.noclip);
    }

    private void SpawnPlayerImpl(Vec3f origin) {
        if (g_player.body != null || g_player.ghost != null) {
            return;
        }
        CapsuleShape collisionShape = new CapsuleShape(3f, 4f);

        Transform transform = new Transform();
        transform.setIdentity();
        transform.origin.set(origin.X, origin.Y, origin.Z);

        float mass = 10.f;
        Vector3f ineritia = new Vector3f();
        MotionState motionState = new DefaultMotionState(transform);
        collisionShape.calculateLocalInertia(mass, ineritia);

        RigidBodyConstructionInfo bodyInfo = new RigidBodyConstructionInfo(mass, motionState, collisionShape, ineritia);

        g_player.body = new RigidBodyCustom(bodyInfo);
        g_player.body.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
        g_player.body.setAngularFactor(0.f);
        dynamicsWorld.addRigidBody(g_player.body);

        g_player.ghost = new PairCachingGhostObjectCustom();
        g_player.ghost.setCollisionShape(collisionShape);
        g_player.ghost.setCollisionFlags(CollisionFlags.NO_CONTACT_RESPONSE);
        g_player.ghost.setWorldTransform(transform);

        dynamicsWorld.addCollisionObject(g_player.ghost, PHYSICS_GROUP_PLAYER, PHYSICS_FILTER_NOT_PLAYER);
        dynamicsWorld.getPairCache().setInternalGhostPairCallback(new GhostPairCallback());
    }

    private void UpdatePlayer(float deltaT, float elapsedTime) {
        if (g_player==null || g_player.body == null || g_player.ghost == null) {
            return;
        }

        Vector3f origin = g_player.body.getWorldTransform().origin;

//        float bottomOffset = (PLAYER_WIDTH / 2.f) + (PLAYER_HEIGHT / 2.f);
//        Vector3f rayEnd = new Vector3f();
//        rayEnd.sub(origin, Scale_WorldToPhysics(new Vector3f(0.f, bottomOffset + 0.f, 0.f)));
//
//        CollisionWorld.ClosestRayResultCallback callback = new CollisionWorld.ClosestRayResultCallback(origin, rayEnd);
//        dynamicsWorld.rayTest(origin, rayEnd, callback);

        boolean onGround = false;
        float onGroundDot = 0.f;

        ObjectArrayList<BroadphasePair> pairs = g_player.ghost.getOverlappingPairCache().getOverlappingPairArray();
        ObjectArrayList<PersistentManifold> manifolds = new ObjectArrayList<>();
        for (BroadphasePair broadphasePair : pairs) {
            manifolds.clear();
            BroadphasePair pair = dynamicsWorld.getPairCache().findPair(broadphasePair.pProxy0, broadphasePair.pProxy1);
            if (pair == null) {
                continue;
            }

            if (pair.algorithm != null) {
                pair.algorithm.getAllContactManifolds(manifolds);
            }

            for (PersistentManifold m : manifolds) {
                boolean isFirstBody = m.getBody0().equals(g_player.ghost);
                float dir = isFirstBody ? -1.f : 1.f;
                for (int c = 0; c < m.getNumContacts(); c++) {
                    ManifoldPoint pt = m.getContactPoint(c);
                    if (pt.getDistance() <= 1e-3) {
                        Vector3f p1 = isFirstBody ? pt.getPositionWorldOnA(new Vector3f()) : pt.getPositionWorldOnB(new Vector3f());
                        Vector3f d = new Vector3f();
                        d.sub(origin, p1);
                        d.normalize();
                        onGroundDot = Math.max(onGroundDot, d.dot(new Vector3f(0.f, 1.f, 0.f)));
                    }
                }
            }
        }

        float checkGroundTime = 0.f;
        if (g_player.falling && checkGroundTime < elapsedTime) {
            if (onGroundDot > 0.85f) {
                onGround = true;
                g_player.falling = false;
            }
        }

        //printf("onGroundDot=%.2f falling=%s\n", onGroundDot, g_player.falling ? "True" : "False");
        Vector3f out = new Vector3f();
        Vector3f currentVelocty = g_player.body.getLinearVelocity(out);
        Vec3f inputVelocity = g_player.velocity.mul(0.05f);
        if (!g_player.noclip) {
            float velocity = currentVelocty.y;
            if (!g_player.falling) {
                velocity = Math.min(0.f, velocity) - (10 * deltaT);
            }

            if (!g_player.falling && g_player.jump) {//		printf("jump\n");
                g_player.body.applyCentralImpulse(new Vector3f(0, 65, 0));
                g_player.jump = false;
                g_player.falling = true;
                checkGroundTime = elapsedTime + 0.1f;
            } else {
                g_player.body.setLinearVelocity(new Vector3f(inputVelocity.X, velocity, inputVelocity.Z));
            }
        } else {
            g_player.body.setLinearVelocity(new Vector3f(inputVelocity.X, inputVelocity.Y, inputVelocity.Z));
        }

        g_player.ghost.getWorldTransform().origin.set(g_player.body.getWorldTransform().origin);
    }
}