package core.physics;

import com.bulletphysics.collision.broadphase.AxisSweep3;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.HashedOverlappingPairCache;
import com.bulletphysics.collision.dispatch.*;
import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.IndexedMesh;
import com.bulletphysics.collision.shapes.ScalarType;
import com.bulletphysics.collision.shapes.TriangleIndexVertexArray;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.Transform;
import core.math.Vec3f;
import core.math.Vec3i;
import dc.ChunkNode;
import dc.entities.MeshBuffer;
import dc.impl.MeshGenerationContext;
import dc.utils.Aabb;

import javax.vecmath.Vector3f;
import java.nio.ByteBuffer;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

import static core.physics.PhysicsOperationType.PhysicsOp_RayCast;
import static core.physics.PhysicsOperationType.PhysicsOp_WorldUpdate;

public class JBulletPhysics implements Physics{
    // i.e. 1 voxel is PHYSICS_SCALE meters
    //final float PHYSICS_SCALE = 0.05f;
    final float PHYSICS_SCALE = 1f;
    private DynamicsWorld dynamicsWorld;
    private BroadphaseInterface broadphase;
    private Aabb g_worldBounds;
    private boolean g_physicsQuit = false;
    private ConcurrentLinkedQueue<Runnable> g_operationQueue;
    private Thread g_physicsThread;
    private ExecutorService service;
    private volatile boolean g_rayCastPending = false;
    private Vec3f collisionPos;
    private Vec3f collisionNorm;
    private float closestHitFraction = Float.MAX_VALUE;

    private void EnqueuePhysicsOperation(PhysicsOperationType opType, Runnable op) {
        try {
            boolean res = g_operationQueue.add(op);
            if(!res){
                int t=3;
            }
        } catch (Throwable e){
            e.printStackTrace();
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

    public JBulletPhysics(Aabb g_worldBounds) {
        service = Executors.newFixedThreadPool(6);
        g_operationQueue = new ConcurrentLinkedQueue<>();
        Physics_Initialise(g_worldBounds);
        collisionPos = new Vec3f();
        collisionNorm = new Vec3f();
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
        if((updateMain && chunkNode.renderMesh==null) || (!updateMain && chunkNode.seamMesh==null)){
            return;
        }
        MeshBuffer meshBuffer = updateMain ? chunkNode.renderMesh.meshBuffer : chunkNode.seamMesh.meshBuffer;
        EnqueuePhysicsOperation(PhysicsOp_WorldUpdate, ()->UpdateCollisionNode(updateMain, chunkNode.worldNode, chunkNode.min, meshBuffer));
    }

    // call after node is update (CSG operation)
    private void UpdateCollisionNode(boolean updateMain, WorldCollisionNode node, Vec3i min, MeshBuffer meshBuffer) {
        service.submit(() -> {
            if (meshBuffer!=null) {
                try{
                    PhysicsMeshData newMesh = addMeshToWorldImpl(min, meshBuffer);
                    EnqueuePhysicsOperation(PhysicsOp_WorldUpdate, ()->ReplaceCollisionNodeMesh(updateMain, node, newMesh));
                } catch (Throwable e){
                    e.printStackTrace();
                }
            }
        });
    }

    private PhysicsMeshData addMeshToWorldImpl(Vec3i nodeMin, MeshBuffer meshBuffer){
        ByteBuffer indicatesByteBuffer = ByteBuffer.allocate(Integer.BYTES * meshBuffer.getNumIndicates());
        indicatesByteBuffer.asIntBuffer().put(meshBuffer.getIndicates());
        meshBuffer.getIndicates().flip();

        ByteBuffer verticesByteBuffer = ByteBuffer.allocate(Float.BYTES * 3 * meshBuffer.getNumVertices());
        for (int i = 0; i < meshBuffer.getNumVertices(); i++) {
            int index = i * 9;
            float x = meshBuffer.getVertices().get(index+0) * PHYSICS_SCALE;
            float y = meshBuffer.getVertices().get(index+1) * PHYSICS_SCALE;
            float z = meshBuffer.getVertices().get(index+2) * PHYSICS_SCALE;
            verticesByteBuffer.putFloat(x);
            verticesByteBuffer.putFloat(y);
            verticesByteBuffer.putFloat(z);
        }

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

//        Transform transform = new Transform();
//        transform.setIdentity();
//        transform.origin.x = nodeMin.x * PHYSICS_SCALE;
//        transform.origin.y = nodeMin.y * PHYSICS_SCALE;
//        transform.origin.z = nodeMin.z * PHYSICS_SCALE;
//
//        meshData.body.setWorldTransform(transform);
//        meshData.body.setCollisionFlags(meshData.body.getCollisionFlags() | CollisionFlags.STATIC_OBJECT);
        return meshData;
    }

    private void ReplaceCollisionNodeMesh(boolean replaceMainMesh, WorldCollisionNode node, PhysicsMeshData newMesh) {
        PhysicsMeshData oldMesh;
        if (replaceMainMesh) {
            oldMesh = node.mainMesh;
            node.mainMesh = newMesh;
        }
        else {
            oldMesh = node.seamMesh;
            node.seamMesh = newMesh;
        }

        if (newMesh!=null) {
            dynamicsWorld.addRigidBody(newMesh.body);
        }
        if (oldMesh!=null) {
            RemoveMeshData(oldMesh);
        }
    }

    public void RemoveMeshData(PhysicsMeshData meshData) {
        dynamicsWorld.removeRigidBody(meshData.body);
        meshData.body = null;
        meshData.shape = null;
        meshData.buffer = null;
    }

    private void CastRayImpl(Vec3f start, Vec3f end){
        Vec3f rayStart = Scale_WorldToPhysics(new Vec3f(start.X, start.Y, start.Z));
        Vec3f rayEnd = Scale_WorldToPhysics(new Vec3f(end.X, end.Y, end.Z));

        CollisionWorld.ClosestRayResultCallback rayCallback = new CollisionWorld.ClosestRayResultCallback(rayStart.convert(), rayEnd.convert());
        if(dynamicsWorld.getNumCollisionObjects()>0){
            int t=3;
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
        while (!g_physicsQuit) {
            Runnable task;
            while (g_operationQueue.size() > 0) {
                try {
                    if ((task = g_operationQueue.poll()) != null) {
                        task.run();
                    }
                } catch (Throwable e) {
                    e.printStackTrace();
                }
            }

		    long deltaTime = System.nanoTime() - prevTime;
		    float dt = deltaTime / 1000.f;
		    float updatePeriod = 1 / 60.f;
            if (dt < updatePeriod) {
                continue;
            }
            prevTime = System.nanoTime();
		    dynamicsWorld.stepSimulation(dt);
        }
    }

    @Override
    public void Physics_Shutdown() {
        g_operationQueue.clear();
        g_physicsQuit = true;
        try {
            g_physicsThread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        dynamicsWorld.destroy();
    }
}