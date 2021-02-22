package core.physics;

import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.physics.bullet.collision.*;
import com.badlogic.gdx.physics.bullet.dynamics.*;
import com.badlogic.gdx.utils.SharedLibraryLoader;
import core.math.Vec3f;
import core.math.Vec3i;
import dc.ChunkNode;
import dc.entities.MeshBuffer;
import dc.utils.Aabb;

import javax.vecmath.Vector3f;
import java.nio.ByteBuffer;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.logging.Level;
import java.util.logging.Logger;

import static core.physics.PhysicsOperationType.PhysicsOp_RayCast;
import static core.physics.PhysicsOperationType.PhysicsOp_WorldUpdate;

public class LibGdxBulletPhysics implements Physics{

    final public static Logger logger = Logger.getLogger(LibGdxBulletPhysics.class.getName());

    // i.e. 1 voxel is PHYSICS_SCALE meters
    //final float PHYSICS_SCALE = 0.05f;
    final float PHYSICS_SCALE = 1f;
    public btCollisionWorld collisionWorld;

    private Aabb g_worldBounds;
    private boolean g_physicsQuit = false;
    private ConcurrentLinkedQueue<Runnable> g_operationQueue;
    private Thread g_physicsThread;
    private ExecutorService service;
    private volatile boolean g_rayCastPending = false;
    private Vector3 collisionPos;
    private Vector3 collisionNorm;
    private Vec3f collPos = new Vec3f();
    private Vec3f collNorm = new Vec3f();

    private float closestHitFraction = Float.MAX_VALUE;

    ClosestRayResultCallback rayTestCB;
    Vector3 rayFrom = new Vector3();
    Vector3 rayTo = new Vector3();
    public int maxSubSteps = 5;
    public float fixedTimeStep = 1f / 60f;

    private void EnqueuePhysicsOperation(PhysicsOperationType opType, Runnable op) {
        try {
            g_operationQueue.add(op);
        } catch (Throwable e){
            e.printStackTrace();
        }
    }

    @Override
    public Vec3f getCollisionNorm() {
        return collNorm;
    }

    @Override
    public Vec3f getCollisionPos() {
        return collPos;
    }

    private Vec3f Scale_WorldToPhysics(Vec3f worldValue) {
        return new Vec3f(worldValue.X * PHYSICS_SCALE, worldValue.Y * PHYSICS_SCALE, worldValue.Z * PHYSICS_SCALE);
    }

    private Vector3f Scale_WorldToPhysics(Vector3f worldValue) {
        return new Vector3f(worldValue.x * PHYSICS_SCALE, worldValue.y * PHYSICS_SCALE, worldValue.z * PHYSICS_SCALE);
    }

    private Vector3f Scale_PhysicsToWorld(Vector3f worldValue) {
        return new Vector3f(worldValue.x / PHYSICS_SCALE, worldValue.y / PHYSICS_SCALE, worldValue.z / PHYSICS_SCALE);
    }

    public LibGdxBulletPhysics(Aabb g_worldBounds) {
        service = Executors.newFixedThreadPool(1);
        g_operationQueue = new ConcurrentLinkedQueue<>();
        Physics_Initialise(g_worldBounds);
        collisionPos = new Vector3();
        collisionNorm = new Vector3();
    }

    private void Physics_Initialise(Aabb worldBounds) {
        new SharedLibraryLoader().load("gdx-bullet");

        g_worldBounds = worldBounds;

        btOverlappingPairCache pairCache = new btHashedOverlappingPairCache();
        Vector3 worldMin = new Vector3(g_worldBounds.min.x, g_worldBounds.min.y, g_worldBounds.min.z);
        Vector3 worldMax = new Vector3(g_worldBounds.max.x, g_worldBounds.max.y, g_worldBounds.max.z);
	    int maxHandles = 3500;

        //btBroadphaseInterface broadphase = new btDbvtBroadphase();
        btBroadphaseInterface broadphase = new btAxisSweep3(worldMin, worldMax, maxHandles, pairCache);
        btConstraintSolver solver = new btSequentialImpulseConstraintSolver();

        btCollisionConfiguration collisionConfiguration = new btDefaultCollisionConfiguration();
        btCollisionDispatcher dispatcher = new btCollisionDispatcher(collisionConfiguration);
        collisionWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

        ((btDynamicsWorld)collisionWorld).setGravity(new Vector3(0, -9.8f, 0));
        ((btDynamicsWorld)collisionWorld).getSolverInfo().setNumIterations(5);

        rayTestCB = new ClosestRayResultCallback(Vector3.Zero, Vector3.Z);

        g_physicsQuit = false;
        g_physicsThread = new Thread(this::PhysicsThreadFunction);
        g_physicsThread.start();

        logger.log(Level.SEVERE, "{0}={1}", new Object[]{"Physics", "Initialise"});
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
        ByteBuffer indicatesByteBuffer = ByteBuffer.allocateDirect(Integer.BYTES * meshBuffer.getNumIndicates());
        indicatesByteBuffer.asIntBuffer().put(meshBuffer.getIndicates());
        meshBuffer.getIndicates().flip();

        ByteBuffer verticesByteBuffer = ByteBuffer.allocateDirect(Float.BYTES * 3 * meshBuffer.getNumVertices());
        for (int i = 0; i < meshBuffer.getNumVertices(); i++) {
            int index = i * 9;
            float x = meshBuffer.getVertices().get(index+0) * PHYSICS_SCALE;
            float y = meshBuffer.getVertices().get(index+1) * PHYSICS_SCALE;
            float z = meshBuffer.getVertices().get(index+2) * PHYSICS_SCALE;
            verticesByteBuffer.putFloat(x);
            verticesByteBuffer.putFloat(y);
            verticesByteBuffer.putFloat(z);
        }
        verticesByteBuffer.flip();

        btIndexedMesh indexedMesh = new btIndexedMesh();
        indexedMesh.setVertexBase(verticesByteBuffer);
        indexedMesh.setVertexStride(3 * Float.BYTES);
        indexedMesh.setNumVertices(meshBuffer.getNumVertices());
        indexedMesh.setTriangleIndexBase(indicatesByteBuffer);
        indexedMesh.setTriangleIndexStride(3 * Float.BYTES);
        indexedMesh.setNumTriangles(meshBuffer.getNumIndicates() / 3);

        PhysicsMeshData meshData = new PhysicsMeshData();
        meshData.gdxbuffer = new btTriangleIndexVertexArray();
        meshData.gdxbuffer.addIndexedMesh(indexedMesh);
        meshData.gdxShape = new btBvhTriangleMeshShape(meshData.gdxbuffer, true, true);
        meshData.gdxBody = new btRigidBody(0, null, meshData.gdxShape);
        meshData.gdxBody.setFriction(0.9f);

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
            collisionWorld.addCollisionObject(newMesh.gdxBody);
        }
        if (oldMesh!=null) {
            RemoveMeshData(oldMesh);
        }
    }

    @Override
    public void RemoveMeshData(PhysicsMeshData meshData) {
        collisionWorld.removeCollisionObject(meshData.gdxBody);
        meshData.body = null;
        meshData.shape = null;
        meshData.buffer = null;
    }

    private void CastRayImpl(Vec3f start, Vec3f end){
        Vector3 rayStart = new Vector3(start.X, start.Y, start.Z);
        Vector3 rayEnd = new Vector3(end.X, end.Y, end.Z);

        rayTestCB.setCollisionObject(null);
        rayTestCB.setClosestHitFraction(500f);
        rayTestCB.setRayFromWorld(rayFrom);
        rayTestCB.setRayToWorld(rayTo);

        collisionWorld.rayTest(rayStart, rayEnd, rayTestCB);
        if(collisionWorld.getNumCollisionObjects()>0){
            int t=3;
        }

        if (rayTestCB.hasHit()) {
            final btCollisionObject obj = rayTestCB.getCollisionObject();
            if (!obj.isStaticOrKinematicObject()) {
                //final btRigidBody body = (btRigidBody)(obj);
                rayTestCB.getHitPointWorld(collisionPos);
                collPos.X = collisionPos.x;
                collPos.Y = collisionPos.y;
                collPos.Z = collisionPos.z;

                rayTestCB.getHitNormalWorld(collisionNorm);
                collNorm.X = collisionNorm.x;
                collNorm.Y = collisionNorm.y;
                collNorm.Z = collisionNorm.z;
                collNorm.normalize();
                closestHitFraction = rayTestCB.getClosestHitFraction();
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

            if (collisionWorld instanceof btDynamicsWorld)
                ((btDynamicsWorld)collisionWorld).stepSimulation(dt, maxSubSteps, fixedTimeStep);
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
        if (rayTestCB != null) rayTestCB.dispose();
        rayTestCB = null;
        collisionWorld.dispose();
    }
}