package core.physics;


import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.SolverType;
import com.jme3.bullet.collision.PhysicsCollisionObject;
import com.jme3.bullet.collision.PhysicsRayTestResult;
import com.jme3.bullet.collision.shapes.MeshCollisionShape;
import com.jme3.bullet.collision.shapes.infos.CompoundMesh;
import com.jme3.bullet.collision.shapes.infos.IndexedMesh;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.math.Vector3f;
import com.jme3.system.NativeLibraryLoader;
import core.math.Vec3f;
import core.math.Vec3i;
import core.model.Vertex;
import core.utils.BufferUtil;
import dc.ChunkNode;
import dc.entities.MeshBuffer;
import dc.entities.MeshVertex;
import dc.utils.Aabb;

import java.io.File;
import java.nio.FloatBuffer;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static core.physics.PhysicsOperationType.PhysicsOp_RayCast;
import static core.physics.PhysicsOperationType.PhysicsOp_WorldUpdate;

public class JniNativeBulletPhysics implements Physics
{
    // i.e. 1 voxel is PHYSICS_SCALE meters
    //final float PHYSICS_SCALE = 0.05f;
    final float PHYSICS_SCALE = 1f;
    private PhysicsSpace dynamicsWorld;
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

    public JniNativeBulletPhysics(Aabb g_worldBounds) {
        service = Executors.newFixedThreadPool(1);
        g_operationQueue = new ConcurrentLinkedQueue<>();
        Physics_Initialise(g_worldBounds);
        collisionPos = new Vec3f();
        collisionNorm = new Vec3f();
    }

    private void Physics_Initialise(Aabb worldBounds) {
        String homePath = System.getProperty("user.home");
        File downloadDirectory = new File(homePath, "Downloads");
        NativeLibraryLoader.loadLibbulletjme(true, downloadDirectory, "Release", "Sp");

        g_worldBounds = worldBounds;
        Vector3f worldMin = new Vector3f(g_worldBounds.min.x, g_worldBounds.min.y, g_worldBounds.min.z);
        Vector3f worldMax = new Vector3f(g_worldBounds.max.x, g_worldBounds.max.y, g_worldBounds.max.z);
        dynamicsWorld = new PhysicsSpace(worldMin, worldMax, PhysicsSpace.BroadphaseType.AXIS_SWEEP_3);
        dynamicsWorld.setGravity(new Vector3f(0, -9.8f, 0));
        dynamicsWorld.getSolverInfo().setNumIterations(5);

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

    public static FloatBuffer createDcFlippedBufferAOS(MeshVertex[] vertices) {
        FloatBuffer buffer = BufferUtil.createFloatBuffer(vertices.length * 9);
        for (Vertex vertice : vertices) {
            buffer.put(vertice.getPos().getX());
            buffer.put(vertice.getPos().getY());
            buffer.put(vertice.getPos().getZ());
            buffer.put(vertice.getNormal().getX());
            buffer.put(vertice.getNormal().getY());
            buffer.put(vertice.getNormal().getZ());
            buffer.put(vertice.getColor().getX());
            buffer.put(vertice.getColor().getY());
            buffer.put(vertice.getColor().getZ());
        }
        buffer.flip();
        return buffer;
    }

//    private Vector3f[] getPositionArray(MeshBuffer buf){
//        Vector3f[] array = new Vector3f[buf.getVertices().capacity()/9];
//        for (int i = 0; i < buf.getVertices().capacity()/9; i++) {
//            int index = i * 9;
//            Vector3f pos = new Vector3f();
//            pos.x = buf.getVertices().get(index + 0);
//            pos.y = buf.getVertices().get(index + 1);
//            pos.z = buf.getVertices().get(index + 2);
//            array[i] = pos;
//        }
//        return array;
//    }

    private Vector3f[] getPositionArray(MeshBuffer buf){
        Vector3f[] array = new Vector3f[buf.getNumVertices()];
        for (int i = 0; i < buf.getNumVertices(); i++) {
            int index = i * 9;
            Vector3f pos = new Vector3f();
            pos.x = buf.getVertices().get(index + 0);
            pos.y = buf.getVertices().get(index + 1);
            pos.z = buf.getVertices().get(index + 2);
            array[i] = pos;
        }
        return array;
    }

    private int[] getIndexArray(MeshBuffer buf){
        int[] arr = new int[buf.getNumIndicates()];
        for (int i=0; i<buf.getNumIndicates(); i++){
            arr[i] = buf.getIndicates().get(i);
        }
        //buf.getIndicates().flip();
        return arr;
    }

    private PhysicsMeshData addMeshToWorldImpl(Vec3i nodeMin, MeshBuffer meshBuffer){
        IndexedMesh indexedMesh = new IndexedMesh(getPositionArray(meshBuffer), getIndexArray(meshBuffer));

        PhysicsMeshData meshData = new PhysicsMeshData();
        meshData.nativeBuffer = new CompoundMesh();
        meshData.nativeBuffer.add(indexedMesh);
        meshData.nativeShape = new MeshCollisionShape(true, indexedMesh);
        meshData.nativeBody = new PhysicsRigidBody(meshData.nativeShape);
        meshData.nativeBody.setFriction(0.9f);
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
            dynamicsWorld.addCollisionObject(newMesh.nativeBody);
        }
        if (oldMesh!=null) {
            RemoveMeshData(oldMesh);
        }
    }

    @Override
    public void RemoveMeshData(PhysicsMeshData meshData) {
        dynamicsWorld.removeCollisionObject(meshData.nativeBody);
        meshData.body = null;
        meshData.shape = null;
        meshData.buffer = null;
    }

    private void CastRayImpl(Vec3f start, Vec3f end){
        Vector3f rayStart = new Vector3f(start.X, start.Y, start.Z);
        Vector3f rayEnd = new Vector3f(end.X, end.Y, end.Z);

        if(dynamicsWorld.countCollisionObjects()>0){
            int t=3;
        }
        List<PhysicsRayTestResult> rayTest = dynamicsWorld.rayTest(rayStart, rayEnd);
        if (rayTest.size() > 0) {
            PhysicsRayTestResult nearestHit = rayTest.get(0);
            PhysicsCollisionObject pco = nearestHit.getCollisionObject();
            Object user = pco.getUserObject();
            int v = 7;
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
		    dynamicsWorld.update(dt);
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
    }

}