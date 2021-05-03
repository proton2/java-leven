package dc;

import core.buffers.DebugMeshVBO;
import core.buffers.MeshDcVBO;
import core.buffers.MeshVBO;
import core.configs.CCW;
import core.configs.CW;
import core.kernel.Camera;
import core.kernel.Input;
import core.math.Vec2f;
import core.math.Vec3f;
import core.model.Mesh;
import core.model.Vertex;
import core.physics.JBulletPhysics;
import core.physics.Physics;
import core.renderer.RenderInfo;
import core.renderer.Renderer;
import core.scene.GameObject;
import core.utils.Constants;
import dc.entities.DebugDrawBuffer;
import dc.impl.*;
import dc.impl.opencl.ComputeContext;
import dc.impl.opencl.KernelNames;
import dc.impl.opencl.KernelsHolder;
import dc.impl.opencl.OCLUtils;
import dc.shaders.DcSimpleShader;
import dc.shaders.RenderDebugShader;
import dc.utils.*;

import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;

public class ChunkOctreeWrapper extends GameObject {

    final public static Logger logger = Logger.getLogger(ChunkOctreeWrapper.class.getName());

    private final ChunkOctree chunkOctree;
    private KernelsHolder kernelHolder;
    protected boolean drawSeamBounds = false;
    protected boolean drawNodeBounds = false;
    private final MeshGenerationContext meshGenCtx;
    private final ComputeContext ctx;
    private Physics physics;
    private boolean enablePhysics = true;
    private int brushSize = 10;
    //private ModelEntity actorCSGCube;

    // Uncomment necessary implementation in constructor
    public ChunkOctreeWrapper() {
        meshGenCtx = new MeshGenerationContext(64);
        SimplexNoise.getInstance("./res/floatArray.dat", meshGenCtx.worldSizeXZ);
        ctx = null;//OCLUtils.getOpenCLContext();
        //actorCSGCube = new ModelEntity(new RenderDebugCmdBuffer().createCube());
        physics = new JBulletPhysics(meshGenCtx.worldBounds, 256);
        Camera camera = Camera.getInstance();
        camera.setPosition(new Vec3f(-131.29f,-158.04f,-1921.52f));
        if(enablePhysics) {
            camera.setPhysics(physics);
        }
        VoxelOctree voxelOctree;
        if(ctx!=null) {
            StringBuilder kernelBuildOptions = VoxelHelperUtils.createMainBuildOptions(meshGenCtx);
            kernelHolder = new KernelsHolder(ctx);
            kernelHolder.buildKernel(KernelNames.DENSITY, kernelBuildOptions);
            kernelHolder.buildKernel(KernelNames.FIND_DEFAULT_EDGES, kernelBuildOptions);
            kernelHolder.buildKernel(KernelNames.SCAN, null);
            kernelHolder.buildKernel(KernelNames.OCTREE, kernelBuildOptions);
            kernelHolder.buildKernel(KernelNames.CUCKOO, kernelBuildOptions);
            voxelOctree = new LevenLinearGPUOctreeImpl(kernelHolder, meshGenCtx, ctx);
        } else{
            //voxelOctree = new PointerBasedOctreeImpl(true, meshGenCtx);
            voxelOctree = new SimpleLinearOctreeImpl(meshGenCtx);
            //voxelOctree = new TransitionLinearOctreeImpl(meshGenCtx);
            //VoxelOctree voxelOctree = new LevenLinearCPUOctreeImpl(meshGenCtx);
            //VoxelOctree voxelOctree = new ManifoldDCOctreeImpl(meshGenCtx);
            //voxelOctree = new LevenLinearCPUOctreeImpl(meshGenCtx);
        }
        chunkOctree = new ChunkOctree(voxelOctree, meshGenCtx, physics, enablePhysics, camera);
        logger.log(Level.SEVERE, "{0}={1}", new Object[]{"Initialise", "complete"});
    }

    public void update() {
        if (refreshMesh) {
            Camera cam = Camera.getInstance();
            Vec2f curPos = Input.getInstance().getCursorPosition();
            Ray ray = cam.getMousePickRay(curPos.X, curPos.Y);
            Vec3f rayTo = new Vec3f(ray.direction.scaleAdd(Constants.ZFAR, ray.origin));
            chunkOctree.processCSGOperations();
            chunkOctree.update(cam, ray.origin, rayTo);
            renderMesh();
        }

        if (Input.getInstance().isKeyHold(GLFW_KEY_F1)) {
            sleep(200);
            drawWireframe = !drawWireframe;
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_F2)) {
            sleep(200);
            drawNodeBounds = !drawNodeBounds;
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_F3)) {
            sleep(200);
            refreshMesh = !refreshMesh;
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_F4)) {
            sleep(200);
            drawSeamBounds = !drawSeamBounds;
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_F6)) {
            Vec3f cam = Camera.getInstance().getPosition();
            sleep(200);
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_SPACE)) {
            physics.Physics_PlayerJump();
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_F10)) {
            sleep(200);
            physics.Physics_TogglePlayerNoClip();
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_RIGHT_BRACKET)) {
            sleep(100);
            brushSize +=5;
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_LEFT_BRACKET)) {
            sleep(100);
            brushSize -=5;
        }

        glPolygonMode(GL_FRONT_AND_BACK, drawWireframe ? GL_LINE : GL_FILL);
        if(!drawWireframe){
            glDisable(GL_CULL_FACE);
        }
    }

    public void cleanUp(){
        if(kernelHolder!=null) {
            kernelHolder.destroyContext();
        }
        chunkOctree.clean();
    }

    private Renderer getRenderer(RenderMesh node){
        if(node.getRender()==null) {
            MeshDcVBO meshDcVBO = new MeshDcVBO(node.meshBuffer);
            Renderer renderer = new Renderer(meshDcVBO, new RenderInfo(new CW(), DcSimpleShader.getInstance()));
            node.setRender(renderer);
        }
        return node.getRender();
    }

    private void deleteRenderer(RenderMesh mesh){
        if(mesh.getRender()!=null){
            mesh.getRender().getVbo().delete();
            mesh.setRender(null);
        }
    }

    private void renderMesh() {
        getComponents().clear();
        RenderDebugCmdBuffer renderCmds = new RenderDebugCmdBuffer();
        List<RenderMesh> invalidateMeshes = chunkOctree.getInvalidateMeshes();
        for(RenderMesh mesh : invalidateMeshes){
            deleteRenderer(mesh);
        }
        chunkOctree.getInvalidateMeshes().clear();
        List<RenderMesh> renderNodes = chunkOctree.getRenderMeshes(true);
        int i=0;
        for (RenderMesh node : renderNodes) {
            if(drawNodeBounds) {
                renderCmds.addWireCube(node.size == meshGenCtx.clipmapLeafSize ? Constants.Blue : Constants.Green, 0.2f, node.min.toVec3f(), node.size);
            }
            addComponent("mesh " + (++i), getRenderer(node));
        }

        if(drawNodeBounds) {
            DebugDrawBuffer buf = renderCmds.UpdateDebugDrawBuffer();
            DebugMeshVBO debugMeshBuffer = new DebugMeshVBO();
            debugMeshBuffer.addData(buf);
            Renderer debugRenderer = new Renderer(debugMeshBuffer);
            debugRenderer.setRenderInfo(new RenderInfo(new CW(), RenderDebugShader.getInstance()));
            addComponent(Constants.RENDERER_COMPONENT, debugRenderer);
        }

        if (enablePhysics && Input.getInstance().isButtonHolding(1)) {
            RenderDebugCmdBuffer camRayCmds = new RenderDebugCmdBuffer();
            camRayCmds.addWireCube(Constants.Yellow, 0.2f, chunkOctree.getRayCollisionPos(), brushSize);
            //camRayCmds.addSphere(Constants.Red, 0.2f, chunkOctree.getRayCollisionPos(), 10);
            //camRayCmds.addLine(Constants.Green, 0.2f, Camera.getInstance().getPosition(), chunkOctree.getRayCollisionPos());

            /*Renderer debugRenderer = new Renderer(actorCSGCube.getVbo());
            actorCSGCube.setTranslation(chunkOctree.getRayCollisionPos());
            CSGActorShader csgActorShader = CSGActorShader.getInstance();
            csgActorShader.updateTransform(actorCSGCube);
            debugRenderer.setRenderInfo(new RenderInfo(new CW(), csgActorShader));
             */
            DebugDrawBuffer buf = camRayCmds.UpdateDebugDrawBuffer();
            DebugMeshVBO camRayBuff = new DebugMeshVBO();
            camRayBuff.addData(buf);
            Renderer debugRenderer = new Renderer(camRayBuff);
            debugRenderer.setRenderInfo(new RenderInfo(new CW(), RenderDebugShader.getInstance()));

            addComponent(Constants.RENDERER_COMPONENT, debugRenderer);

            Vec3f dir = Camera.getInstance().getForward().getNormalDominantAxis();
            Vec3f brushSizeV = new Vec3f(brushSize);
            Vec3f offset = dir.mul(brushSizeV);
            Vec3f origin = offset.add(chunkOctree.getRayCollisionPos());
            chunkOctree.queueCSGOperation(origin, brushSizeV, RenderShape.RenderShape_None, 1, true);
        }
    }

    private void renderDebugVoxelsBounds(ChunkNode node){
        RenderDebugCmdBuffer renderDebugVoxelsBounds = new RenderDebugCmdBuffer();
        for(OctreeNode n : node.chunkBorderNodes){
            renderDebugVoxelsBounds.addCube(Constants.Green, 0.2f, n.min.toVec3f(), n.size);
        }
        DebugDrawBuffer buf = renderDebugVoxelsBounds.UpdateDebugDrawBuffer();
        DebugMeshVBO debugMeshBuffer = new DebugMeshVBO();
        debugMeshBuffer.addData(buf);
        Renderer debugRenderer = new Renderer(debugMeshBuffer);
        debugRenderer.setRenderInfo(new RenderInfo(new CW(), RenderDebugShader.getInstance()));
        addComponent("voxel nodes " + node.min, debugRenderer);
    }

    private void testPlane() {
        Vertex[] vertArray = new Vertex[4];
        vertArray[0] = new Vertex(new Vec3f(-50.5f, 0f, 50.5f));
        vertArray[1] = new Vertex(new Vec3f(-50.5f, 0f, -50.5f));
        vertArray[2] = new Vertex(new Vec3f(50.5f, 0f, -50.5f));
        vertArray[3] = new Vertex(new Vec3f(50.5f, 0f, 50.5f));
        int[] indices = {
                0, 1, 3,//top left triangle (v0, v1, v3)
                3, 1, 2//bottom right triangle (v3, v1, v2)
        };

        Mesh mesh = new Mesh(vertArray, indices);
        MeshVBO meshBuffer = new MeshVBO();
        meshBuffer.addData(mesh);
        Renderer testRenderer = new Renderer(meshBuffer);
        testRenderer.setRenderInfo(new RenderInfo(new CCW(), DcSimpleShader.getInstance()));
        addComponent(Constants.RENDERER_COMPONENT, testRenderer);
    }
}
