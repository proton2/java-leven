package dc;

import core.buffers.DebugMeshVBO;
import core.buffers.MeshDcVBO;
import core.buffers.MeshVBO;
import core.configs.CCW;
import core.configs.CW;
import core.kernel.Camera;
import core.kernel.Input;
import core.kernel.Window;
import core.math.Matrix4f;
import core.math.Vec2f;
import core.math.Vec3f;
import core.math.Vec4f;
import core.model.Mesh;
import core.model.Vertex;
import core.physics.JBulletPhysics;
import core.physics.JniNativeBulletPhysics;
import core.physics.LibGdxBulletPhysics;
import core.physics.Physics;
import core.renderer.RenderInfo;
import core.renderer.Renderer;
import core.scene.GameObject;
import core.utils.Constants;
import dc.entities.DebugDrawBuffer;
import dc.impl.LevenLinearCPUOctreeImpl;
import dc.impl.LevenLinearGPUOctreeImpl;
import dc.impl.MeshGenerationContext;
import dc.impl.opencl.ComputeContext;
import dc.impl.opencl.KernelNames;
import dc.impl.opencl.KernelsHolder;
import dc.impl.opencl.OCLUtils;
import dc.shaders.DcSimpleShader;
import dc.shaders.RenderDebugShader;
import dc.utils.RenderDebugCmdBuffer;
import dc.utils.RenderShape;
import dc.utils.SimplexNoise;
import dc.utils.VoxelHelperUtils;

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
    protected boolean drawCamRay = false;
    private final MeshGenerationContext meshGenCtx;
    private final ComputeContext ctx;

    private Matrix4f invProjectionMatrix;
    private Matrix4f invViewMatrix;
    private Vec3f mouseDir;
    private Vec4f tmpVec;
    private Physics physics;

    // Uncomment necessary implementation in constructor
    public ChunkOctreeWrapper() {
        invProjectionMatrix = new Matrix4f();
        invViewMatrix = new Matrix4f();
        mouseDir = new Vec3f();
        tmpVec = new Vec4f();

        meshGenCtx = new MeshGenerationContext(64);
        SimplexNoise.getInstance("./res/floatArray.dat", meshGenCtx.worldSizeXZ);
        ctx = null;
        physics = new JBulletPhysics(meshGenCtx.worldBounds);
        Camera camera = Camera.getInstance();
        camera.setPosition(new Vec3f(-131.29f,-158.04f,-1921.52f));
        camera.setForward(new Vec3f(0.54f,-0.31f,0.77f).normalize());
        camera.setUp(new Vec3f(0.18f,0.94f,0.26f));
        camera.setPhysics(physics);
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
            //VoxelOctree voxelOctree = new PointerBasedOctreeImpl(true, meshGenCtx);
            //VoxelOctree voxelOctree = new SimpleLinearOctreeImpl(meshGenCtx);
            //VoxelOctree voxelOctree = new TransitionLinearOctreeImpl(meshGenCtx);
            //VoxelOctree voxelOctree = new LevenLinearCPUOctreeImpl(meshGenCtx);
            //VoxelOctree voxelOctree = new ManifoldDCOctreeImpl(meshGenCtx);
            voxelOctree = new LevenLinearCPUOctreeImpl(meshGenCtx);
        }

        //        Camera.getInstance().setPosition(new Vec3f(906,-3109,-2694));
//        Camera.getInstance().setForward(new Vec3f(-0.12f,-0.99f,-0.001f).normalize());
//        Camera.getInstance().setUp(new Vec3f(-0.99f,0.12f,-0.008f));
        chunkOctree = new ChunkOctree(voxelOctree, meshGenCtx, physics, true, camera);

//        Camera.getInstance().setPosition(new Vec3f(-286,0,-1908));
//        Camera.getInstance().setForward(new Vec3f(0.54f,-0.31f,0.77f).normalize());
//        Camera.getInstance().setUp(new Vec3f(0.18f,0.94f,0.26f));
        logger.log(Level.SEVERE, "{0}={1}", new Object[]{"Initialise", "complete"});
    }

    public void update() {
        if (refreshMesh) {
            if(Input.getInstance().isButtonHolding(1)){
                selectItem(mouseDir);
                Vec3f dir = Camera.getInstance().getForward().getNormalDominantAxis();
                Vec3f brushSize = new Vec3f(1);
                Vec3f offset = dir.mul(brushSize);
                Vec3f origin = offset.add(mouseDir);
                chunkOctree.queueCSGOperation(origin, brushSize, RenderShape.RenderShape_None, 1, true);
            }
            chunkOctree.processCSGOperations(false);
            chunkOctree.update(Camera.getInstance(), true);
        }

        if (refreshMesh) {
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
        if (Input.getInstance().isKeyHold(GLFW_KEY_F5)) {
            sleep(200);
            drawCamRay = !drawCamRay;
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_F6)) {
            Vec3f cam = Camera.getInstance().getPosition();
            sleep(200);
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_SPACE)) {
            physics.Physics_PlayerJump();
            sleep(200);
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_F10)) {
            sleep(200);
            physics.Physics_TogglePlayerNoClip();
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

        if (drawCamRay) {
            RenderDebugCmdBuffer camRayCmds = new RenderDebugCmdBuffer();
            Vec3f camRayEnd = chunkOctree.getRayCollisionPos();
            //chunkOctree.getCollisionPos()
            //camRayCmds.addWireCube(Constants.Yellow, 0.2f, chunkOctree.getCollisionPos(), 10);
            camRayCmds.addWireCube(Constants.Yellow, 0.2f, camRayEnd, 10);
            //camRayCmds.addSphere(Constants.Red, 0.2f, camRayEnd, 10);
            //camRayCmds.addLine(Constants.Red, 0.2f, Camera.getInstance().getPosition(), camRayEnd);

            DebugDrawBuffer buf = camRayCmds.UpdateDebugDrawBuffer();
            DebugMeshVBO camRayBuff = new DebugMeshVBO();
            camRayBuff.addData(buf);
            Renderer debugRenderer = new Renderer(camRayBuff);
            debugRenderer.setRenderInfo(new RenderInfo(new CW(), RenderDebugShader.getInstance()));
            addComponent(Constants.RENDERER_COMPONENT, debugRenderer);
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

    private void selectItem(Vec3f mouseDir){
        int wdwWitdh = Window.getInstance().getWidth();
        int wdwHeight = Window.getInstance().getHeight();
        Vec2f mousePos = Input.getInstance().getCursorPosition();
        Camera camera = Camera.getInstance();

        float x = (2 * mousePos.X) / (float)wdwWitdh - 1.0f;
        float y = 1.0f - (2 * mousePos.Y) / (float)wdwHeight;
        float z = -1.0f;

        invProjectionMatrix.set(camera.getProjectionMatrix().invert());

        tmpVec.set(x, y, z, 1.0f);
        tmpVec.set(tmpVec.mul(invProjectionMatrix));
        tmpVec.z = -1.0f;
        tmpVec.w = 0.0f;

        invViewMatrix.set(camera.getViewMatrix().invert());
        tmpVec.set(tmpVec.mul(invViewMatrix));

        mouseDir.set(tmpVec.x, tmpVec.y, tmpVec.z);
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
