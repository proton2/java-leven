package dc;

import core.buffers.DebugMeshVBO;
import core.buffers.MeshDcVBO;
import core.configs.CW;
import core.kernel.Camera;
import core.kernel.Input;
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
import dc.utils.RenderDebugCmdBuffer;
import dc.utils.SimplexNoise;
import dc.utils.VoxelHelperUtils;

import java.util.List;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;

public class ChunkOctreeWrapper extends GameObject {
    private final ChunkOctree chunkOctree;
    private final KernelsHolder kernelHolder;
    protected boolean drawSeamBounds = false;
    protected boolean drawNodeBounds = false;
    private final MeshGenerationContext meshGenCtx;
    private final ComputeContext ctx;

    // Uncomment necessary implementation in constructor
    public ChunkOctreeWrapper() {
        meshGenCtx = new MeshGenerationContext(64);
        SimplexNoise.getInstance("./res/textures/floatArray.dat", meshGenCtx.worldSizeXZ);
        ctx = OCLUtils.getOpenCLContext();
        StringBuilder kernelBuildOptions = VoxelHelperUtils.createMainBuildOptions(meshGenCtx);
        kernelHolder = new KernelsHolder(ctx);
        kernelHolder.buildKernel(KernelNames.DENSITY, kernelBuildOptions);
        kernelHolder.buildKernel(KernelNames.FIND_DEFAULT_EDGES, kernelBuildOptions);
        kernelHolder.buildKernel(KernelNames.SCAN, null);
        kernelHolder.buildKernel(KernelNames.OCTREE, kernelBuildOptions);
        kernelHolder.buildKernel(KernelNames.CUCKOO, kernelBuildOptions);

        //VoxelOctree voxelOctree = new PointerBasedOctreeImpl(true, meshGenCtx);
        //VoxelOctree voxelOctree = new SimpleLinearOctreeImpl(meshGenCtx);
        //VoxelOctree voxelOctree = new TransitionLinearOctreeImpl(meshGenCtx);
        //VoxelOctree voxelOctree = new LevenLinearCPUOctreeImpl(meshGenCtx);
        VoxelOctree voxelOctree = new LevenLinearGPUOctreeImpl(kernelHolder, meshGenCtx, ctx);
        chunkOctree = new ChunkOctree(voxelOctree, meshGenCtx);
    }

    public void update() {
        if (refreshMesh) {
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
        glPolygonMode(GL_FRONT_AND_BACK, drawWireframe ? GL_LINE : GL_FILL);
        if(!drawWireframe){
            glDisable(GL_CULL_FACE);
        }
    }

    public void cleanUp(){
        kernelHolder.destroyContext();
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
            renderCmds.addCube(node.size == meshGenCtx.clipmapLeafSize ? Constants.Blue : Constants.Green, 0.2f, node.min, node.size);
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
    }

    private void renderDebugVoxelsBounds(ChunkNode node){
        RenderDebugCmdBuffer renderDebugVoxelsBounds = new RenderDebugCmdBuffer();
        for(PointerBasedOctreeNode n : node.chunkBorderNodes){
            renderDebugVoxelsBounds.addCube(n.drawInfo.color, 0.2f, n.min, n.size);
        }
        DebugDrawBuffer buf = renderDebugVoxelsBounds.UpdateDebugDrawBuffer();
        DebugMeshVBO debugMeshBuffer = new DebugMeshVBO();
        debugMeshBuffer.addData(buf);
        Renderer debugRenderer = new Renderer(debugMeshBuffer);
        debugRenderer.setRenderInfo(new RenderInfo(new CW(), RenderDebugShader.getInstance()));
        addComponent("voxel nodes " + node.min, debugRenderer);
    }
}
