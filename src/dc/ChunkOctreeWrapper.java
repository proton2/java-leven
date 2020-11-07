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
    private ComputeContext ctx;

    // Uncomment necessary implementation in constructor
    public ChunkOctreeWrapper() {
        meshGenCtx = new MeshGenerationContext(64);
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
        //VoxelOctree voxelOctree = new LevenLinearOpenCLOctreeImpl(kernelHolder, meshGenCtx);
        VoxelOctree voxelOctree = new LevenLinearTestOctreeImpl(kernelHolder, meshGenCtx);
        chunkOctree = new ChunkOctree(voxelOctree, meshGenCtx);
    }

    public void update() {
        if (refreshMesh) {
            chunkOctree.update(Camera.getInstance(), false);
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

    private void renderMesh() {
        getComponents().clear();
        RenderDebugCmdBuffer renderCmds = new RenderDebugCmdBuffer();

        List<RenderMesh> renderNodes = chunkOctree.getRenderMeshes(true);

        for (RenderMesh node : renderNodes) {
            renderCmds.addCube(node.size == meshGenCtx.clipmapLeafSize ? Constants.Blue : Constants.Green, 0.2f, node.min, node.size);
            if (node.meshRender == null) {
                Renderer renderer = new Renderer(new MeshDcVBO(node.renderMesh));
                renderer.setRenderInfo(new RenderInfo(new CW(), DcSimpleShader.getInstance()));
                node.meshRender = renderer;
            }
            addComponent("chunks " + node.min, node.meshRender);
            if (node.seamMesh != null) {
                if (node.seamRender == null) {
                    Renderer seamRenderer = new Renderer(new MeshDcVBO(node.seamMesh));
                    seamRenderer.setRenderInfo(new RenderInfo(new CW(), DcSimpleShader.getInstance()));
                    node.seamRender = seamRenderer;
                }
                addComponent("seams " + node.min, node.seamRender);
            }
//          if (drawSeamBounds) {
//              renderDebugVoxelsBounds(node);
//          }
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
        for(PointerBasedOctreeNode n : node.seamNodes){
            renderDebugVoxelsBounds.addCube(Constants.White, 0.2f, n.min, n.size);
        }
        DebugDrawBuffer buf = renderDebugVoxelsBounds.UpdateDebugDrawBuffer();
        DebugMeshVBO debugMeshBuffer = new DebugMeshVBO();
        debugMeshBuffer.addData(buf);
        Renderer debugRenderer = new Renderer(debugMeshBuffer);
        debugRenderer.setRenderInfo(new RenderInfo(new CW(), RenderDebugShader.getInstance()));
        addComponent("voxel nodes " + node.min, debugRenderer);
    }
}
