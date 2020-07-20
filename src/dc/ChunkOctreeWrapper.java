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
import dc.impl.TransitionLinearOctreeImpl;
import dc.shaders.DcSimpleShader;
import dc.shaders.RenderDebugShader;
import dc.utils.Frustum;
import dc.utils.RenderDebugCmdBuffer;

import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.stream.Collectors;

import static dc.ChunkOctree.CLIPMAP_LEAF_SIZE;
import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;

public class ChunkOctreeWrapper extends GameObject {
    private ChunkOctree chunkOctree;
    protected boolean drawSeamBounds = false;
    protected boolean drawNodeBounds = false;
    private ChunkNode rootChunk;
    private List<ChunkNode> renderNodes;
    ExecutorService service;

    // Uncomment necessary implementation in constructor
    public ChunkOctreeWrapper() {
        //ComputeShaderTest computeShaderTest = new ComputeShaderTest(1);
        //computeShaderTest.render();
        //chunkOctree = new ChunkOctree(new PointerBasedOctreeImpl());
        //chunkOctree = new ChunkOctree(new SimpleLinearOctreeImpl());
        chunkOctree = new ChunkOctree(new TransitionLinearOctreeImpl());
        //chunkOctree = new ChunkOctree(new LevenLinearOctreeImpl());
        rootChunk = chunkOctree.buildChunkOctree();

        service = Executors.newFixedThreadPool(1);
    }

    public void update() {
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

    void refreshCallBack(List<ChunkNode> refreshedNodes){
        this.renderNodes = refreshedNodes;
    }

    private void renderMesh() {
        getComponents().clear();
        RenderDebugCmdBuffer renderCmds = new RenderDebugCmdBuffer();

        service.submit(()->{
            List<ChunkNode> refreshedNodes = chunkOctree.update(rootChunk, Camera.getInstance()).stream().filter(e->!e.empty).collect(Collectors.toList());
            refreshCallBack(refreshedNodes);
        });

        if(renderNodes!=null) {
            for (ChunkNode node : renderNodes) {
                if (Frustum.cubeIntoFrustum(Camera.getInstance().getFrustumPlanes(), node.min, node.size)) {//!node.empty &&
                    renderCmds.addCube(node.size == CLIPMAP_LEAF_SIZE ? Constants.Blue : Constants.Green, 0.2f, node.min, node.size);
                    if (node.meshRender == null) {
                        Renderer renderer = new Renderer(new MeshDcVBO(node.renderMesh));
                        node.renderMesh.getVertices().clear();
                        node.renderMesh.getIndicates().clear();
                        renderer.setRenderInfo(new RenderInfo(new CW(), DcSimpleShader.getInstance()));
                        node.meshRender = renderer;
                    }
                    addComponent("chunks " + node.min, node.meshRender);
                    if (node.seamMesh != null) {
                        if (node.seamRender == null) {
                            Renderer seamRenderer = new Renderer(new MeshDcVBO(node.seamMesh));
                            node.seamMesh.getVertices().clear();
                            node.seamMesh.getIndicates().clear();
                            seamRenderer.setRenderInfo(new RenderInfo(new CW(), DcSimpleShader.getInstance()));
                            node.seamRender = seamRenderer;
                        }
                        addComponent("seams " + node.min, node.seamRender);
                    }
                    if (drawSeamBounds) {
                        renderDebugVoxelsBounds(node);
                    }
                }
            }
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
