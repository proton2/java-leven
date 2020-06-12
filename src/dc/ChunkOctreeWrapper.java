package dc;

import core.buffers.DebugMeshVBO;
import core.configs.CW;
import core.kernel.Camera;
import core.kernel.Input;
import core.renderer.RenderInfo;
import core.renderer.Renderer;
import core.scene.GameObject;
import core.utils.Constants;
import dc.entities.DebugDrawBuffer;
import dc.shaders.RenderDebugShader;
import dc.utils.Frustum;
import dc.utils.RenderDebugCmdBuffer;

import java.util.List;
import java.util.stream.Collectors;

import static dc.ChunkOctree.CLIPMAP_LEAF_SIZE;
import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;

public class ChunkOctreeWrapper extends GameObject {
    private ChunkOctree chunkOctree;
    protected boolean drawSeamBounds = false;
    protected boolean drawNodeBounds = false;
    private ChunkNode rootChunk;

    public ChunkOctreeWrapper() {
        DualContouring dc = new DualContouringImpl();
        VoxelOctree octree = new PointerBasedOctreeImpl(dc, false);
        chunkOctree = new ChunkOctree(octree);
        rootChunk = chunkOctree.buildChunkOctree();
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

    private void renderMesh() {
        getComponents().clear();
        RenderDebugCmdBuffer renderCmds = new RenderDebugCmdBuffer();
        List<ChunkNode> renderNodes = chunkOctree.update(rootChunk, Camera.getInstance()).stream().filter(e->!e.empty).collect(Collectors.toList());
        for (ChunkNode node : renderNodes) {
            if(Frustum.cubeIntoFrustum(Camera.getInstance().getFrustumPlanes(), node.min, node.size)) {//!node.empty &&
                renderCmds.addCube(node.size == CLIPMAP_LEAF_SIZE ? Constants.Blue : Constants.Green, 0.2f, node.min, node.size);
                addComponent("chunks " + node.min, node.renderMesh);
                if (node.seamMesh != null) {
                    addComponent("seams " + node.min, node.seamMesh);
                }
                if (drawSeamBounds) {
                    renderDebugVoxelsBounds(node);
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
