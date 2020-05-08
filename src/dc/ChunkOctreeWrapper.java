package dc;

import core.buffers.DebugMeshVBO;
import core.configs.CW;
import core.kernel.Camera;
import core.kernel.Input;
import core.renderer.RenderInfo;
import core.renderer.Renderer;
import core.scene.GameObject;
import core.utils.Constants;
import dc.shaders.RenderDebugShader;
import dc.entities.DebugDrawBuffer;
import dc.utils.RenderDebugCmdBuffer;

import java.util.ArrayList;

import static org.lwjgl.glfw.GLFW.GLFW_KEY_F1;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_F3;
import static org.lwjgl.opengl.GL11.*;

public class ChunkOctreeWrapper extends GameObject {
    private ChunkOctree chunkOctree;
    protected boolean drawVoxelsBounds = false;
    ArrayList<ChunkNode> constructedNodes;

    public ChunkOctreeWrapper() {
        DualContouring dc = new DualContouringImpl();
        VoxelOctree octree = new VoxelOctreeImpl(dc);
        chunkOctree = new ChunkOctree(octree);
    }

    public void update() {
        if (refreshMesh) {
            refreshMesh = false;
            renderMesh();
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_F1)) {
            sleep(200);
            drawWireframe = !drawWireframe;
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_F3)) {
            sleep(200);
            refreshMesh = true;
        }
        glPolygonMode(GL_FRONT_AND_BACK, drawWireframe ? GL_LINE : GL_FILL);
        if(!drawWireframe){
            glDisable(GL_CULL_FACE);
        }
    }

    private void renderMesh() {
        RenderDebugCmdBuffer renderCmds = new RenderDebugCmdBuffer();
        ChunkNode rootChunk = chunkOctree.buildChunkOctree();
        constructedNodes = chunkOctree.update(rootChunk, Camera.getInstance(), renderCmds);

        for (ChunkNode node : constructedNodes) {
            addComponent("chunks "+node.min, node.mainMesh);
            if(node.seamMesh!=null) {
                addComponent("seams " + node.min, node.seamMesh);
            }
            if(drawVoxelsBounds) {
                renderDebugVoxelsBounds(node);
            }
        }
        DebugDrawBuffer buf = renderCmds.UpdateDebugDrawBuffer();
        DebugMeshVBO debugMeshBuffer = new DebugMeshVBO();
        debugMeshBuffer.addData(buf);
        Renderer debugRenderer = new Renderer(debugMeshBuffer);
        debugRenderer.setRenderInfo(new RenderInfo(new CW(), RenderDebugShader.getInstance()));
        addComponent(Constants.RENDERER_COMPONENT, debugRenderer);
    }

    private void renderDebugVoxelsBounds(ChunkNode node){
        RenderDebugCmdBuffer renderDebugVoxelsBounds = new RenderDebugCmdBuffer();
        for(OctreeNode n : node.seamNodes){
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
