package dc;

import core.buffers.DebugMeshVBO;
import core.buffers.MeshDcVBO;
import core.configs.CW;
import core.kernel.Camera;
import core.kernel.Input;
import core.math.Vec3f;
import core.renderer.RenderInfo;
import core.renderer.Renderer;
import core.scene.GameObject;
import core.utils.Constants;
import modules.DcSimpleShader;
import modules.RenderDebugShader;

import java.util.ArrayList;
import java.util.List;

import static org.lwjgl.glfw.GLFW.GLFW_KEY_F1;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_F3;
import static org.lwjgl.opengl.GL11.*;

public class ChunkOctreeWrapper extends GameObject {
    private ChunkOctree chunkOctree;

    public ChunkOctreeWrapper() {
        super();
        chunkOctree = new ChunkOctree();
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
        ArrayList<ChunkNode> constructedNodes = chunkOctree.update(rootChunk, Camera.getInstance(), renderCmds);

        for (ChunkNode node : constructedNodes) {
            MeshDcVBO meshBuffer = new MeshDcVBO();
            meshBuffer.addData(node.vertArray, node.indices);
            Renderer renderer = new Renderer(meshBuffer);
            renderer.setRenderInfo(new RenderInfo(new CW(), DcSimpleShader.getInstance()));
            addComponent("chunks "+node.min, renderer);
            renderNodeSeam(node);
        }

        DebugDrawBuffer buf = renderCmds.UpdateDebugDrawBuffer();
        DebugMeshVBO debugMeshBuffer = new DebugMeshVBO();
        debugMeshBuffer.addData(buf);
        Renderer debugRenderer = new Renderer(debugMeshBuffer);
        debugRenderer.setRenderInfo(new RenderInfo(new CW(), RenderDebugShader.getInstance()));
        addComponent(Constants.RENDERER_COMPONENT, debugRenderer);
    }

    private void renderNodeSeam(ChunkNode node) {
        if(node.seamVertArray!=null && node.seamVertArray.length > 0) {
            MeshDcVBO seamsMeshBuffer = new MeshDcVBO();
            seamsMeshBuffer.addData(node.seamVertArray, node.seamIndices);
            Renderer seamRenderer = new Renderer(seamsMeshBuffer);
            seamRenderer.setRenderInfo(new RenderInfo(new CW(), DcSimpleShader.getInstance()));
            addComponent("seams " + node.min, seamRenderer);
        }
    }
}
