package dc;

import core.buffers.MeshDcVBO;
import core.buffers.MeshVBO;
import core.configs.CCW;
import core.configs.CW;
import core.kernel.Input;
import core.math.Vec3f;
import core.math.Vec3i;
import core.model.Mesh;
import core.model.Vertex;
import core.renderer.RenderInfo;
import core.renderer.Renderer;
import core.scene.GameObject;
import core.utils.Constants;
import modules.DcSimpleShader;

import java.util.ArrayList;
import java.util.List;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.opengl.GL11.*;

/**
 * Created by proton2 on 28.12.2019.
 */
public class DcWrapper extends GameObject {
    private OctreeNode root;
    private int thresholdIndex = 0;
    private float[] THRESHOLDS = {-1.0f, 0.1f, 1.0f, 10.0f, 50.0f};
    //public float[] THRESHOLDS = {0.0f, 0.001f, 0.01f, 0.05f, 0.1f, 0.2f, 0.4f, 0.5f, 0.8f, 1.0f, 1.5f, 2.0f, 5.0f, 10.0f, 25.0f, 50.0f, 100.0f, 250.0f, 500.0f, 1000.0f, 2500.0f, 5000.0f, 10000.0f, 25000.0f, 50000.0f, 100000.0f};
    private List<MeshVertex> vertices;
    private List<Integer> indcies;

    // octreeSize must be a power of two!
    private int octreeSize = 64;

    public DcWrapper() {
        vertices = new ArrayList<>();
        indcies = new ArrayList<>();
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
        if (Input.getInstance().isKeyHold(GLFW_KEY_F2)) {
            sleep(200);
            refreshMesh = true;
            thresholdIndex = (thresholdIndex + 1) % THRESHOLDS.length;
        }
        if (Input.getInstance().isKeyHold(GLFW_KEY_F3)) {
            sleep(200);
            refreshMesh = true;
        }
        glPolygonMode(GL_FRONT_AND_BACK, drawWireframe ? GL_LINE : GL_FILL);
    }

    private void renderMesh() {
        root = Octree.BuildOctree(getTransform().getFrustumPlanes(),
                new Vec3i(-octreeSize / 2, -octreeSize / 2, -octreeSize / 2), octreeSize, THRESHOLDS[thresholdIndex]);
        Octree.GenerateMeshFromOctree(root, vertices, indcies, false);

        Vertex[] vertArray = vertices.toArray(new Vertex[0]);
        int[] indices = indcies.stream().mapToInt(x -> x).toArray();

        MeshDcVBO meshBuffer = new MeshDcVBO();
        meshBuffer.addData(vertArray, indices);
        Renderer renderer = new Renderer(meshBuffer);
        renderer.setRenderInfo(new RenderInfo(new CW(), DcSimpleShader.getInstance()));
        addComponent(Constants.RENDERER_COMPONENT, renderer);
    }

    public void shutDown() {
        Octree.DestroyOctree(root);
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