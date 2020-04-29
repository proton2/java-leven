package dc;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;


import java.util.ArrayList;

public class RenderDebugCmdBuffer {
    private static int NUM_INDICATES = 24;
    private static int NUM_VERTICES = 8;
    static class RenderDebugCmd {
        Vec3i min, max;
        Vec3f rgb;
        float alpha;
    }

    ArrayList<RenderDebugCmd> cmds = new ArrayList<>();
    int vertexBufferSize = 0;
    int indexBufferSize = 0;
    int currVertexBufferSize, currIndexBufferSize;

    public void addCube(Vec3f rgb, float alpha, Vec3i min, int size) {
        RenderDebugCmd cmd = new RenderDebugCmd();
        cmd.alpha = alpha;
        cmd.rgb = rgb;
        cmd.min = min;
        cmd.max = new Vec3i();
        cmd.max.x = min.x + size;
        cmd.max.y = min.y + size;
        cmd.max.z = min.z + size;
        cmds.add(cmd);
    }

    public DebugDrawBuffer UpdateDebugDrawBuffer() {
        for (int i = cmds.size() - 1; i >= 0; i--) {
            int vertices = NUM_VERTICES;
            int indices = NUM_INDICATES;
            vertexBufferSize += vertices;
            indexBufferSize += indices;
        }
        Vec4f[] vertexBuffer = new Vec4f[vertexBufferSize];
        Vec4f[] colourBuffer = new Vec4f[vertexBufferSize];
        int[] indexBuffer = new int[indexBufferSize];

        for (int i = cmds.size() - 1; i >= 0; i--) {
            RenderDebugCmd cmd = cmds.get(i);
            Vec4f colour = new Vec4f(cmd.rgb, cmd.alpha);
            int numVerticesBefore = currVertexBufferSize;

            GetCubeData(cmd.min, cmd.max, vertexBuffer, indexBuffer);

            for (int j = numVerticesBefore; j < currVertexBufferSize; j++) {
                colourBuffer[j] = colour;
            }
        }
        assert(vertexBufferSize==currVertexBufferSize);
        assert(vertexBufferSize==vertexBuffer.length);
        assert(indexBufferSize==currIndexBufferSize);
        assert(indexBufferSize==indexBuffer.length);

        DebugDrawBuffer buffer = new DebugDrawBuffer();
        buffer.setVertexBuffer(vertexBuffer);
        buffer.setIndexBuffer(indexBuffer);
        buffer.setColourBuffer(colourBuffer);
        buffer.setNumIndices(indexBufferSize);
        buffer.setNumVertices(vertexBufferSize);
        return buffer;
    }

    public void GetCubeData(Vec3i min, Vec3i max, Vec4f[] vertexDataBuffer, int[] indexDataBuffer) {
        vertexDataBuffer[currVertexBufferSize] = new Vec4f(min.x, min.y, min.z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 1] = new Vec4f(min.x, max.y, min.z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 2] = new Vec4f(max.x, max.y, min.z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 3] = new Vec4f(max.x, min.y, min.z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 4] = new Vec4f(min.x, min.y, max.z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 5] = new Vec4f(min.x, max.y, max.z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 6] = new Vec4f(max.x, max.y, max.z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 7] = new Vec4f(max.x, min.y, max.z, 0.f);

        int[] indices = {
                0, 1, 2, 0, 2, 3,
                4, 1, 0, 4, 5, 1,
                0, 3, 4, 3, 7, 4,
                3, 2, 6, 6, 7, 3,
                2, 1, 5, 5, 6, 2,
                4, 6, 5, 4, 7, 6,
        };

        int[] edgeElementArray = {
                    0,1,  1,5,  5,4,  4,0,    // edges of the top face
                    7,3,  3,2,  2,6,  6,7,    // edges of the bottom face
                    1,2,  0,3,  4,7,  5,6
        }; // edges connecting top face to bottom face

        for (int i = 0; i < NUM_INDICATES; i++) {
            indexDataBuffer[i + currIndexBufferSize] = currVertexBufferSize + edgeElementArray[i];
        }
        currVertexBufferSize += NUM_VERTICES;
        currIndexBufferSize += NUM_INDICATES;
    }
}
