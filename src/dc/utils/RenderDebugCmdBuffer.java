package dc.utils;

import core.buffers.DebugMeshVBO;
import core.math.Vec2f;
import core.math.Vec3f;
import core.math.Vec4f;
import core.utils.Constants;
import dc.entities.DebugDrawBuffer;


import java.util.ArrayList;

import static dc.utils.RenderShape.*;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

public class RenderDebugCmdBuffer {
    int SPHERE_SUBDIVISIONS = 10;			// how many long/lat subdivisions
    static class RenderDebugCmd {
        RenderShape shape;
        Vec3f rgb;
        float alpha;

        RenderDebugCubeInfo    cube;
        RenderDebugSphereInfo  sphere;
        RenderDebugLineInfo    line;
        RenderDebugCubeArrayCoordsInfo cubeArrayCoordsInfo;
    }

    static class RenderDebugCubeInfo {
        Vec3f    min;
        Vec3f    max;
    };

    static class RenderDebugCubeArrayCoordsInfo {
        Vec3f[] coords;
    };

    static class RenderDebugSphereInfo {
        Vec3f    origin;
        float	 radius;
    };

    static class RenderDebugLineInfo {
        Vec3f    start;
        Vec3f    end;
    };

    ArrayList<RenderDebugCmd> cmds = new ArrayList<>();
    int vertexBufferSize = 0;
    int indexBufferSize = 0;
    int currVertexBufferSize, currIndexBufferSize;

    public RenderDebugCmd addWireCube(Vec3f rgb, float alpha, Vec3f min, int size) {
        RenderDebugCmd cmd = new RenderDebugCmd();
        cmd.shape = RenderShape_WireCube;
        cmd.alpha = alpha;
        cmd.rgb = rgb;
        cmd.cube = new RenderDebugCubeInfo();
        cmd.cube.min = min;
        cmd.cube.max = new Vec3f(min.X + size, min.Y + size, min.Z + size);
        cmds.add(cmd);
        return cmd;
    }

    public RenderDebugCmd addWireCubeArrayCoords(Vec3f rgb, float alpha, Vec3f[] coords) {
        RenderDebugCmd cmd = new RenderDebugCmd();
        cmd.shape = RenderShape_CubeArrayCoords;
        cmd.alpha = alpha;
        cmd.rgb = rgb;
        cmd.cubeArrayCoordsInfo = new RenderDebugCubeArrayCoordsInfo();
        cmd.cubeArrayCoordsInfo.coords = coords;
        cmds.add(cmd);
        return cmd;
    }

    public void addCube(Vec3f rgb, float alpha, Vec3f min, int size) {
        RenderDebugCmd cmd = new RenderDebugCmd();
        cmd.shape = RenderShape_Cube;
        cmd.rgb = rgb;
        cmd.alpha = alpha;
        cmd.cube = new RenderDebugCubeInfo();
        cmd.cube.min = min;
        cmd.cube.max = new Vec3f(min.X + size, min.Y + size, min.Z + size);
        cmds.add(cmd);
    }

    public void addSphere(Vec3f rgb, float alpha, Vec3f origin, float radius) {
        RenderDebugCmd cmd = new RenderDebugCmd();
        cmd.shape = RenderShape_Sphere;
        cmd.rgb = rgb;
        cmd.alpha = alpha;
        cmd.sphere = new RenderDebugSphereInfo();
        cmd.sphere.origin = origin;
        cmd.sphere.radius = radius;
        cmds.add(cmd);
    }

    public void addLine(Vec3f rgb, float alpha, Vec3f start, Vec3f end) {
        RenderDebugCmd cmd = new RenderDebugCmd();
        cmd.shape = RenderShape_Line;
        cmd.rgb = rgb;
        cmd.alpha = alpha;
        cmd.line = new RenderDebugLineInfo();
        cmd.line.start = start;
        cmd.line.end = end;
        cmds.add(cmd);
    }

//    public DebugDrawBuffer UpdateDebugDrawBuffer() {
//        for (int i = cmds.size() - 1; i >= 0; i--) {
//            int vertices = 8;
//            int indices = 24;
//
//            //cmds.get(i).shape
//            vertexBufferSize += vertices;
//            indexBufferSize += indices;
//        }
//        Vec4f[] vertexBuffer = new Vec4f[vertexBufferSize];
//        Vec4f[] colourBuffer = new Vec4f[vertexBufferSize];
//        int[] indexBuffer = new int[indexBufferSize];
//
//        for (int i = cmds.size() - 1; i >= 0; i--) {
//            RenderDebugCmd cmd = cmds.get(i);
//            Vec4f colour = new Vec4f(cmd.rgb, cmd.alpha);
//            int numVerticesBefore = currVertexBufferSize;
//
//            GetWireCubeData(cmd.cube.min, cmd.cube.max, vertexBuffer, indexBuffer);
//
//            for (int j = numVerticesBefore; j < currVertexBufferSize; j++) {
//                colourBuffer[j] = colour;
//            }
//        }
//        assert(vertexBufferSize==currVertexBufferSize);
//        assert(vertexBufferSize==vertexBuffer.length);
//        assert(indexBufferSize==currIndexBufferSize);
//        assert(indexBufferSize==indexBuffer.length);
//
//        DebugDrawBuffer buffer = new DebugDrawBuffer();
//        buffer.setVertexBuffer(vertexBuffer);
//        buffer.setIndexBuffer(indexBuffer);
//        buffer.setColourBuffer(colourBuffer);
//        buffer.setNumIndices(indexBufferSize);
//        buffer.setNumVertices(vertexBufferSize);
//        return buffer;
//    }

    public DebugDrawBuffer UpdateDebugDrawBuffer() {
        for (int i = cmds.size() - 1; i >= 0; i--) {
            int vertices = 0;
            int indices = 0;

            int[] res;
            switch (cmds.get(i).shape) {
                case RenderShape_Cube:
                    res = GetCubeDataSizes();
                    vertices = res[0];
                    indices = res[1];
                    break;

                case RenderShape_WireCube:
                    res = GetWireCubeDataSizes();
                    vertices = res[0];
                    indices = res[1];
                    break;

                case RenderShape_CubeArrayCoords:
                    res = GetWireCubeDataSizes();
                    vertices = res[0];
                    indices = res[1];
                    break;

                case RenderShape_Sphere:
                    res = GetSphereDataSizes();
                    vertices = res[0];
                    indices = res[1];
                    break;

                case RenderShape_Line:
                    res = GetLineDataSizes();
                    vertices = res[0];
                    indices = res[1];
                    break;
            }
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

            switch (cmd.shape) {
                case RenderShape_Cube: {
                    GetCubeData(cmd.cube.min, cmd.cube.max, vertexBuffer, indexBuffer);
                    break;
                }
                case RenderShape_WireCube: {
                    GetWireCubeData(cmd.cube.min, cmd.cube.max, vertexBuffer, indexBuffer);
                    break;
                }
                case RenderShape_CubeArrayCoords: {
                    GetWireCubeDataArrayCoords(cmd.cubeArrayCoordsInfo.coords, vertexBuffer, indexBuffer);
                    break;
                }
                case RenderShape_Sphere: {
                    GetSphereData(cmd.sphere.origin, cmd.sphere.radius, vertexBuffer, indexBuffer);
                    break;
                }
                case RenderShape_Line: {
                    GetLineData(cmd.line.start, cmd.line.end, vertexBuffer, indexBuffer);
                    break;
                }
            }

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

    public void GetWireCubeData(Vec3f min, Vec3f max, Vec4f[] vertexDataBuffer, int[] indexDataBuffer) {
        vertexDataBuffer[currVertexBufferSize + 0] = new Vec4f(min.X, min.Y, min.Z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 1] = new Vec4f(min.X, max.Y, min.Z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 2] = new Vec4f(max.X, max.Y, min.Z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 3] = new Vec4f(max.X, min.Y, min.Z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 4] = new Vec4f(min.X, min.Y, max.Z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 5] = new Vec4f(min.X, max.Y, max.Z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 6] = new Vec4f(max.X, max.Y, max.Z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 7] = new Vec4f(max.X, min.Y, max.Z, 0.f);

        int[] edgeElementArray = {
                    0,1,  1,5,  5,4,  4,0,    // edges of the top face
                    7,3,  3,2,  2,6,  6,7,    // edges of the bottom face
                    1,2,  0,3,  4,7,  5,6
        }; // edges connecting top face to bottom face

        for (int i = 0; i < 24; i++) {
            indexDataBuffer[i + currIndexBufferSize] = currVertexBufferSize + edgeElementArray[i];
        }
        currVertexBufferSize += 8;
        currIndexBufferSize += 24;
    }

    public void GetWireCubeDataArrayCoords(Vec3f[] corners, Vec4f[] vertexDataBuffer, int[] indexDataBuffer) {
        vertexDataBuffer[currVertexBufferSize + 0] = new Vec4f(corners[0]);
        vertexDataBuffer[currVertexBufferSize + 1] = new Vec4f(corners[1]);
        vertexDataBuffer[currVertexBufferSize + 2] = new Vec4f(corners[2]);
        vertexDataBuffer[currVertexBufferSize + 3] = new Vec4f(corners[3]);
        vertexDataBuffer[currVertexBufferSize + 4] = new Vec4f(corners[4]);
        vertexDataBuffer[currVertexBufferSize + 5] = new Vec4f(corners[5]);
        vertexDataBuffer[currVertexBufferSize + 6] = new Vec4f(corners[6]);
        vertexDataBuffer[currVertexBufferSize + 7] = new Vec4f(corners[7]);

        int[] edgeElementArray = {
                0,1,  1,5,  5,4,  4,0,    // edges of the top face
                7,3,  3,2,  2,6,  6,7,    // edges of the bottom face
                1,2,  0,3,  4,7,  5,6
        }; // edges connecting top face to bottom face

        for (int i = 0; i < 24; i++) {
            indexDataBuffer[i + currIndexBufferSize] = currVertexBufferSize + edgeElementArray[i];
        }
        currVertexBufferSize += 8;
        currIndexBufferSize += 24;
    }

    void GetLineData(Vec3f start, Vec3f end, Vec4f[] vertexDataBuffer, int[] indexDataBuffer) {
        vertexDataBuffer[0] = new Vec4f(start, 0.f);
        vertexDataBuffer[1] = new Vec4f(end, 0.f);

        indexDataBuffer[0] = vertexBufferSize + 0;
        indexDataBuffer[1] = vertexBufferSize + 1;

        currVertexBufferSize += 2;
        currIndexBufferSize += 2;
    }

    void GetCubeData(Vec3f min, Vec3f max, Vec4f[] vertexDataBuffer, int[] indexDataBuffer) {
        vertexDataBuffer[currVertexBufferSize + 0] = new Vec4f(min.X, min.Y, min.Z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 1] = new Vec4f(min.X, max.Y, min.Z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 2] = new Vec4f(max.X, max.Y, min.Z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 3] = new Vec4f(max.X, min.Y, min.Z, 0.f);

        vertexDataBuffer[currVertexBufferSize + 4] = new Vec4f(min.X, min.Y, max.Z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 5] = new Vec4f(min.X, max.Y, max.Z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 6] = new Vec4f(max.X, max.Y, max.Z, 0.f);
        vertexDataBuffer[currVertexBufferSize + 7] = new Vec4f(max.X, min.Y, max.Z, 0.f);

        int[] edgeElementArray = {
                    0, 1, 2, 0, 2, 3,
                    4, 1, 0, 4, 5, 1,
                    0, 3, 4, 3, 7, 4,
                    3, 2, 6, 6, 7, 3,
                    2, 1, 5, 5, 6, 2,
                    4, 6, 5, 4, 7, 6,
        };

        for (int i = 0; i < 36; i++) {
		    indexDataBuffer[i + currIndexBufferSize] = currVertexBufferSize + edgeElementArray[i];
        }

	    currVertexBufferSize += 8;
        currIndexBufferSize += 36;
    }

    private int[] GetLineDataSizes() {
        return new int[]{2,2};
    }

    private int[] GetCubeDataSizes() {
        return new int[]{8,36};
    }

    private int[] GetWireCubeDataSizes() {
        return new int[]{8,24};
    }

    private int[] GetSphereDataSizes() {
	    int numVertices = 0;
	    int numIndices = 0;
        for (int i = 0; i < SPHERE_SUBDIVISIONS; i++) {
            for (int j = 0; j < SPHERE_SUBDIVISIONS; j++) {
                numVertices += 4;
                numIndices += 6;
            }
        }
        return new int[]{numVertices, numIndices};
    }

    void CalculateNormals(int[] indices, int numIndices, Vec4f[] vertexData, int numVertices, Vec4f[] normalData) {
        for (int i = 0; i < numVertices; i++) {
            normalData[i] = new Vec4f(0,0,0,0);
        }

        for (int i = 0; i < numIndices; i += 3) {
		    Vec3f p0 = vertexData[indices[i + 0]].getVec3f();
		    Vec3f p1 = vertexData[indices[i + 1]].getVec3f();
            Vec3f p2 = vertexData[indices[i + 2]].getVec3f();

		    Vec3f normal = p1.sub(p0).cross(p2.sub(p0)).normalize();

            normalData[indices[i + 0]] = normalData[indices[i + 0]].add(new Vec4f(normal, 0.f));
            normalData[indices[i + 1]] = normalData[indices[i + 1]].add(new Vec4f(normal, 0.f));
            normalData[indices[i + 2]] = normalData[indices[i + 2]].add(new Vec4f(normal, 0.f));
        }

        for (int i = 0; i < numVertices; i++) {
            normalData[i].normalize();
        }
    }

    private void GetSphereData(Vec3f origin, float radius, Vec4f[] vertexDataBuffer, int[] indexDataBuffer) {
        int indices[] = {0, 2, 1, 2, 3, 1};
        Vec4f o = new Vec4f(origin, 0.f);
        Vec4f diameter = new Vec4f(radius, radius, radius, 0.f).mul(2);

        Vec4f[] vertexPositions = new Vec4f[SPHERE_SUBDIVISIONS * SPHERE_SUBDIVISIONS];

        for (int n = 0; n < SPHERE_SUBDIVISIONS; n++) {
            float theta = (float) ((Math.PI * n) / (float) SPHERE_SUBDIVISIONS);
            for (int m = 0; m < SPHERE_SUBDIVISIONS; m++) {
                int offset = (n * SPHERE_SUBDIVISIONS) + m;
                float phi = (float) ((2 * Math.PI * m) / (float) SPHERE_SUBDIVISIONS);
                float x = (float) (sin(theta) * cos(phi));
                float y = (float) (sin(theta) * sin(phi));
                float z = (float) cos(theta);
                vertexPositions[offset] = new Vec4f(x, y, z, 0.f);
            }
        }

//        int vertexOffset = 0;
//        int indexOffset = 0;
        for (int n = 0; n < SPHERE_SUBDIVISIONS; n++) {
            for (int m = 0; m < SPHERE_SUBDIVISIONS; m++) {
                int mPlusOne = (m + 1) % SPHERE_SUBDIVISIONS;
                vertexDataBuffer[currVertexBufferSize + 0] = vertexPositions[(n * SPHERE_SUBDIVISIONS) + m];
                vertexDataBuffer[currVertexBufferSize + 1] = vertexPositions[(n * SPHERE_SUBDIVISIONS) + mPlusOne];
                vertexDataBuffer[currVertexBufferSize + 2] = n < (SPHERE_SUBDIVISIONS - 1) ? vertexPositions[((n + 1) * SPHERE_SUBDIVISIONS) + m] : new Vec4f(0.f, 0.f, -1.f, 0.f);
                vertexDataBuffer[currVertexBufferSize + 3] = n < (SPHERE_SUBDIVISIONS - 1) ? vertexPositions[((n + 1) * SPHERE_SUBDIVISIONS) + mPlusOne] : new Vec4f(0.f, 0.f, -1.f, 0.f);

                for (int i = 0; i < 4; i++) {
                    vertexDataBuffer[currVertexBufferSize + i] = (vertexDataBuffer[currVertexBufferSize + i].add(diameter)).add(o);
                }
                for (int i = 0; i < 6; i++) {
                    indexDataBuffer[currIndexBufferSize + i] = vertexBufferSize + indices[i];
                }
                currVertexBufferSize += 4;
                currIndexBufferSize += 6;
//                vertexOffset += 4;
//                indexOffset += 6;
            }
        }
        int t=3;
    }

    private void getSingleWireCubeData(Vec3f min, Vec3f max, Vec4f[] vertexDataBuffer, int[] indexDataBuffer) {
        vertexDataBuffer[0] = new Vec4f(min.X, min.Y, min.Z, 0.f);
        vertexDataBuffer[1] = new Vec4f(min.X, max.Y, min.Z, 0.f);
        vertexDataBuffer[2] = new Vec4f(max.X, max.Y, min.Z, 0.f);
        vertexDataBuffer[3] = new Vec4f(max.X, min.Y, min.Z, 0.f);
        vertexDataBuffer[4] = new Vec4f(min.X, min.Y, max.Z, 0.f);
        vertexDataBuffer[5] = new Vec4f(min.X, max.Y, max.Z, 0.f);
        vertexDataBuffer[6] = new Vec4f(max.X, max.Y, max.Z, 0.f);
        vertexDataBuffer[7] = new Vec4f(max.X, min.Y, max.Z, 0.f);

        int[] edgeElementArray = {
                0,1,  1,5,  5,4,  4,0,    // edges of the top face
                7,3,  3,2,  2,6,  6,7,    // edges of the bottom face
                1,2,  0,3,  4,7,  5,6
        }; // edges connecting top face to bottom face

        System.arraycopy(edgeElementArray, 0, indexDataBuffer, 0, 24);
    }

    public DebugMeshVBO createCube(){
        RenderDebugCmd cubeCmd = addWireCube(Constants.Yellow, 0.2f, new Vec3f(), 10);
        int[] res = GetWireCubeDataSizes();
        int vertices = res[0];
        int indices = res[1];
        Vec4f[] vertexBuffer = new Vec4f[vertices];
        Vec4f[] colourBuffer = new Vec4f[vertices];
        int[] indexBuffer = new int[indices];

        Vec4f colour = new Vec4f(cubeCmd.rgb, cubeCmd.alpha);
        for (int i=0; i<vertices; i++){
            colourBuffer[i] = colour;
        }

        getSingleWireCubeData(cubeCmd.cube.min, cubeCmd.cube.max, vertexBuffer, indexBuffer);

        DebugDrawBuffer buffer = new DebugDrawBuffer();
        buffer.setVertexBuffer(vertexBuffer);
        buffer.setIndexBuffer(indexBuffer);
        buffer.setColourBuffer(colourBuffer);
        buffer.setNumIndices(indices);
        buffer.setNumVertices(vertices);

        DebugMeshVBO camRayBuff = new DebugMeshVBO();
        camRayBuff.addData(buffer);
        return camRayBuff;
    }
}
