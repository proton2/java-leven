package dc.impl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.utils.BufferUtil;
import core.utils.Constants;
import dc.*;
import dc.entities.MeshBuffer;
import dc.entities.MeshVertex;
import dc.solver.GlslSvd;
import dc.solver.LevenQefSolver;
import dc.solver.QEFData;
import dc.utils.VoxelHelperUtils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static dc.utils.SimplexNoise.getNoise;

/*
    Simple Linear octree dual contouring implementation, in series steps to calculate leafs data
    Holes in seams are fixed.
 */

public class SimpleLinearOctreeImpl extends AbstractDualContouring implements VoxelOctree {

    public SimpleLinearOctreeImpl(MeshGenerationContext meshGenerationContext) {
        super(meshGenerationContext);
    }

    @Override
    public boolean createLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                        List<OctreeNode> seamNodes, MeshBuffer buffer, GPUDensityField field) {

        // usyal in serial calculating leaf nodes data. More slowly.
        return createLeafVoxelNodesTraditional(chunkSize, chunkMin, meshGen.getVoxelsPerChunk(), seamNodes, buffer, field);
    }

    public boolean createLeafVoxelNodesTraditional(int chunkSize, Vec3i chunkMin, int voxelsPerChunk,
                                                   List<OctreeNode> seamNodes, MeshBuffer buffer, GPUDensityField field)
    {
        ArrayList<LinearLeafHolder> listHolder = new ArrayList<>();
        Map<Integer, Integer> octreeNodes = new HashMap<>();

        int current = 0;
        for (int i = 0; i < voxelsPerChunk*voxelsPerChunk*voxelsPerChunk; i++) {
            int indexShift = VoxelHelperUtils.log2(voxelsPerChunk); // max octree depth
            int x = (i >> (indexShift * 0)) & voxelsPerChunk - 1;
            int y = (i >> (indexShift * 1)) & voxelsPerChunk - 1;
            int z = (i >> (indexShift * 2)) & voxelsPerChunk - 1;
            Vec3i pos = new Vec3i(x, y, z);
            LinearLeafHolder linearLeafHolder = constructLeaf(pos, chunkMin, chunkSize);
            if(linearLeafHolder!=null){
                listHolder.add(linearLeafHolder);
                octreeNodes.put(linearLeafHolder.encodedVoxelPosition, current);
                ++current;
            }
        }
        if(listHolder.size()==0){
            return false;
        }

        int[] d_nodeCodes = listHolder.stream().mapToInt(e->e.encodedVoxelPosition).toArray();
        int[] d_nodeMaterials = listHolder.stream().mapToInt(e->e.materialIndex).toArray();
        Vec4f[] d_vertexPositions = listHolder.stream().map(e->e.solvedPosition).toArray(Vec4f[]::new);
        Vec4f[] d_vertexNormals = listHolder.stream().map(e->e.averageNormal).toArray(Vec4f[]::new);

        processDc(chunkSize, chunkMin, voxelsPerChunk, seamNodes, buffer, octreeNodes,
                d_nodeCodes, d_nodeMaterials, d_vertexPositions, d_vertexNormals);
        return true;
    }

    private void processDc(int chunkSize, Vec3i chunkMin, int voxelsPerChunk, List<OctreeNode> seamNodes,
                           MeshBuffer buffer, Map<Integer, Integer> octreeNodes, int[] d_nodeCodes, int[] d_nodeMaterials,
                           Vec4f[] d_vertexPositions, Vec4f[] d_vertexNormals) {
        int numVertices = octreeNodes.size();
        int indexBufferSize = numVertices * 6 * 3;
        int[] d_indexBuffer = new int[indexBufferSize];
        int trianglesValidSize = numVertices * 3;
        int[] d_trianglesValid = new int[trianglesValidSize];

        int trianglesValidCount = generateMesh(octreeNodes, d_nodeCodes, d_nodeMaterials,
                d_indexBuffer, d_trianglesValid);

        int numTriangles = trianglesValidCount * 2;
        int[] d_compactIndexBuffer = new int[numTriangles * 3];
        compactMeshTriangles(d_trianglesValid, d_indexBuffer,
                d_compactIndexBuffer);

        MeshVertex[] d_vertexBuffer = new MeshVertex[numVertices];
        GenerateMeshVertexBuffer(d_vertexPositions, d_vertexNormals, d_nodeMaterials, Constants.Red, d_vertexBuffer);

        buffer.setVertices(BufferUtil.createDcFlippedBufferAOS(d_vertexBuffer));
        buffer.setNumVertices(numVertices);
        buffer.setIndicates(BufferUtil.createFlippedBuffer(d_compactIndexBuffer));
        buffer.setNumIndicates(d_compactIndexBuffer.length);

        boolean[] isSeamNode = new boolean[numVertices];
        // ToDo return seamNodes which size have seamSize from method
        int seamSize = findSeamNodes(d_nodeCodes, isSeamNode, 0, d_nodeCodes.length);

        extractNodeInfo(isSeamNode, Constants.Yellow,
                chunkSize / voxelsPerChunk, chunkMin, 0, numVertices,
                d_nodeCodes, d_nodeMaterials, d_vertexPositions, d_vertexNormals, seamNodes);
    }

    private static class LinearLeafHolder{
        int materialIndex;
        int voxelEdgeInfo;
        int encodedVoxelPosition;
        Vec4f solvedPosition;
        Vec4f averageNormal;
    }

    private LinearLeafHolder constructLeaf(Vec3i pos, Vec3i chunkMin, int chunkSize) {
        int leafSize = (chunkSize / meshGen.getVoxelsPerChunk());
        Vec3i leafMin = pos.mul(leafSize).add(chunkMin);
        int[] cornerMaterials = new int[8];
        int corners = 0;
        for (int i = 0; i < 8; i++) {
            Vec3f cornerPos = leafMin.add(CHILD_MIN_OFFSETS[i].mul(leafSize)).toVec3f();
            float density = getNoise(cornerPos);
            int material = density < 0.f ? meshGen.MATERIAL_SOLID : meshGen.MATERIAL_AIR;
            cornerMaterials[i] = material;
            corners |= (material << i);
        }
        if (corners == 0 || corners == 255) {
            // to avoid holes in seams between chunks with different resolution we creating some other nodes only in seams
            //https://www.reddit.com/r/VoxelGameDev/comments/6kn8ph/dual_contouring_seam_stitching_problem/
            Vec4f nodePos = tryToCreateBoundSeamPseudoNode(leafMin, leafSize, pos, corners, meshGen.leafSizeScale);
            if(nodePos==null){
                return null;
            } else {
                LinearLeafHolder leafHolder = new LinearLeafHolder();
                leafHolder.solvedPosition = nodePos;
                int materialIndex = findDominantMaterial(cornerMaterials);
                leafHolder.materialIndex = (materialIndex << 8) | corners;
                leafHolder.encodedVoxelPosition = LinearOctreeTest.codeForPosition(pos, meshGen.MAX_OCTREE_DEPTH);
                leafHolder.voxelEdgeInfo = corners;//edgeList;
                leafHolder.averageNormal = CalculateSurfaceNormal(nodePos);
                return leafHolder;
            }
        }
        int edgeList = 0;

        // otherwise the voxel contains the surface, so find the edge intersections
        int MAX_CROSSINGS = 6;
        int edgeCount = 0;
        Vec4f averageNormal = new Vec4f();
        Vec4f[] edgePositions = new Vec4f[12];
        Vec4f[] edgeNormals = new Vec4f[12];
        QEFData qef = new QEFData(new LevenQefSolver());
        for (int i = 0; i < 12 && edgeCount < MAX_CROSSINGS; i++) {
            int c1 = edgevmap[i][0];
            int c2 = edgevmap[i][1];
            int m1 = (corners >> c1) & 1;
            int m2 = (corners >> c2) & 1;
            int signChange = m1 != m2 ? 1 : 0;
            edgeList |= (signChange << i);
            if ((m1 == meshGen.MATERIAL_AIR && m2 == meshGen.MATERIAL_AIR) || (m1 == meshGen.MATERIAL_SOLID && m2 == meshGen.MATERIAL_SOLID)) {
                continue; // no zero crossing on this edge
            }
            Vec3f p1 = leafMin.add(CHILD_MIN_OFFSETS[c1].mul(leafSize)).toVec3f();
            Vec3f p2 = leafMin.add(CHILD_MIN_OFFSETS[c2].mul(leafSize)).toVec3f();
            Vec4f p = ApproximateZeroCrossingPosition(p1, p2);
            Vec4f n = CalculateSurfaceNormal(p);
            edgePositions[edgeCount] = p;
            edgeNormals[edgeCount] = n;
            averageNormal = averageNormal.add(n);
            edgeCount++;
        }
        qef.qef_create_from_points(edgePositions, edgeNormals, edgeCount);

        LinearLeafHolder leafHolder = new LinearLeafHolder();
        leafHolder.solvedPosition = qef.solve();
        int materialIndex = findDominantMaterial(cornerMaterials);
        leafHolder.materialIndex = (materialIndex << 8) | corners;
        leafHolder.encodedVoxelPosition = LinearOctreeTest.codeForPosition(pos, meshGen.MAX_OCTREE_DEPTH);
        leafHolder.voxelEdgeInfo = edgeList;
        leafHolder.averageNormal = averageNormal.div((float)edgeCount).normalize();
        return leafHolder;
    }

    private int findSeamNodes(int[] nodeCodes, boolean[] isSeamNode, int from, int to) {
        int res = 0;
        for (int index = from; index < to; index++) {
            int code = nodeCodes[index];
            Vec3i position = LinearOctreeTest.positionForCode(code);
            boolean xSeam = position.x == 0 || position.x == (meshGen.getVoxelsPerChunk() - 1);
            boolean ySeam = position.y == 0 || position.y == (meshGen.getVoxelsPerChunk() - 1);
            boolean zSeam = position.z == 0 || position.z == (meshGen.getVoxelsPerChunk() - 1);
            boolean isSeam = xSeam | ySeam | zSeam;
            if (isSeam) {
                ++res;
            }
            isSeamNode[index] = xSeam | ySeam | zSeam;
        }
        return res;
    }

    private Vec3i[][] EDGE_NODE_OFFSETS = {
            {new Vec3i(0, 0, 0), new Vec3i(0, 0, 1), new Vec3i(0, 1, 0), new Vec3i(0, 1, 1)},
            {new Vec3i(0, 0, 0), new Vec3i(1, 0, 0), new Vec3i(0, 0, 1), new Vec3i(1, 0, 1)},
            {new Vec3i(0, 0, 0), new Vec3i(0, 1, 0), new Vec3i(1, 0, 0), new Vec3i(1, 1, 0)},
    };

    private int generateMesh(Map<Integer, Integer> nodes, int[] octreeNodeCodes, int[] octreeMaterials,
                             int[] meshIndexBuffer, int[] trianglesValid) {
        int size = 0;
        for (int index = 0; index < octreeNodeCodes.length; index++) {
            int code = octreeNodeCodes[index];
            int triIndex = index * 3;

            Vec3i offset = LinearOctreeTest.positionForCode(code);
            int[] pos = {offset.x, offset.y, offset.z};
            Integer[] nodeIndices = {null, null, null, null};

            for (int axis = 0; axis < 3; axis++) {
                trianglesValid[triIndex + axis] = 0;

                // need to check that the position generated when the offsets are added won't exceed
                // the chunk bounds -- if this happens rather than failing the octree lookup
                // will actually wrap around to 0 again causing weird polys to be generated

                int a = pos[(axis + 1) % 3];
                int b = pos[(axis + 2) % 3];
                boolean isEdgeVoxel = a == (meshGen.getVoxelsPerChunk() - 1) || b == (meshGen.getVoxelsPerChunk() - 1);
                if (isEdgeVoxel) {
                    continue;
                }

                nodeIndices[0] = index;
                for (int n = 1; n < 4; n++) {
                    Vec3i p = offset.add(EDGE_NODE_OFFSETS[axis][n]);
                    int c = LinearOctreeTest.codeForPosition(p, meshGen.MAX_OCTREE_DEPTH);
                    nodeIndices[n] = nodes.get(c);
                }

                if (nodeIndices[1] != null && nodeIndices[2] != null && nodeIndices[3] != null) {
                    int bufferOffset = (triIndex * 6) + (axis * 6);
                    int trisEmitted = processEdge(nodeIndices, octreeMaterials[index], axis, meshIndexBuffer, bufferOffset);
                    size += trisEmitted;
                    trianglesValid[triIndex + axis] = trisEmitted;
                }
            }
        }
        return size;
    }

    private int processEdge(Integer[] nodeIndices, int nodeMaterial, int axis, int[] indexBuffer, int bufferOffset) {
        int edge = (axis * 4) + 3;
        int c1 = edgevmap[edge][0];
        int c2 = edgevmap[edge][1];

        int corners = nodeMaterial & 0xff;
        int m1 = (corners >> c1) & 1;
        int m2 = (corners >> c2) & 1;

        boolean signChange = (m1 == meshGen.MATERIAL_AIR && m2 != meshGen.MATERIAL_AIR) || (m1 != meshGen.MATERIAL_AIR && m2 == meshGen.MATERIAL_AIR);
        if (!signChange) {
            return 0;
        }

        // flip the winding depending on which end of the edge is outside the volume
        int flip = m1 != 0 ? 1 : 0;
        int[][] indices = {
                // different winding orders depending on the sign change direction
                {0, 1, 3, 0, 3, 2},
                {0, 3, 1, 0, 2, 3},
        };

        indexBuffer[bufferOffset + 0] = nodeIndices[indices[flip][0]];
        indexBuffer[bufferOffset + 1] = nodeIndices[indices[flip][1]];
        indexBuffer[bufferOffset + 2] = nodeIndices[indices[flip][2]];
        indexBuffer[bufferOffset + 3] = nodeIndices[indices[flip][3]];
        indexBuffer[bufferOffset + 4] = nodeIndices[indices[flip][4]];
        indexBuffer[bufferOffset + 5] = nodeIndices[indices[flip][5]];
        return 1;
    }

    private void compactMeshTriangles(int[] trianglesValid, int[] meshIndexBuffer, int[] compactMeshIndexBuffer) {
        int current = 0;
        for (int index = 0; index < trianglesValid.length; index++) {
            if (trianglesValid[index] == 1) {
                int scanOffset = current * 6;
                int bufferOffset = (index * 6);
                System.arraycopy(meshIndexBuffer, bufferOffset, compactMeshIndexBuffer, scanOffset, 6);
//                for (int i = 0; i < 6; i++) {
//                    compactMeshIndexBuffer[scanOffset + i] = meshIndexBuffer[bufferOffset + i];
//                }
                ++current;
            }
        }
    }

    private void GenerateMeshVertexBuffer(Vec4f[] vertexPositions, Vec4f[] vertexNormals, int[] nodeMaterials, Vec3f colour,
                                          MeshVertex[] meshVertexBuffer) {
        for (int index = 0; index < vertexPositions.length; index++) {
            int material = nodeMaterials[index];
            meshVertexBuffer[index] = new MeshVertex();
            meshVertexBuffer[index].setPos(vertexPositions[index].getVec3f());
            meshVertexBuffer[index].setNormal(vertexNormals[index].getVec3f());
            meshVertexBuffer[index].setColor(colour); //colour = new Vec4f(colour.X, colour.Y, colour.Z, (float) (material >> 8));
        }
    }

    private void extractNodeInfo(boolean[] isSeamNode, Vec3f color,
                                     int leafSize, Vec3i chunkMin, int from, int to,
                                     int[] octreeCodes, int[] octreeMaterials, Vec4f[] octreePositions, Vec4f[] octreeNormals,
                                     List<OctreeNode> seamNodes) {
        for (int index = from; index < to; index++) {
            if (isSeamNode==null || isSeamNode[index]) {
                Vec3i min = LinearOctreeTest.positionForCode(octreeCodes[index]).mul(leafSize).add(chunkMin);
                PointerBasedOctreeNode node = new PointerBasedOctreeNode(min, leafSize, OctreeNodeType.Node_Leaf);
                OctreeDrawInfo drawInfo = new OctreeDrawInfo();
                drawInfo.position = octreePositions[index].getVec3f();
                drawInfo.color = color;
                drawInfo.averageNormal = octreeNormals[index].getVec3f();
                node.corners = octreeMaterials[index];
                node.drawInfo = drawInfo;
                seamNodes.add(node);
            }
        }
    }
}
