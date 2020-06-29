package dc;

import core.math.Vec3f;
import core.math.Vec3i;
import core.utils.BufferUtil;
import core.utils.Constants;
import dc.entities.MeshBuffer;
import dc.entities.MeshVertex;
import dc.svd.QefSolver;
import dc.utils.Density;
import dc.utils.VoxelHelperUtils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.*;

import static dc.ChunkOctree.*;
import static dc.LinearOctreeImpl.fieldSize;
import static dc.LinearOctreeTest.MAX_OCTREE_DEPTH;

/*
    Simple Linear octree dual contouring implementation, in series steps to calculate leafs data
 */

public class SimpleLinearOctreeImpl extends AbstractDualContouring implements VoxelOctree {

    @Override
    public boolean createLeafVoxelNodes(int chunkSize, Vec3i chunkMin, int voxelsPerChunk,
                                        int clipmapLeafSize, int leafSizeScale,
                                        float[] densityField,
                                        List<PointerBasedOctreeNode> seamNodes, MeshBuffer buffer) {

        // usyal in serial calculating leaf nodes data. More slowly.
        return createLeafVoxelNodesTraditional(chunkSize, chunkMin, voxelsPerChunk, clipmapLeafSize, leafSizeScale, densityField, seamNodes, buffer);

        // experimental transitional branch. It worked successfull but seams have some holes. More faster
        //return createLeafVoxelNodesExperimental(chunkSize, chunkMin, voxelsPerChunk, densityField, seamNodes, buffer);
    }

    public boolean createLeafVoxelNodesTraditional(int chunkSize, Vec3i chunkMin, int voxelsPerChunk,
                                                   int clipmapLeafSize, int leafSizeScale,
                                                   float[] densityField,
                                                   List<PointerBasedOctreeNode> seamNodes, MeshBuffer buffer)
    {
        ArrayList<LinearLeafHolder> listHolder = new ArrayList<>();
        Map<Integer, Integer> octreeNodes = new HashMap<>();

        int current = 0;
        for (int i = 0; i < voxelsPerChunk*voxelsPerChunk*voxelsPerChunk; i++) {
            int indexShift = log2(voxelsPerChunk); // max octree depth
            int x = (i >> (indexShift * 0)) & voxelsPerChunk - 1;
            int y = (i >> (indexShift * 1)) & voxelsPerChunk - 1;
            int z = (i >> (indexShift * 2)) & voxelsPerChunk - 1;
            Vec3i pos = new Vec3i(x, y, z);
            LinearLeafHolder linearLeafHolder = constructLeaf(pos, chunkMin, chunkSize, densityField);
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
        Vec3f[] d_vertexPositions = listHolder.stream().map(e->e.solvedPosition).toArray(Vec3f[]::new);
        Vec3f[] d_vertexNormals = listHolder.stream().map(e->e.averageNormal).toArray(Vec3f[]::new);

        processDc(chunkSize, chunkMin, voxelsPerChunk, seamNodes, buffer, octreeNodes,
                d_nodeCodes, d_nodeMaterials, d_vertexPositions, d_vertexNormals);
        return true;
    }

    private void processDc(int chunkSize, Vec3i chunkMin, int voxelsPerChunk, List<PointerBasedOctreeNode> seamNodes,
                           MeshBuffer buffer, Map<Integer, Integer> octreeNodes, int[] d_nodeCodes, int[] d_nodeMaterials,
                           Vec3f[] d_vertexPositions, Vec3f[] d_vertexNormals) {
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

    int GenerateDefaultField(float[] densityField, Vec3i offset, int sampleScale, int defaultMaterialIndex,
                             int[] field_materials)
    {
        int size = 0;
        for (int z = 0; z < fieldSize; z++) {
            for (int y = 0; y < fieldSize; y++) {
                for (int x = 0; x < fieldSize; x++) {
                    Vec3i local_pos = new Vec3i(x, y, z);
                    Vec3f world_pos = local_pos.mul(sampleScale).add(offset).toVec3f();
                    float density = Density.getNoise(world_pos, densityField);
                    int index = field_index(local_pos);
                    int material = density < 0.f ? defaultMaterialIndex : MATERIAL_AIR;
                    field_materials[index] = material;
                    if(material==defaultMaterialIndex) size++;
                }
            }
        }
        return size;
    }

    private boolean createLeafVoxelNodesExperimental(int chunkSize, Vec3i chunkMin, int voxelsPerChunk,
                                                     float[] densityField,
                                                     List<PointerBasedOctreeNode> seamNodes, MeshBuffer buffer) {
        int availableProcessors = Runtime.getRuntime().availableProcessors();
        int threadBound = (voxelsPerChunk * voxelsPerChunk * voxelsPerChunk) / availableProcessors;

        int[] materials = new int[fieldSize*fieldSize*fieldSize];
        GenerateDefaultField(densityField, chunkMin, chunkSize / voxelsPerChunk, MATERIAL_SOLID, materials);

        boolean[] d_leafOccupancy = new boolean[voxelsPerChunk * voxelsPerChunk * voxelsPerChunk];
        int[] d_leafEdgeInfo = new int[voxelsPerChunk * voxelsPerChunk * voxelsPerChunk];
        int[] d_leafCodes = new int[voxelsPerChunk * voxelsPerChunk * voxelsPerChunk];
        int[] d_leafMaterials = new int[voxelsPerChunk * voxelsPerChunk * voxelsPerChunk];

        ExecutorService service = Executors.newFixedThreadPool(availableProcessors);
        int activeLeafsSize = 0;
        List<Callable<Integer>> tasks = new ArrayList<>();
        for (int i = 0; i < availableProcessors; i++) {
            int finalI = i;
            Callable<Integer> task = () -> {
                int from = finalI * threadBound;
                int to = from + threadBound;
                return FindActiveVoxels(from, to, materials,
                        d_leafOccupancy, d_leafEdgeInfo, d_leafCodes, d_leafMaterials);
            };
            tasks.add(task);
        }
        try {
            List<Future<Integer>> futures = service.invokeAll(tasks);
            for (Future<Integer> size : futures) {
                activeLeafsSize += size.get();
            }
        } catch (InterruptedException | ExecutionException e) {
            e.printStackTrace();
            return false;
        }
        service.shutdown();

        if (activeLeafsSize == 0) {
            return false;
        }

        int[] d_nodeCodes = new int[activeLeafsSize];
        int[] d_compactLeafEdgeInfo = new int[activeLeafsSize];
        int[] d_nodeMaterials = new int[activeLeafsSize];

        Map<Integer, Integer> octreeNodes = compactVoxels(d_leafOccupancy, d_leafEdgeInfo, d_leafCodes, d_leafMaterials,
                d_nodeCodes, d_compactLeafEdgeInfo, d_nodeMaterials, activeLeafsSize);

        int numVertices = activeLeafsSize;
        QefSolver[] qefs = new QefSolver[numVertices];
        Vec3f[] d_vertexNormals = new Vec3f[numVertices];
        createLeafNodes(chunkSize, chunkMin, 0, numVertices, densityField,
                d_nodeCodes, d_compactLeafEdgeInfo,
                d_vertexNormals, qefs);

        Vec3f[] d_vertexPositions = new Vec3f[numVertices];
        solveQEFs(d_nodeCodes, chunkSize, voxelsPerChunk, chunkMin, 0, numVertices,
                qefs, d_vertexPositions);

        processDc(chunkSize, chunkMin, voxelsPerChunk, seamNodes, buffer, octreeNodes, d_nodeCodes, d_nodeMaterials, d_vertexPositions, d_vertexNormals);
        return true;
    }

    private static class LinearLeafHolder{
        int materialIndex;
        int voxelEdgeInfo;
        int encodedVoxelPosition;
        Vec3f solvedPosition;
        Vec3f averageNormal;
    }

    private LinearLeafHolder constructLeaf(Vec3i pos, Vec3i chunkMin, int chunkSize, float[] densityField) {
        int leafSize = (chunkSize / VOXELS_PER_CHUNK);
        Vec3i leafMin = pos.mul(leafSize).add(chunkMin);
        int[] cornerMaterials = new int[8];
        int corners = 0;
        for (int i = 0; i < 8; i++) {
            Vec3f cornerPos = leafMin.add(CHILD_MIN_OFFSETS[i].mul(leafSize)).toVec3f();
            float density = Density.getNoise(cornerPos, densityField);
            int material = density < 0.f ? MATERIAL_SOLID : MATERIAL_AIR;
            cornerMaterials[i] = material;
            corners |= (material << i);
        }
        if (corners == 0 || corners == 255) {
            // to avoid holes in seams between chunks with different resolution we creating some other nodes only in seams
            //https://www.reddit.com/r/VoxelGameDev/comments/6kn8ph/dual_contouring_seam_stitching_problem/
            PosNormHolder holder = tryToCreateBoundSeamPseudoNode(leafMin, leafSize, pos, corners, LEAF_SIZE_SCALE, densityField);
            if(holder==null){
                return null;
            } else {
                LinearLeafHolder leafHolder = new LinearLeafHolder();
                leafHolder.solvedPosition = holder.position;
                int materialIndex = findDominantMaterial(cornerMaterials);
                leafHolder.materialIndex = (materialIndex << 8) | corners;
                leafHolder.encodedVoxelPosition = LinearOctreeTest.codeForPosition(pos, MAX_OCTREE_DEPTH);
                leafHolder.voxelEdgeInfo = corners;//edgeList;
                leafHolder.averageNormal = holder.averageNormal;
                return leafHolder;
            }
        }
        int edgeList = 0;

        // otherwise the voxel contains the surface, so find the edge intersections
        int MAX_CROSSINGS = 6;
        int edgeCount = 0;
        Vec3f averageNormal = new Vec3f(0.f);
        QefSolver qef = new QefSolver();
        for (int i = 0; i < 12 && edgeCount < MAX_CROSSINGS; i++) {
            int c1 = edgevmap[i][0];
            int c2 = edgevmap[i][1];
            int m1 = (corners >> c1) & 1;
            int m2 = (corners >> c2) & 1;
            int signChange = m1 != m2 ? 1 : 0;
            edgeList |= (signChange << i);
            if ((m1 == MATERIAL_AIR && m2 == MATERIAL_AIR) || (m1 == MATERIAL_SOLID && m2 == MATERIAL_SOLID)) {
                continue; // no zero crossing on this edge
            }
            Vec3f p1 = leafMin.add(CHILD_MIN_OFFSETS[c1].mul(leafSize)).toVec3f();
            Vec3f p2 = leafMin.add(CHILD_MIN_OFFSETS[c2].mul(leafSize)).toVec3f();
            Vec3f p = VoxelHelperUtils.ApproximateZeroCrossingPosition(p1, p2, densityField).getVec3f();
            Vec3f n = VoxelHelperUtils.CalculateSurfaceNormal(p, densityField);
            qef.add(p, n);
            averageNormal = averageNormal.add(n);
            edgeCount++;
        }

        Vec3f qefPosition = new Vec3f(qef.getMassPoint());
        qef.solve(qefPosition, QEF_ERROR, QEF_SWEEPS, QEF_ERROR);

        LinearLeafHolder leafHolder = new LinearLeafHolder();
        leafHolder.solvedPosition = VoxelHelperUtils.isOutFromBounds(qefPosition, leafMin.toVec3f(), leafSize) ? qef.getMassPoint(): qefPosition;
        int materialIndex = findDominantMaterial(cornerMaterials);
        leafHolder.materialIndex = (materialIndex << 8) | corners;
        leafHolder.encodedVoxelPosition = LinearOctreeTest.codeForPosition(pos, MAX_OCTREE_DEPTH);
        leafHolder.voxelEdgeInfo = edgeList;
        leafHolder.averageNormal = averageNormal.div((float)edgeCount).normalize();
        return leafHolder;
    }

    int FindActiveVoxels(int from, int to, int[] materials,
                         boolean[] voxelOccupancy, int[] voxelEdgeInfo, int[] voxelPositions, int[] voxelMaterials) {
        int size = 0;
        for (int k = from; k < to; k++) {
            int indexShift = log2(VOXELS_PER_CHUNK); // max octree depth
            int x = (k >> (indexShift * 0)) & VOXELS_PER_CHUNK - 1;
            int y = (k >> (indexShift * 1)) & VOXELS_PER_CHUNK - 1;
            int z = (k >> (indexShift * 2)) & VOXELS_PER_CHUNK - 1;

            int index = x + (y * VOXELS_PER_CHUNK) + (z * VOXELS_PER_CHUNK * VOXELS_PER_CHUNK);
            Vec3i pos = new Vec3i(x, y, z);

            int[] cornerMaterials = {
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[0]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[1]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[2]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[3]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[4]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[5]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[6]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[7]))],
            };

            // record the on/off values at the corner of each voxel
            int cornerValues = 0;
            cornerValues |= (((cornerMaterials[0]) == MATERIAL_AIR ? 0 : 1) << 0);
            cornerValues |= (((cornerMaterials[1]) == MATERIAL_AIR ? 0 : 1) << 1);
            cornerValues |= (((cornerMaterials[2]) == MATERIAL_AIR ? 0 : 1) << 2);
            cornerValues |= (((cornerMaterials[3]) == MATERIAL_AIR ? 0 : 1) << 3);
            cornerValues |= (((cornerMaterials[4]) == MATERIAL_AIR ? 0 : 1) << 4);
            cornerValues |= (((cornerMaterials[5]) == MATERIAL_AIR ? 0 : 1) << 5);
            cornerValues |= (((cornerMaterials[6]) == MATERIAL_AIR ? 0 : 1) << 6);
            cornerValues |= (((cornerMaterials[7]) == MATERIAL_AIR ? 0 : 1) << 7);

            if (cornerValues != 0 && cornerValues != 255) {
                ++size;
            }
            // record which of the 12 voxel edges are on/off
            int edgeList = 0;
            for (int i = 0; i < 12; i++) {
                int i0 = edgevmap[i][0];
                int i1 = edgevmap[i][1];
                int edgeStart = (cornerValues >> i0) & 1;
                int edgeEnd = (cornerValues >> i1) & 1;
                int signChange = edgeStart != edgeEnd ? 1 : 0;
                edgeList |= (signChange << i);
            }
            voxelOccupancy[index] = cornerValues != 0 && cornerValues != 255;
            voxelPositions[index] = LinearOctreeTest.codeForPosition(pos, MAX_OCTREE_DEPTH);
            voxelEdgeInfo[index] = edgeList;

            // store cornerValues here too as its needed by the CPU side and edgeInfo isn't exported
            int materialIndex = findDominantMaterial(cornerMaterials);
            voxelMaterials[index] = (materialIndex << 8) | cornerValues;
        }
        return size;
    }

    private Map<Integer, Integer> compactVoxels(boolean[] voxelValid, int[] voxelEdgeInfo, int[] voxelPositions, int[] voxelMaterials,
                               int[] compactPositions, int[] compactEdgeInfo, int[] compactMaterials, int numVertices) {
        int current = 0;
        Map<Integer, Integer> octreeNodes = new HashMap<>(numVertices);
        for (int i = 0; i < voxelPositions.length; i++) {
            if (voxelValid[i]) {
                octreeNodes.put(voxelPositions[i], current);
                compactPositions[current] = voxelPositions[i];
                compactEdgeInfo[current] = voxelEdgeInfo[i];
                compactMaterials[current] = voxelMaterials[i];
                ++current;
            }
        }
        return octreeNodes;
    }

    private Integer createLeafNodes(int chunkSize, Vec3i chunkMin, int from, int to, float[] densityField,
                                    int[] voxelPositions,
                                    int[] voxelEdgeInfo,
                                    Vec3f[] vertexNormals, QefSolver[] qefs) {
        for (int index = from; index < to; index++) {
            int encodedPosition = voxelPositions[index];
            Vec3i position = LinearOctreeTest.positionForCode(encodedPosition);
            int corners = voxelEdgeInfo[index];

            int MAX_CROSSINGS = 6;
            int edgeCount = 0;
            Vec3f averageNormal = new Vec3f(0.f);
            QefSolver qef = new QefSolver();

            int leafSize = (chunkSize / VOXELS_PER_CHUNK);
            Vec3i leafMin = position.mul(leafSize).add(chunkMin);

            for (int i = 0; i < 12 && edgeCount < MAX_CROSSINGS; i++) {
                int active = (corners >> i) & 1;
                if (active==0) {
                    continue;
                }
                int c1 = edgevmap[i][0];
                int c2 = edgevmap[i][1];
                Vec3f p1 = leafMin.add(CHILD_MIN_OFFSETS[c1].mul(leafSize)).toVec3f();
                Vec3f p2 = leafMin.add(CHILD_MIN_OFFSETS[c2].mul(leafSize)).toVec3f();
                Vec3f p = VoxelHelperUtils.ApproximateLevenCrossingPosition(p1, p2, densityField).getVec3f();
                Vec3f n = VoxelHelperUtils.CalculateSurfaceNormal(p, densityField);
                qef.add(p, n);
                averageNormal = averageNormal.add(n);
                edgeCount++;
            }
            qefs[index] = qef;
            vertexNormals[index] = averageNormal.div((float) edgeCount).normalize();
        }
        return 1;
    }

    private int solveQEFs(int[] d_nodeCodes, int chunkSize, int voxelsPerChunk, Vec3i chunkMin, int from, int to,
                          QefSolver[] qefs, Vec3f[] solvedPositions) {
        for (int index = from; index < to; index++) {
            int encodedPosition = d_nodeCodes[index];
            Vec3i pos = LinearOctreeTest.positionForCode(encodedPosition);
            int leafSize = (chunkSize / voxelsPerChunk);
            Vec3i leaf = pos.mul(leafSize).add(chunkMin);

            QefSolver qef = qefs[index];

            Vec3f qefPosition = new Vec3f(qef.getMassPoint());
            qef.solve(qefPosition, QEF_ERROR, QEF_SWEEPS, QEF_ERROR);
            //qefPosition = qefPosition.mul(leafSize).add(chunkMin.toVec3f());
            Vec3f position = VoxelHelperUtils.isOutFromBounds(qefPosition, leaf.toVec3f(), leafSize) ? qef.getMassPoint() : qefPosition;
            solvedPositions[index] = position;
        }
        return 1;
    }

    private int findSeamNodes(int[] nodeCodes, boolean[] isSeamNode, int from, int to) {
        int res = 0;
        for (int index = from; index < to; index++) {
            int code = nodeCodes[index];
            Vec3i position = LinearOctreeTest.positionForCode(code);
            boolean xSeam = position.x == 0 || position.x == (VOXELS_PER_CHUNK - 1);
            boolean ySeam = position.y == 0 || position.y == (VOXELS_PER_CHUNK - 1);
            boolean zSeam = position.z == 0 || position.z == (VOXELS_PER_CHUNK - 1);
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
                boolean isEdgeVoxel = a == (VOXELS_PER_CHUNK - 1) || b == (VOXELS_PER_CHUNK - 1);
                if (isEdgeVoxel) {
                    continue;
                }

                nodeIndices[0] = index;
                for (int n = 1; n < 4; n++) {
                    Vec3i p = offset.add(EDGE_NODE_OFFSETS[axis][n]);
                    int c = LinearOctreeTest.codeForPosition(p, MAX_OCTREE_DEPTH);
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

        boolean signChange = (m1 == MATERIAL_AIR && m2 != MATERIAL_AIR) || (m1 != MATERIAL_AIR && m2 == MATERIAL_AIR);
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

    private void GenerateMeshVertexBuffer(Vec3f[] vertexPositions, Vec3f[] vertexNormals, int[] nodeMaterials, Vec3f colour,
                                          MeshVertex[] meshVertexBuffer) {
        for (int index = 0; index < vertexPositions.length; index++) {
            int material = nodeMaterials[index];
            meshVertexBuffer[index] = new MeshVertex();
            meshVertexBuffer[index].setPos(vertexPositions[index]);
            meshVertexBuffer[index].setNormal(vertexNormals[index]);
            meshVertexBuffer[index].setColor(colour); //colour = new Vec4f(colour.X, colour.Y, colour.Z, (float) (material >> 8));
        }
    }

    private void extractNodeInfo(boolean[] isSeamNode, Vec3f color,
                                     int leafSize, Vec3i chunkMin, int from, int to,
                                     int[] octreeCodes, int[] octreeMaterials, Vec3f[] octreePositions, Vec3f[] octreeNormals,
                                     List<PointerBasedOctreeNode> seamNodes) {
        for (int index = from; index < to; index++) {
            if (isSeamNode==null || isSeamNode[index]) {
                PointerBasedOctreeNode node = new PointerBasedOctreeNode();
                node.min = LinearOctreeTest.positionForCode(octreeCodes[index]).mul(leafSize).add(chunkMin);
                node.size = leafSize;
                node.Type = OctreeNodeType.Node_Leaf;
                OctreeDrawInfo drawInfo = new OctreeDrawInfo();
                drawInfo.position = octreePositions[index];
                drawInfo.color = color;
                drawInfo.averageNormal = octreeNormals[index];
                drawInfo.corners = octreeMaterials[index];
                node.drawInfo = drawInfo;
                seamNodes.add(node);
            }
        }
    }
}