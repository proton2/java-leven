package dc.impl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.math.Vec4i;
import core.utils.BufferUtil;
import core.utils.Constants;
import dc.*;
import dc.entities.MeshBuffer;
import dc.entities.MeshVertex;
import dc.solver.LevenQefSolver;
import dc.solver.QEFData;
import dc.utils.VoxelHelperUtils;

import java.util.*;

import static dc.utils.SimplexNoise.getNoise;

/*
    Transition implementation Linear octree dual contouring between simple implementation and NickGildea OpenCL DC implementation
    Some holes in seams is not fixed.
 */

public class TransitionLinearOctreeImpl extends AbstractDualContouring implements VoxelOctree {
    public TransitionLinearOctreeImpl(MeshGenerationContext meshGenerationContext, ICSGOperations csgOperations,
                                      Map<Vec4i, GPUDensityField> densityFieldCache, Map<Vec4i, GpuOctree> octreeCache) {
        super(meshGenerationContext, csgOperations, densityFieldCache, octreeCache);
    }

    @Override
    public boolean createLeafVoxelNodes(ChunkNode node, List<OctreeNode> seamNodes, MeshBuffer buffer) {
        // experimental transitional branch. It worked successfull but seams have some holes. More faster
        return createLeafVoxelNodesExperimental(node, seamNodes, buffer);
    }

    private void processDc(ChunkNode node, List<OctreeNode> seamNodes, MeshBuffer buffer,
                           Map<Integer, Integer> octreeNodes, int[] d_nodeCodes, int[] d_nodeMaterials,
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

        extractNodeInfo(isSeamNode, Constants.Yellow,node.size / meshGen.getVoxelsPerChunk(), node.min, 0, numVertices,
                d_nodeCodes, d_nodeMaterials, d_vertexPositions, d_vertexNormals, seamNodes);

        Set<Integer> seamNodeCodes = new HashSet<>(seamSize);
        for (OctreeNode seamNode : seamNodes) {
            if (seamNode.size > meshGen.leafSizeScale) {
                seamNodeCodes.add(codeForPosition(seamNode.nodeNum, meshGen.MAX_OCTREE_DEPTH));
            }
        }
        List<OctreeNode> addedNodes = findAndCreateBorderNodes(seamNodeCodes, node.min, node.size / meshGen.getVoxelsPerChunk());
        seamNodes.addAll(addedNodes);
    }

    int GenerateDefaultField(Vec3i offset, int from, int to, int sampleScale, int defaultMaterialIndex,
                             int[] field_materials)
    {
        int size = 0;
        for (int z = 0; z < meshGen.getFieldSize(); z++) {
            for (int y = 0; y < meshGen.getFieldSize(); y++) {
                for (int x = 0; x < meshGen.getFieldSize(); x++) {
                    Vec3i local_pos = new Vec3i(x, y, z);
                    Vec3i world_pos = local_pos.mul(sampleScale).add(offset);
                    float density = getNoise(world_pos);
                    int index = meshGen.getMaterialIndex(local_pos);
                    int material = density < 0.f ? defaultMaterialIndex : meshGen.MATERIAL_AIR;
                    field_materials[index] = material;
                    if(material==defaultMaterialIndex) size++;
                }
            }
        }
        return size;
    }

    private boolean createLeafVoxelNodesExperimental(ChunkNode node,
                                                     List<OctreeNode> seamNodes, MeshBuffer buffer) {
        int availableProcessors = Runtime.getRuntime().availableProcessors();
        int threadBound = (meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk()) / availableProcessors;

        int[] materials = new int[meshGen.getFieldSize()*meshGen.getFieldSize()*meshGen.getFieldSize()];
        GenerateDefaultField(node.min, 0, meshGen.getFieldSize()*meshGen.getFieldSize()*meshGen.getFieldSize(),
                node.size / meshGen.getVoxelsPerChunk(), meshGen.MATERIAL_SOLID, materials);
//        GPUDensityField field = new GPUDensityField();
//        field.setMin(chunkMin);
//        field.setSize(chunkSize);
//        BufferGpuService bufferGpuService = new BufferGpuService(ctx);
//        OpenCLCalculateMaterialsService calculateMaterialsService = new OpenCLCalculateMaterialsService(ctx, meshGen.getFieldSize(),
//                meshGen, field, bufferGpuService);
//        calculateMaterialsService.run(kernels, materials);

//        int count = 0;
//        for (int material : materials) {
//            if (material > 0) {
//                ++count;
//            }
//        }
        boolean[] d_leafOccupancy = new boolean[meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk()];
        int[] d_leafEdgeInfo = new int[meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk()];
        int[] d_leafCodes = new int[meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk()];
        int[] d_leafMaterials = new int[meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk()];
        Map<Integer, Vec4f> borderNodePositions = new HashMap<>();

        //ExecutorService service = Executors.newFixedThreadPool(availableProcessors);
        //  int activeLeafsSize = 0;
//        List<Callable<Integer>> tasks = new ArrayList<>();
//        for (int i = 0; i < availableProcessors; i++) {
//            int finalI = i;
//            Callable<Integer> task = () -> {
//                int from = finalI * threadBound;
//                int to = from + threadBound;
//                return FindActiveVoxels(chunkSize, chunkMin, from, to, materials,
//                        d_leafOccupancy, d_leafEdgeInfo, d_leafCodes, d_leafMaterials, borderNodePositions);
//            };
//            tasks.add(task);
//        }
//        try {
//            List<Future<Integer>> futures = service.invokeAll(tasks);
//            for (Future<Integer> size : futures) {
//                activeLeafsSize += size.get();
//            }
//        } catch (InterruptedException | ExecutionException e) {
//            e.printStackTrace();
//            return false;
//        }
//        service.shutdown();
        int activeLeafsSize = FindActiveVoxels(node.size, node.min,
                0, meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk(), materials,
                d_leafOccupancy, d_leafEdgeInfo, d_leafCodes, d_leafMaterials, borderNodePositions);

        if (activeLeafsSize == 0) {
            return false;
        }

        int[] d_nodeCodes = new int[activeLeafsSize];
        int[] d_compactLeafEdgeInfo = new int[activeLeafsSize];
        int[] d_nodeMaterials = new int[activeLeafsSize];

        Map<Integer, Integer> octreeNodes = compactVoxels(d_leafOccupancy, d_leafEdgeInfo, d_leafCodes, d_leafMaterials,
                d_nodeCodes, d_compactLeafEdgeInfo, d_nodeMaterials, activeLeafsSize);

        int numVertices = activeLeafsSize;
        QEFData[] qefs = new QEFData[numVertices];
        Vec3f[] d_vertexNormals = new Vec3f[numVertices];
        createLeafNodes(node.size, node.min, 0, numVertices, borderNodePositions,
                d_nodeCodes, d_compactLeafEdgeInfo,
                d_vertexNormals, qefs);

//        ExecutorService serviceLeafs = Executors.newFixedThreadPool(availableProcessors);
//        List<Callable<Integer>> leafsTasks = new ArrayList<>();
//        for (int i = 0; i < availableProcessors; i++) {
//            int finalI = i;
//            Callable<Integer> task = () -> {
//                int from = finalI * threadBound;
//                int to = from + threadBound;
//                return createLeafNodes(chunkSize, chunkMin, from, to, densityField, borderNodePositions,
//                        d_nodeCodes, d_compactLeafEdgeInfo,
//                        d_vertexNormals, qefs);
//            };
//            leafsTasks.add(task);
//        }
//        try {
//            serviceLeafs.invokeAll(leafsTasks);
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//            return false;
//        }
//        serviceLeafs.shutdown();

        Vec3f[] d_vertexPositions = new Vec3f[numVertices];
        solveQEFs(d_nodeCodes, node, meshGen.getVoxelsPerChunk(),0, numVertices,
                qefs, d_vertexPositions);

        processDc(node, seamNodes, buffer, octreeNodes, d_nodeCodes, d_nodeMaterials, d_vertexPositions, d_vertexNormals);
        return true;
    }

    int FindActiveVoxels(int chunkSize, Vec3i chunkMin,
                         int from, int to, int[] materials,
                         boolean[] voxelOccupancy, int[] voxelEdgeInfo, int[] voxelPositions, int[] voxelMaterials, Map<Integer, Vec4f> borderNodePositions) {
        int size = 0;
        for (int k = from; k < to; k++) {
            int indexShift = VoxelHelperUtils.log2(meshGen.getVoxelsPerChunk()); // max octree depth
            int x = (k >> (indexShift * 0)) & meshGen.getVoxelsPerChunk() - 1;
            int y = (k >> (indexShift * 1)) & meshGen.getVoxelsPerChunk() - 1;
            int z = (k >> (indexShift * 2)) & meshGen.getVoxelsPerChunk() - 1;

            int index = x + (y * meshGen.getVoxelsPerChunk()) + (z * meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk());
            Vec3i pos = new Vec3i(x, y, z);

            int[] cornerMaterials = {
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[0]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[1]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[2]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[3]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[4]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[5]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[6]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[7]))],
            };

            // record the on/off values at the corner of each voxel
            int cornerValues = 0;
            cornerValues |= (((cornerMaterials[0]) == meshGen.MATERIAL_AIR ? 0 : 1) << 0);
            cornerValues |= (((cornerMaterials[1]) == meshGen.MATERIAL_AIR ? 0 : 1) << 1);
            cornerValues |= (((cornerMaterials[2]) == meshGen.MATERIAL_AIR ? 0 : 1) << 2);
            cornerValues |= (((cornerMaterials[3]) == meshGen.MATERIAL_AIR ? 0 : 1) << 3);
            cornerValues |= (((cornerMaterials[4]) == meshGen.MATERIAL_AIR ? 0 : 1) << 4);
            cornerValues |= (((cornerMaterials[5]) == meshGen.MATERIAL_AIR ? 0 : 1) << 5);
            cornerValues |= (((cornerMaterials[6]) == meshGen.MATERIAL_AIR ? 0 : 1) << 6);
            cornerValues |= (((cornerMaterials[7]) == meshGen.MATERIAL_AIR ? 0 : 1) << 7);

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

            int codePos = codeForPosition(pos, meshGen.MAX_OCTREE_DEPTH);
            boolean isHaveVoxel = cornerValues != 0 && cornerValues != 255;
            if (isHaveVoxel) {
                ++size;
            }
            voxelOccupancy[index] = isHaveVoxel;
            voxelPositions[index] = codePos;
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

    private Integer createLeafNodes(int chunkSize, Vec3i chunkMin, int from, int to,
                                    Map<Integer, Vec4f> borderNodePositions,
                                    int[] voxelPositions,
                                    int[] voxelEdgeInfo,
                                    Vec3f[] vertexNormals, QEFData[] qefs) {
        for (int index = from; index < to; index++) {
            int encodedPosition = voxelPositions[index];
            Vec3i position = positionForCode(encodedPosition);
            int corners = voxelEdgeInfo[index];
            QEFData qef = new QEFData(new LevenQefSolver());

            int MAX_CROSSINGS = 6;
            int edgeCount = 0;
            Vec4f averageNormal = new Vec4f();
            Vec4f[] edgePositions = new Vec4f[12];
            Vec4f[] edgeNormals = new Vec4f[12];

            int leafSize = (chunkSize / meshGen.getVoxelsPerChunk());
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
                Vec4f p = ApproximateLevenCrossingPosition(p1, p2);
                Vec4f n = CalculateSurfaceNormal(p);
                edgePositions[edgeCount] = p;
                edgeNormals[edgeCount] = n;
                averageNormal = averageNormal.add(n);
                edgeCount++;
            }
            qef.qef_create_from_points(edgePositions, edgeNormals, edgeCount);
            qefs[index] = qef;
            vertexNormals[index] = averageNormal.div((float) edgeCount).getVec3f().normalize();
        }
        return 1;
    }

    private int solveQEFs(int[] d_nodeCodes, ChunkNode node, int voxelsPerChunk, int from, int to,
                          QEFData[] qefs, Vec3f[] solvedPositions) {
        for (int index = from; index < to; index++) {
            int encodedPosition = d_nodeCodes[index];
            Vec3i pos = positionForCode(encodedPosition);
            int leafSize = (node.size / voxelsPerChunk);
            Vec3i leaf = pos.mul(leafSize).add(node.min);

            Vec3f qefPosition;
            if (leafSize == meshGen.leafSizeScale) {
                qefPosition = qefs[index].solve().getVec3f();       // run solver only for LOD 0
            } else {
                qefPosition = qefs[index].getMasspoint().getVec3f();// for other LOD's get masspoint - to increase performance
            }
            //qefPosition = qefPosition.mul(leafSize).add(chunkMin.toVec3f());
            solvedPositions[index] = qefPosition;
        }
        return 1;
    }

    private int findSeamNodes(int[] nodeCodes, boolean[] isSeamNode, int from, int to) {
        int res = 0;
        for (int index = from; index < to; index++) {
            int code = nodeCodes[index];
            Vec3i position = positionForCode(code);
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

            Vec3i offset = positionForCode(code);
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
                    int c = codeForPosition(p, meshGen.MAX_OCTREE_DEPTH);
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
                                     List<OctreeNode> seamNodes) {
        for (int index = from; index < to; index++) {
            if (isSeamNode==null || isSeamNode[index]) {
                Vec3i min = positionForCode(octreeCodes[index]).mul(leafSize).add(chunkMin);
                PointerBasedOctreeNode node = new PointerBasedOctreeNode(min, leafSize, OctreeNodeType.Node_Leaf);
                OctreeDrawInfo drawInfo = new OctreeDrawInfo();
                drawInfo.position = octreePositions[index];
                drawInfo.color = color;
                drawInfo.averageNormal = octreeNormals[index];
                node.corners = octreeMaterials[index];
                node.drawInfo = drawInfo;
                node.nodeNum = positionForCode(octreeCodes[index]);
                seamNodes.add(node);
            }
        }
    }
}
