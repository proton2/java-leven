package dc.impl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.math.Vec4i;
import core.utils.BufferUtil;
import dc.*;
import dc.entities.MeshBuffer;
import dc.entities.MeshVertex;
import dc.impl.opencl.*;
import dc.solver.LevenQefSolver;
import dc.solver.QefSolver;
import dc.utils.VoxelHelperUtils;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/*
    Temporary class to catch bug in seams in GPU OpenCL version
 */

public class LevenLinearTestOctreeImpl extends AbstractDualContouring implements VoxelOctree {
    private final KernelsHolder kernels;
    private final ComputeContext ctx;

    public LevenLinearTestOctreeImpl(KernelsHolder kernels, MeshGenerationContext meshGenerationContext, ComputeContext ctx) {
        super(meshGenerationContext);
        this.kernels = kernels;
        this.ctx = ctx;
    }

    @Override
    public boolean createLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                        float[] densityField,
                                        List<PointerBasedOctreeNode> seamNodes, MeshBuffer buffer, GPUDensityField field)
    {
        int[] materials = new int[meshGen.getFieldSize() * meshGen.getFieldSize() * meshGen.getFieldSize()];

        field.setMin(chunkMin);
        field.setSize(chunkSize);
        OpenCLCalculateMaterialsService calculateMaterialsService = new OpenCLCalculateMaterialsService(ctx, meshGen.getFieldSize(), meshGen, field);
        calculateMaterialsService.run(kernels, materials);

        //ScanOpenCLService scanService = new ScanOpenCLService(ctx, kernels.getKernel(KernelNames.SCAN));
        //FindDefaultEdgesOpenCLService findDefEdges = new FindDefaultEdgesOpenCLService(ctx, meshGen, field, scanService);

        int edgeBufferSize = meshGen.getHermiteIndexSize() * meshGen.getHermiteIndexSize() * meshGen.getHermiteIndexSize() * 3;
        int[] edgeOccupancy = new int[edgeBufferSize];
        int[] edgeIndicesNonCompact = new int[edgeBufferSize];
        //int compactEdgesSize = findDefEdges.findFieldEdgesKernel(kernels, meshGen.getHermiteIndexSize(), edgeOccupancy, edgeIndicesNonCompact);
        int compactEdgesSize = FindFieldEdges(materials, edgeOccupancy, edgeIndicesNonCompact);
        if(compactEdgesSize<=0){
            return false;
        }

        int[] edgeIndicesCompact = new int [compactEdgesSize];
        //////////////////////////////
        Map<Integer, Integer> edgeIndicatesMap = compactEdges(edgeOccupancy, edgeIndicesNonCompact, compactEdgesSize,
                edgeIndicesCompact);

        //findDefEdges.compactEdgeKernel(kernels, field);
        Vec4f[] normals = new Vec4f[compactEdgesSize];
        //findDefEdges.FindEdgeIntersectionInfoKernel(kernels, normals);
        FindEdgeIntersectionInfo(chunkMin, chunkSize / meshGen.getVoxelsPerChunk(), 0, compactEdgesSize, densityField,
                edgeIndicesCompact,
                normals);

        //GpuOctree gpuOctree = new GpuOctree();
        //ConstructOctreeFromFieldService constructOctreeFromFieldService = new ConstructOctreeFromFieldService(ctx, meshGen, field, gpuOctree, scanService);

        int[] d_leafOccupancy = new int [meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()];
        int[] d_leafEdgeInfo = new int [meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()];
        int[] d_leafCodes = new int [meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()];
        int[] d_leafMaterials = new int [meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()];
        //int octreeNumNodes = constructOctreeFromFieldService.findActiveVoxelsKernel(kernels, d_leafOccupancy, d_leafEdgeInfo, d_leafCodes, d_leafMaterials);
        int octreeNumNodes = FindActiveVoxels(0, meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk(), materials,
                d_leafOccupancy, d_leafEdgeInfo, d_leafCodes, d_leafMaterials);
        if (octreeNumNodes<=0){
            return false;
        }

        int[] d_nodeCodes = new int[octreeNumNodes];
        int[] d_compactLeafEdgeInfo = new int[octreeNumNodes];
        int[] d_nodeMaterials = new int[octreeNumNodes];
        //constructOctreeFromFieldService.compactVoxelsKernel(kernels, null, null);
        Map<Integer, Integer> octreeNodes = compactVoxels(d_leafOccupancy, d_leafEdgeInfo, d_leafCodes, d_leafMaterials,
                d_nodeCodes, d_compactLeafEdgeInfo, d_nodeMaterials, octreeNumNodes);

        Vec4f[] d_vertexNormals = new Vec4f[octreeNumNodes];
        QefSolver[] qefs = new QefSolver[octreeNumNodes];
        //constructOctreeFromFieldService.createLeafNodesKernel(kernels, qefs, d_vertexNormals);
        createLeafNodes(0, octreeNumNodes, d_nodeCodes, d_compactLeafEdgeInfo, normals, edgeIndicatesMap,
                qefs, d_vertexNormals);

        Vec4f[] d_vertexPositions = new Vec4f[octreeNumNodes];
        //constructOctreeFromFieldService.solveQefKernel(kernels, d_vertexPositions);
        solveQEFs(chunkSize, meshGen.getVoxelsPerChunk(), chunkMin, 0, octreeNumNodes, qefs,
                d_vertexPositions);

        //////////////////////////////
        //MeshBufferGPU meshBufferGPU = new MeshBufferGPU();
        //GenerateMeshFromOctreeService generateMeshFromOctreeService = new GenerateMeshFromOctreeService(ctx, meshGen, scanService, gpuOctree, meshBufferGPU);

        int[] d_indexBuffer = new int[octreeNumNodes * 6 * 3];
        int[] d_trianglesValid = new int[octreeNumNodes * 3];

        int trianglesValidCount = generateMesh(octreeNodes, d_nodeCodes, d_nodeMaterials, d_indexBuffer, d_trianglesValid);
        //int numTriangles = generateMeshFromOctreeService.generateMeshKernel(kernels, d_indexBuffer, d_trianglesValid);
        if(trianglesValidCount<=0){
            return false;
        }

        int numTriangles = trianglesValidCount * 2;
        int[] d_compactIndexBuffer = new int[numTriangles * 3];
        //generateMeshFromOctreeService.compactMeshTrianglesKernel(kernels, numTriangles, d_compactIndexBuffer);
        compactMeshTriangles(d_trianglesValid, d_indexBuffer, d_compactIndexBuffer);

        //////////////////////////////
        MeshVertex[] d_vertexBuffer = new MeshVertex[octreeNumNodes];
        GenerateMeshVertexBuffer(d_vertexPositions, d_vertexNormals, d_nodeMaterials,
                VoxelHelperUtils.ColourForMinLeafSize(chunkSize/meshGen.clipmapLeafSize), d_vertexBuffer);
        buffer.setVertices(BufferUtil.createDcFlippedBufferAOS(d_vertexBuffer));
        buffer.setNumVertices(octreeNumNodes);
        buffer.setIndicates(BufferUtil.createFlippedBuffer(d_compactIndexBuffer));
        buffer.setNumIndicates(d_compactIndexBuffer.length);

        int[] isSeamNode = new int[octreeNumNodes];
        // ToDo return seamNodes which size have seamSize from method
        int seamSize = findSeamNodes(d_nodeCodes, isSeamNode, 0, d_nodeCodes.length);

        extractNodeInfo(isSeamNode, VoxelHelperUtils.ColourForMinLeafSize(chunkSize/meshGen.getVoxelsPerChunk()),//Constants.Yellow,
                chunkSize / meshGen.getVoxelsPerChunk(), chunkMin, 0, octreeNumNodes,
                d_nodeCodes, d_nodeMaterials, d_vertexPositions, d_vertexNormals, seamNodes);

        //findDefEdges.destroy();
        //calculateMaterialsService.destroy();
        return true;
    }

    int GenerateDefaultField(float[] densityField, Vec3i offset, int from, int to, int sampleScale, int defaultMaterialIndex,
                             int[] field_materials)
    {
        int size = 0;
        for (int z = 0; z < meshGen.getFieldSize(); z++) {
            for (int y = 0; y < meshGen.getFieldSize(); y++) {
                for (int x = 0; x < meshGen.getFieldSize(); x++) {
                    Vec3i local_pos = new Vec3i(x, y, z);
                    Vec3f world_pos = local_pos.mul(sampleScale).add(offset).toVec3f();
                    float density = getNoise(world_pos, densityField);
                    int index = field_index(local_pos);
                    int material = density < 0.f ? defaultMaterialIndex : meshGen.MATERIAL_AIR;
                    field_materials[index] = material;
                    if(material==defaultMaterialIndex) size++;
                }
            }
        }
        return size;
    }

    int FindFieldEdges(int[] materials,
                       int[] edgeOccupancy, int[] edgeIndices) {
        int size = 0;
        for (int z = 0; z < meshGen.getHermiteIndexSize(); z++) {
            for (int y = 0; y < meshGen.getHermiteIndexSize(); y++) {
                for (int x = 0; x < meshGen.getHermiteIndexSize(); x++)
                {
                    Vec4i pos = new Vec4i(x, y, z, 0);
                    int index = (x + (y * meshGen.getHermiteIndexSize()) + (z * meshGen.getHermiteIndexSize() * meshGen.getHermiteIndexSize()));
                    int edgeIndex = index * 3;

                    int[] CORNER_MATERIALS = {
                            materials[field_index(pos.add(new Vec4i(0, 0, 0, 0)))],
                            materials[field_index(pos.add(new Vec4i(1, 0, 0, 0)))],
                            materials[field_index(pos.add(new Vec4i(0, 1, 0, 0)))],
                            materials[field_index(pos.add(new Vec4i(0, 0, 1, 0)))],
                    };

                    int voxelIndex = pos.x | (pos.y << meshGen.getIndexShift()) | (pos.z << (meshGen.getIndexShift() * 2));

                    for (int i = 0; i < 3; i++) {
                        int e = 1 + i;
                        boolean signChange =(CORNER_MATERIALS[0] != meshGen.MATERIAL_AIR && CORNER_MATERIALS[e] == meshGen.MATERIAL_AIR) ||
                                (CORNER_MATERIALS[0] == meshGen.MATERIAL_AIR && CORNER_MATERIALS[e] != meshGen.MATERIAL_AIR);
                        edgeOccupancy[edgeIndex + i] = signChange ? 1 : 0;
                        edgeIndices[edgeIndex + i] = signChange ? ((voxelIndex << 2) | i) : -1;
                        if (signChange)
                            ++size;
                    }
                }
            }
        }
        return size;
    }

    Vec3i[] EDGE_END_OFFSETS = {
            new Vec3i(1,0,0),
            new Vec3i(0,1,0),
            new Vec3i(0,0,1)
    };

    void FindEdgeIntersectionInfo(Vec3i chunkMin, int sampleScale, int from, int to, float[] densityField,
                                  int[] encodedEdges,
                                  Vec4f[] edgeInfo)
    {
        for (int index = from; index < to; index++) {
            int edge = encodedEdges[index];
            int axisIndex = edge & 3;
            int hermiteIndex = edge >> 2;

            int x = (hermiteIndex >> (meshGen.getIndexShift() * 0)) & meshGen.getIndexMask();
            int y = (hermiteIndex >> (meshGen.getIndexShift() * 1)) & meshGen.getIndexMask();
            int z = (hermiteIndex >> (meshGen.getIndexShift() * 2)) & meshGen.getIndexMask();

            Vec3f p0 = new Vec3i(x, y, z).mul(sampleScale).add(chunkMin).toVec3f();
            Vec3f p1 = p0.add(EDGE_END_OFFSETS[axisIndex].mul(sampleScale).toVec3f());

            Vec4f p = ApproximateLevenCrossingPosition(p0, p1, densityField);
            Vec4f normal = CalculateSurfaceNormal(p, densityField);

            edgeInfo[index] = new Vec4f(normal.getVec3f(), p.w);
        }
    }

    int FindActiveVoxels(int from, int to, int[] materials,
                         int[] voxelOccupancy,
                         int[] voxelEdgeInfo,
                         int[] voxelPositions,
                         int[] voxelMaterials) {
        int size = 0;
        for (int k = from; k < to; k++) {
            int indexShift = VoxelHelperUtils.log2(meshGen.getVoxelsPerChunk()); // max octree depth
            int x = (k >> (indexShift * 0)) & meshGen.getVoxelsPerChunk() - 1;
            int y = (k >> (indexShift * 1)) & meshGen.getVoxelsPerChunk() - 1;
            int z = (k >> (indexShift * 2)) & meshGen.getVoxelsPerChunk() - 1;

            int index = x + (y * meshGen.getVoxelsPerChunk()) + (z * meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk());
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
            cornerValues |= (((cornerMaterials[0]) == meshGen.MATERIAL_AIR ? 0 : 1) << 0);
            cornerValues |= (((cornerMaterials[1]) == meshGen.MATERIAL_AIR ? 0 : 1) << 1);
            cornerValues |= (((cornerMaterials[2]) == meshGen.MATERIAL_AIR ? 0 : 1) << 2);
            cornerValues |= (((cornerMaterials[3]) == meshGen.MATERIAL_AIR ? 0 : 1) << 3);
            cornerValues |= (((cornerMaterials[4]) == meshGen.MATERIAL_AIR ? 0 : 1) << 4);
            cornerValues |= (((cornerMaterials[5]) == meshGen.MATERIAL_AIR ? 0 : 1) << 5);
            cornerValues |= (((cornerMaterials[6]) == meshGen.MATERIAL_AIR ? 0 : 1) << 6);
            cornerValues |= (((cornerMaterials[7]) == meshGen.MATERIAL_AIR ? 0 : 1) << 7);

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
            voxelOccupancy[index] = cornerValues != 0 && cornerValues != 255 ? 1 : 0;
            voxelPositions[index] = LinearOctreeTest.codeForPosition(pos, meshGen.MAX_OCTREE_DEPTH);
            voxelEdgeInfo[index] = edgeList;

            // store cornerValues here too as its needed by the CPU side and edgeInfo isn't exported
            int materialIndex = findDominantMaterial(cornerMaterials);
            voxelMaterials[index] = (materialIndex << 8) | cornerValues;
        }
        return size;
    }

    Map<Integer, Integer> compactEdges(int[] edgeValid, int[] edges, int compactEdgesSize, int[] compactActiveEdges) {
        int current = 0;
        Map<Integer, Integer> edgeIndicatesMap = new HashMap<>(compactEdgesSize);
        for (int index = 0; index < edges.length; index++) {
            if (edgeValid[index]==1) {
                edgeIndicatesMap.put(edges[index], current);
                compactActiveEdges[current] = edges[index];
                ++current;
            }
        }
        return edgeIndicatesMap;
    }

    void createLeafNodes(int from, int to, int[] voxelPositions, int[] voxelEdgeInfo, Vec4f[] edgeDataTable, Map<Integer, Integer> nodes,
                         QefSolver[] leafQEFs, Vec4f[] vertexNormals)
    {
        for (int index = from; index < to; index++) {
            int encodedPosition = voxelPositions[index];
            Vec3i position = LinearOctreeTest.positionForCode(encodedPosition);

            int edgeList = voxelEdgeInfo[index];

            Vec4f[] edgePositions = new Vec4f[12];
            Vec4f[] edgeNormals = new Vec4f[12];
            int edgeCount = 0;
            QefSolver qef = new QefSolver(new LevenQefSolver());

            for (int i = 0; i < 12; i++) {
                int active = (edgeList >> i) & 1;
                if (active==0) {
                    continue;
                }
                int e0 = edgevmap[i][0];
                int e1 = edgevmap[i][1];
                Vec4f p0 = position.add(CHILD_MIN_OFFSETS[e0]).toVec4f();
                Vec4f p1 = position.add(CHILD_MIN_OFFSETS[e1]).toVec4f();

                // this works due to the layout EDGE_VERTEX_MAP, the first 4 elements are the X axis
                // the next 4 are the Y axis and the last 4 are the Z axis
                int axis = i / 4;
                Vec3i hermiteIndexPosition = position.add(CHILD_MIN_OFFSETS[e0]);
                int edgeIndex = (encodeVoxelIndex(hermiteIndexPosition) << 2) | axis;

                Integer dataIndex = nodes.get(edgeIndex);
                if (dataIndex!=null && dataIndex != ~0) {
                    Vec4f edgeData = edgeDataTable[dataIndex];
                    edgePositions[edgeCount] = VoxelHelperUtils.mix(p0, p1, edgeData.w);//.mul(sampleScale);
                    edgeNormals[edgeCount] = new Vec4f(edgeData.x, edgeData.y, edgeData.z, 0);
                    edgeCount++;
                }
            }
            qef.qef_create_from_points(edgePositions, edgeNormals, edgeCount);
            leafQEFs[index] = qef;

            Vec4f normal = new Vec4f(0.f, 0.f, 0.f, 0.f);
            for (int i = 0; i < edgeCount; i++) {
                normal = normal.add(edgeNormals[i]);
                normal.w += 1.f;
            }

            Vec4f nor = normal.div(normal.w).normalize();
            normal.w = 0.f;

            vertexNormals[index] = nor;
        }
    }

    private void solveQEFs(int chunkSize, int voxelsPerChunk, Vec3i chunkMin, int from, int to,
                           QefSolver[] qefs, Vec4f[] solvedPositions){

        for (int index = from; index < to; index++) {
            int leafSize = (chunkSize / voxelsPerChunk);
            Vec4f solvedPos = qefs[index].solve();
            solvedPos = solvedPos.mul(leafSize).add(chunkMin);
            solvedPositions[index] = solvedPos;
        }
    }

    private Map<Integer, Integer> compactVoxels(int[] voxelValid, int[] voxelEdgeInfo, int[] voxelPositions, int[] voxelMaterials,
                                                int[] compactPositions, int[] compactEdgeInfo, int[] compactMaterials, int numVertices){
        int current = 0;
        Map<Integer, Integer> octreeNodes = new HashMap<>(numVertices);
        for (int i = 0; i < voxelPositions.length; i++) {
            if (voxelValid[i]==1) {
                octreeNodes.put(voxelPositions[i], current);
                compactPositions[current] = voxelPositions[i];
                compactEdgeInfo[current] = voxelEdgeInfo[i];
                compactMaterials[current] = voxelMaterials[i];
                ++current;
            }
        }
        return octreeNodes;
    }

    private final Vec3i[][] EDGE_NODE_OFFSETS = {
            { new Vec3i(0, 0, 0), new Vec3i(0, 0, 1), new Vec3i(0, 1, 0), new Vec3i(0, 1, 1) },
            { new Vec3i(0, 0, 0), new Vec3i(1, 0, 0), new Vec3i(0, 0, 1), new Vec3i(1, 0, 1) },
            { new Vec3i(0, 0, 0), new Vec3i(0, 1, 0), new Vec3i(1, 0, 0), new Vec3i(1, 1, 0) },
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

    private int compactMeshTriangles(int[] trianglesValid, int[] meshIndexBuffer, int[] compactMeshIndexBuffer) {
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
        return current;
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

    private int findSeamNodes(int[] nodeCodes, int[] isSeamNode, int from, int to) {
        int res = 0;
        for (int index = from; index < to; index++) {
            int code = nodeCodes[index];
            Vec3i position = LinearOctreeTest.positionForCode(code);
            boolean xSeam = position.x == 0 || position.x == (meshGen.getVoxelsPerChunk() - 1);
            boolean ySeam = position.y == 0 || position.y == (meshGen.getVoxelsPerChunk() - 1);
            boolean zSeam = position.z == 0 || position.z == (meshGen.getVoxelsPerChunk() - 1);
            boolean isSeam = xSeam | ySeam | zSeam;
            if(isSeam) {
                ++res;
            }
            isSeamNode[index] = xSeam | ySeam | zSeam ? 1 : 0;
        }
        return res;
    }

    private void extractNodeInfo(int[] isSeamNode, Vec3f color,
                                 int leafSize, Vec3i chunkMin, int from, int to,
                                 int[] octreeCodes, int[] octreeMaterials, Vec4f[] octreePositions, Vec4f[] octreeNormals,
                                 List<PointerBasedOctreeNode> seamNodes) {
        for (int index = from; index < to; index++) {
            if (isSeamNode==null || isSeamNode[index]==1) {
                PointerBasedOctreeNode node = new PointerBasedOctreeNode();
                node.min = LinearOctreeTest.positionForCode(octreeCodes[index]).mul(leafSize).add(chunkMin);
                node.size = leafSize;
                node.Type = OctreeNodeType.Node_Leaf;
                OctreeDrawInfo drawInfo = new OctreeDrawInfo();
                drawInfo.position = octreePositions[index].getVec3f();
                drawInfo.color = color;
                drawInfo.averageNormal = octreeNormals[index].getVec3f();
                drawInfo.corners = octreeMaterials[index];
                node.drawInfo = drawInfo;
                seamNodes.add(node);
            }
        }
    }
}
