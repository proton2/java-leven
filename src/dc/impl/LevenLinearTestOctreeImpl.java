package dc.impl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
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
    Nick Gildea Leven OpenCL kernels Dual contouring implementation translated to java calling OpenCL kernels
    Some holes in seams is not fixed.
    The first raw version will still improve.
 */

public class LevenLinearTestOctreeImpl extends AbstractDualContouring implements VoxelOctree {
    private final KernelsHolder kernels;

    public LevenLinearTestOctreeImpl(KernelsHolder kernels, MeshGenerationContext meshGenerationContext) {
        super(meshGenerationContext);
        this.kernels = kernels;
    }

    @Override
    public boolean createLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                        float[] densityField,
                                        List<PointerBasedOctreeNode> seamNodes, MeshBuffer buffer, GPUDensityField field)
    {
        field.setMin(chunkMin);
        field.setSize(chunkSize);
        ComputeContext ctx = OCLUtils.getOpenCLContext();
        OpenCLCalculateMaterialsService calculateMaterialsService = new OpenCLCalculateMaterialsService(ctx, meshGen.getFieldSize(), meshGen, field);
        calculateMaterialsService.run(kernels);

        ScanOpenCLService scanService = new ScanOpenCLService(ctx, kernels.getKernel(KernelNames.SCAN));
        FindDefaultEdgesOpenCLService findDefEdges = new FindDefaultEdgesOpenCLService(ctx, meshGen, field, scanService);
        int compactEdgesSize = findDefEdges.findFieldEdgesKernel(kernels, meshGen.getHermiteIndexSize(), null, null);
        if(compactEdgesSize<=0){
            return false;
        }
        findDefEdges.compactEdgeKernel(kernels, field);
        findDefEdges.FindEdgeIntersectionInfoKernel(kernels, null);

        GpuOctree gpuOctree = new GpuOctree();
        ConstructOctreeFromFieldService constructOctreeFromFieldService = new ConstructOctreeFromFieldService(ctx, meshGen, field, gpuOctree, scanService);
        int octreeNumNodes = constructOctreeFromFieldService.findActiveVoxelsKernel(kernels, null, null, null, null);
        if (octreeNumNodes<=0){
            return false;
        }

        int[] d_nodeCodes = new int[octreeNumNodes];
        int[] d_nodeMaterials = new int[octreeNumNodes];
        constructOctreeFromFieldService.compactVoxelsKernel(kernels, d_nodeCodes, d_nodeMaterials);

        Vec4f[] d_vertexNormals = new Vec4f[octreeNumNodes];
        constructOctreeFromFieldService.createLeafNodesKernel(kernels, null, d_vertexNormals);
        Vec4f[] d_vertexPositions = new Vec4f[octreeNumNodes];
        constructOctreeFromFieldService.solveQefKernel(kernels, d_vertexPositions);

        //////////////////////////////
        MeshBufferGPU meshBufferGPU = new MeshBufferGPU();
        GenerateMeshFromOctreeService generateMeshFromOctreeService = new GenerateMeshFromOctreeService(ctx,
                meshGen, scanService, gpuOctree, meshBufferGPU);

        int[] d_indexBuffer = new int[octreeNumNodes * 6 * 3];
        int[] d_trianglesValid = new int[octreeNumNodes * 3];

        int numTriangles = generateMeshFromOctreeService.generateMeshKernel(kernels, d_indexBuffer, d_trianglesValid);
        if(numTriangles<=0){
            return false;
        }

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

        findDefEdges.destroy();
        calculateMaterialsService.destroy();
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

    void createLeafNodes(int chunkSize, Vec3i chunkMin, int from, int to, int sampleScale, int[] voxelPositions,
                         int[] voxelEdgeInfo, Vec4f[] edgeDataTable,
                         QefSolver[] leafQEFs, Map<Integer, Integer> nodes,
                         Vec4f[] vertexNormals)
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

    private void solveQEFs(MeshGenerationContext meshGen, Vec3i chunkMin, int from, int to, QefSolver[] qefs, Vec4f[] solvedPositions){
        for (int index = from; index < to; index++) {
            Vec4f solvedPos = qefs[index].solve();
            solvedPos = solvedPos.mul(meshGen.leafSizeScale).add(chunkMin);
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
