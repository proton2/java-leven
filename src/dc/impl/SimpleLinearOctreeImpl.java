package dc.impl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.utils.BufferUtil;
import core.utils.Constants;
import dc.*;
import dc.entities.MeshBuffer;
import dc.entities.MeshVertex;
import dc.solver.LevenQefSolver;
import dc.solver.QEFData;
import dc.utils.VoxelHelperUtils;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;

import static dc.utils.SimplexNoise.getNoise;
import static java.lang.Math.max;

/*
    Simple Linear octree dual contouring implementation, in series steps to calculate leafs data
    Holes in seams are fixed.
 */

public class SimpleLinearOctreeImpl extends AbstractDualContouring implements VoxelOctree {

    private final ExecutorService service;
    int availableProcessors;

    public SimpleLinearOctreeImpl(MeshGenerationContext meshGenerationContext) {
        super(meshGenerationContext);
        availableProcessors = max(1, Runtime.getRuntime().availableProcessors() / 2);
        service = Executors.newFixedThreadPool(availableProcessors, new ThreadFactory() {
            private final AtomicInteger count = new AtomicInteger();
            @Override
            public Thread newThread(Runnable r) {
                Thread thread = new Thread(r);
                thread.setName("SimpleLinearOctree " + count.getAndIncrement());
                return thread;
            }
        });
    }

    @Override
    public boolean createLeafVoxelNodes(int chunkSize, Vec3i chunkMin, List<OctreeNode> seamNodes, MeshBuffer buffer) {
        // usyal in serial calculating leaf nodes data. More slowly.
        try {
            return createLeafVoxelNodesTraditionalConcurrent(chunkSize, chunkMin, meshGen.getVoxelsPerChunk(), seamNodes, buffer);
            //return createLeafVoxelNodesM(chunkSize, chunkMin, meshGen.getVoxelsPerChunk(), seamNodes, buffer);
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
    }

    public boolean createLeafVoxelNodesTraditionalConcurrent(int chunkSize, Vec3i chunkMin, int voxelsPerChunk,
                                                   List<OctreeNode> seamNodes, MeshBuffer buffer) throws Exception {
        ArrayList<LinearLeafHolder> listHolders = new ArrayList<>();
        List<Callable<List<LinearLeafHolder>>> tasks = new ArrayList<>();

        int size = voxelsPerChunk*voxelsPerChunk*voxelsPerChunk;
        int threadBound = (size) / availableProcessors;

        for (int i=0; i<availableProcessors; i++){
            int finalI = i;
            Callable<List<LinearLeafHolder>> task = () -> {
                int from = finalI * threadBound;
                int to = from + threadBound;
                List<LinearLeafHolder> listHolder = new ArrayList<>();
                for (int it = from; it < to; it++) {
                    int indexShift = VoxelHelperUtils.log2(voxelsPerChunk); // max octree depth
                    int x = (it >> (indexShift * 0)) & voxelsPerChunk - 1;
                    int y = (it >> (indexShift * 1)) & voxelsPerChunk - 1;
                    int z = (it >> (indexShift * 2)) & voxelsPerChunk - 1;
                    Vec3i pos = new Vec3i(x, y, z);
                    LinearLeafHolder linearLeafHolder = constructLeaf(pos, chunkMin, chunkSize);
                    if(linearLeafHolder!=null){
                        listHolder.add(linearLeafHolder);
                    }
                }
                return listHolder;
            };
            tasks.add(task);
        }

        List<Future<List<LinearLeafHolder>>> futures = service.invokeAll(tasks);
        for (Future<List<LinearLeafHolder>> future : futures){
            listHolders.addAll(future.get());
        }

        if(listHolders.size()==0){
            return false;
        }
        Map<Integer, Integer> octreeNodes = new HashMap<>();
        Map<Vec3i, OctreeNode> seamNodesMap = new HashMap<>();
        for(int i=0; i<listHolders.size(); i++){
            octreeNodes.put(listHolders.get(i).encodedVoxelPosition, i);
            if (listHolders.get(i).isSeam) {
                OctreeNode seamNode = extractSeamNode(listHolders.get(i), chunkMin, chunkSize / voxelsPerChunk);
                if(seamNode.size > meshGen.leafSizeScale) {
                    seamNodesMap.put(seamNode.min, seamNode);
                }
                seamNodes.add(seamNode);
            }
        }

        List<OctreeNode> addedNodes = findAndCreateBorderNodes(seamNodesMap);
        seamNodes.addAll(addedNodes);

        int[] d_nodeCodes = listHolders.stream().mapToInt(e->e.encodedVoxelPosition).toArray();
        int[] d_nodeMaterials = listHolders.stream().mapToInt(e->e.materialIndex).toArray();
        Vec4f[] d_vertexPositions = listHolders.stream().map(e->e.solvedPosition).toArray(Vec4f[]::new);
        Vec4f[] d_vertexNormals = listHolders.stream().map(e->e.averageNormal).toArray(Vec4f[]::new);

        processDc(chunkSize, chunkMin, voxelsPerChunk, seamNodes, buffer, octreeNodes,
                d_nodeCodes, d_nodeMaterials, d_vertexPositions, d_vertexNormals);
        return true;
    }

    private Vec3i getRight(OctreeNode node, boolean nodeOnX, boolean nodeOnY, boolean nodeOnZ){
        Vec3i right = new Vec3i();
        if (nodeOnX && node.nodeNum.y < meshGen.getVoxelsPerChunk() - 1) {
            right.add(0, node.size, 0);
        }
        if (nodeOnY && node.nodeNum.z < meshGen.getVoxelsPerChunk() - 1) {
            right.add(0, 0, node.size);
        }
        if (nodeOnZ && node.nodeNum.x < meshGen.getVoxelsPerChunk() - 1) {
            right.add(node.size, 0, 0);
        }
        return node.min.add(right);
    }

    private Vec3i getLeft(OctreeNode node, boolean nodeOnX, boolean nodeOnY, boolean nodeOnZ){
        Vec3i left = new Vec3i();
        if (nodeOnX && node.nodeNum.y > 0) {
            left.add(0, -node.size, 0);
        }
        if (nodeOnY && node.nodeNum.z > 0) {
            left.add(0, 0, -node.size);
        }
        if (nodeOnZ && node.nodeNum.x > 0) {
            left.add(-node.size, 0, 0);
        }
        return node.min.add(left);
    }

    private Vec3i getTop(OctreeNode node, boolean nodeOnX, boolean nodeOnY, boolean nodeOnZ){
        Vec3i top = new Vec3i();
        if (nodeOnX && node.nodeNum.z < meshGen.getVoxelsPerChunk() - 1) {
            top.add(0, 0, node.size);
        }
        if (nodeOnY && node.nodeNum.x < meshGen.getVoxelsPerChunk() - 1) {
            top.add(node.size, 0, 0);
        }
        if (nodeOnZ && node.nodeNum.y < meshGen.getVoxelsPerChunk() - 1) {
            top.add(0, node.size, 0);
        }
        return node.min.add(top);
    }

    private Vec3i getBottom(OctreeNode node, boolean nodeOnX, boolean nodeOnY, boolean nodeOnZ){
        Vec3i bottom = new Vec3i();
        if (nodeOnX && node.nodeNum.z > 0) {
            bottom.add(0, 0, - node.size);
        }
        if (nodeOnY && node.nodeNum.x > 0) {
            bottom.add(- node.size, 0, 0);
        }
        if (nodeOnZ && node.nodeNum.y > 0) {
            bottom.add(0, - node.size, 0);
        }
        return node.min.add(bottom);
    }

    private List<OctreeNode> findAndCreateBorderNodes(Map<Vec3i, OctreeNode> seamNodesMap) {
        List<OctreeNode> addedNodes = new ArrayList<>();
        for(Map.Entry<Vec3i, OctreeNode>set : seamNodesMap.entrySet()){
            OctreeNode node = set.getValue();
            boolean nodeOnX = node.nodeNum.x==0 || node.nodeNum.x==meshGen.getVoxelsPerChunk()-1;
            boolean nodeOnY = node.nodeNum.y==0 || node.nodeNum.y==meshGen.getVoxelsPerChunk()-1;
            boolean nodeOnZ = node.nodeNum.z==0 || node.nodeNum.z==meshGen.getVoxelsPerChunk()-1;

            Vec3i right = getRight(node, nodeOnX, nodeOnY, nodeOnZ);
            if (seamNodesMap.get(right) == null) {
                OctreeNode rightNode = createBorderNode(right, node.size);
                if (rightNode != null) {
                    addedNodes.add(rightNode);
                }
            }

            Vec3i left = getLeft(node, nodeOnX, nodeOnY, nodeOnZ);
            if (seamNodesMap.get(left) == null) {
                OctreeNode leftNode = createBorderNode(left, node.size);
                if (leftNode != null) {
                    addedNodes.add(leftNode);
                }
            }

            Vec3i top = getTop(node, nodeOnX, nodeOnY, nodeOnZ);
            if (seamNodesMap.get(top) == null) {
                OctreeNode topNode = createBorderNode(top, node.size);
                if (topNode != null) {
                    addedNodes.add(topNode);
                }
            }

            Vec3i bottom = getBottom(node, nodeOnX, nodeOnY, nodeOnZ);
            if (seamNodesMap.get(bottom) == null) {
                OctreeNode bottomNode = createBorderNode(bottom, node.size);
                if (bottomNode != null) {
                    addedNodes.add(bottomNode);
                }
            }
        }
        return addedNodes;
    }

    private OctreeNode createBorderNode(Vec3i min, int size) {
        int corners = 0;
        for (int i = 0; i < 8; i++) {
            Vec3i cornerPos = min.add(CHILD_MIN_OFFSETS[i].mul(size));
            float density = getNoise(cornerPos);
            int material = density < 0.f ? meshGen.MATERIAL_SOLID : meshGen.MATERIAL_AIR;
            corners |= (material << i);
        }

        for (int i = 0; i < 12; i++) {
            // node size at LOD 0 = 1, LOD 1 = 2, LOD 2 = 4, LOD 3 = 8
            int x = min.x + (BORDER_EDGE_OFFSETS[i].x) * size / 2;
            int y = min.y + (BORDER_EDGE_OFFSETS[i].y) * size / 2;
            int z = min.z + (BORDER_EDGE_OFFSETS[i].z) * size / 2;

            Vec4f nodePos = new Vec4f(x,y,z);
            float density = getNoise(nodePos);
            if ((density < 0 && corners == 0) || (density >= 0 && corners == 255)) {
                PointerBasedOctreeNode node = new PointerBasedOctreeNode(min, size, OctreeNodeType.Node_Leaf);
                OctreeDrawInfo drawInfo = new OctreeDrawInfo();
                drawInfo.position = nodePos.getVec3f();
                drawInfo.color = Constants.Yellow;
                drawInfo.averageNormal = CalculateSurfaceNormal(nodePos).getVec3f();
                node.corners = corners;
                node.drawInfo = drawInfo;
                return node;
            }
        }
        return null;
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
    }

    private static class LinearLeafHolder{
        int materialIndex;
        int encodedVoxelPosition;
        Vec4f solvedPosition;
        Vec4f averageNormal;
        boolean isSeam;
    }

    private LinearLeafHolder constructLeaf(Vec3i pos, Vec3i chunkMin, int chunkSize) {
        int leafSize = (chunkSize / meshGen.getVoxelsPerChunk());
        Vec3i leafMin = pos.mul(leafSize).add(chunkMin);
        int[] cornerMaterials = new int[8];
        int corners = 0;
        for (int i = 0; i < 8; i++) {
            Vec3i cornerPos = leafMin.add(CHILD_MIN_OFFSETS[i].mul(leafSize));
            float density = getNoise(cornerPos);
            int material = density < 0.f ? meshGen.MATERIAL_SOLID : meshGen.MATERIAL_AIR;
            cornerMaterials[i] = material;
            corners |= (material << i);
        }
        if (corners == 0 || corners == 255) {
            // to avoid holes in seams between chunks with different resolution we creating some other nodes only in seams
            //https://www.reddit.com/r/VoxelGameDev/comments/6kn8ph/dual_contouring_seam_stitching_problem/
            //return createSeamBoundNode(pos, leafSize, leafMin, cornerMaterials, corners);
            return null;
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
        leafHolder.encodedVoxelPosition = codeForPosition(pos, meshGen.MAX_OCTREE_DEPTH);
        //leafHolder.voxelEdgeInfo = edgeList;
        leafHolder.averageNormal = averageNormal.div((float)edgeCount).normalize();

        boolean xSeam = pos.x == 0 || pos.x == (meshGen.getVoxelsPerChunk() - 1);
        boolean ySeam = pos.y == 0 || pos.y == (meshGen.getVoxelsPerChunk() - 1);
        boolean zSeam = pos.z == 0 || pos.z == (meshGen.getVoxelsPerChunk() - 1);
        leafHolder.isSeam = xSeam | ySeam | zSeam;
        return leafHolder;
    }

    private LinearLeafHolder createSeamBoundNode(Vec3i pos, int leafSize, Vec3i leafMin, int[] cornerMaterials, int corners) {
        Vec4f nodePos = tryToCreateBoundSeamPseudoNode(leafMin, leafSize, pos, corners, meshGen.leafSizeScale);
        if(nodePos==null){
            return null;
        } else {
            LinearLeafHolder leafHolder = new LinearLeafHolder();
            leafHolder.solvedPosition = nodePos;
            int materialIndex = findDominantMaterial(cornerMaterials);
            leafHolder.materialIndex = (materialIndex << 8) | corners;
            leafHolder.encodedVoxelPosition = codeForPosition(pos, meshGen.MAX_OCTREE_DEPTH);
            //leafHolder.voxelEdgeInfo = corners;//edgeList;
            leafHolder.averageNormal = CalculateSurfaceNormal(nodePos);
            leafHolder.isSeam = true;
            return leafHolder;
        }
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

    private OctreeNode extractSeamNode(LinearLeafHolder leafHolder, Vec3i chunkMin, int leafSize){
        Vec3i min = positionForCode(leafHolder.encodedVoxelPosition).mul(leafSize).add(chunkMin);
        PointerBasedOctreeNode node = new PointerBasedOctreeNode(min, leafSize, OctreeNodeType.Node_Leaf);
        OctreeDrawInfo drawInfo = new OctreeDrawInfo();
        drawInfo.position = leafHolder.solvedPosition.getVec3f();
        drawInfo.color = Constants.Yellow;
        drawInfo.averageNormal = leafHolder.averageNormal.getVec3f();
        node.corners = leafHolder.materialIndex;
        node.drawInfo = drawInfo;
        node.nodeNum = positionForCode(leafHolder.encodedVoxelPosition);
        return node;
    }

    //    public boolean createLeafVoxelNodesM(int chunkSize, Vec3i chunkMin, int voxelsPerChunk,
//                                                   List<OctreeNode> seamNodes, MeshBuffer buffer) throws Exception {
//        int size = voxelsPerChunk*voxelsPerChunk*voxelsPerChunk;
//
//        int availableProcessors = max(1, Runtime.getRuntime().availableProcessors() / 2);
//        ExecutorService service = Executors.newFixedThreadPool(availableProcessors);
//        int threadBound = (size) / availableProcessors;
//
//        int[] d_nodeCodes = new int[size];
//        int[] d_nodeMaterials = new int[size];
//        Vec4f[] d_vertexPositions = new Vec4f[size];
//        Vec4f[] d_vertexNormals = new Vec4f[size];
//
//        List<Callable<Integer>> tasks = new ArrayList<>();
//        for (int i=0; i<availableProcessors; i++){
//            int finalI = i;
//            Callable<Integer> task = () -> {
//                int from = finalI * threadBound;
//                int to = from + threadBound;
//                int current = from;
//                int count=0;
//                for (int it = from; it < to; it++) {
//                    int indexShift = VoxelHelperUtils.log2(voxelsPerChunk); // max octree depth
//                    int x = (it >> (indexShift * 0)) & voxelsPerChunk - 1;
//                    int y = (it >> (indexShift * 1)) & voxelsPerChunk - 1;
//                    int z = (it >> (indexShift * 2)) & voxelsPerChunk - 1;
//                    boolean res = constructLeafM(new Vec3i(x, y, z), chunkMin, chunkSize, current,
//                            d_nodeCodes, d_nodeMaterials, d_vertexPositions, d_vertexNormals);
//                    if(res){
//                        ++current;
//                        ++count;
//                    }
//                }
//                return count;
//            };
//            tasks.add(task);
//        }
//
//        List<Future<Integer>> futures = service.invokeAll(tasks);
//        service.shutdown();
//        int compactSize = 0;
//        for (Future<Integer> future : futures){
//            compactSize += future.get();
//        }
//
//        if(compactSize==0){
//            return false;
//        }
//
//        int[] nodeCodes = new int[compactSize];
//        int[] nodeMaterials = new int[compactSize];
//        Vec4f[] vertexPositions = new Vec4f[compactSize];
//        Vec4f[] vertexNormals = new Vec4f[compactSize];
//
//        Map<Integer, Integer> octreeNodes = compactVoxels(d_nodeCodes, d_nodeMaterials, d_vertexPositions, d_vertexNormals,
//                nodeCodes, nodeMaterials, vertexPositions, vertexNormals);
//
//        processDc(chunkSize, chunkMin, voxelsPerChunk, seamNodes, buffer, octreeNodes,
//                nodeCodes, nodeMaterials, vertexPositions, vertexNormals);
//        return true;
//    }
//
//    private  Map<Integer, Integer> compactVoxels(int[] d_nodeCodes, int[] d_nodeMaterials,  Vec4f[] d_vertexPositions, Vec4f[] d_vertexNormals,
//                               int[] nodeCodes, int[] nodeMaterials, Vec4f[] vertexPositions, Vec4f[] vertexNormals){
//        Map<Integer, Integer> octreeNodes = new HashMap<>(nodeCodes.length);
//        int current = 0;
//        for (int i = 0; i < d_nodeCodes.length; i++) {
//            if (d_vertexPositions[i]!=null) {
//                octreeNodes.put(d_nodeCodes[i], current);
//                nodeCodes[current] = d_nodeCodes[i];
//                nodeMaterials[current] = d_nodeMaterials[i];
//                vertexPositions[current] = d_vertexPositions[i];
//                vertexNormals[current] = d_vertexNormals[i];
//                ++current;
//            }
//        }
//        return octreeNodes;
//    }

    //    private boolean constructLeafM(Vec3i pos, Vec3i chunkMin, int chunkSize, int current,
//                                           int[] d_nodeCodes, int[] d_nodeMaterials, Vec4f[] d_vertexPositions, Vec4f[] d_vertexNormals)
//    {
//        int leafSize = (chunkSize / meshGen.getVoxelsPerChunk());
//        Vec3i leafMin = pos.mul(leafSize).add(chunkMin);
//        int[] cornerMaterials = new int[8];
//        int corners = 0;
//        for (int i = 0; i < 8; i++) {
//            Vec3i cornerPos = leafMin.add(CHILD_MIN_OFFSETS[i].mul(leafSize));
//            float density = getNoise(cornerPos);
//            int material = density < 0.f ? meshGen.MATERIAL_SOLID : meshGen.MATERIAL_AIR;
//            cornerMaterials[i] = material;
//            corners |= (material << i);
//        }
//        if (corners == 0 || corners == 255) {
//            // to avoid holes in seams between chunks with different resolution we creating some other nodes only in seams
//            //https://www.reddit.com/r/VoxelGameDev/comments/6kn8ph/dual_contouring_seam_stitching_problem/
//            return getLinearLeafHolder(pos, leafSize, leafMin, cornerMaterials, corners, current,
//                    d_nodeCodes, d_nodeMaterials, d_vertexPositions, d_vertexNormals);
//        }
//        int edgeList = 0;
//
//        // otherwise the voxel contains the surface, so find the edge intersections
//        int MAX_CROSSINGS = 6;
//        int edgeCount = 0;
//        Vec4f averageNormal = new Vec4f();
//        Vec4f[] edgePositions = new Vec4f[12];
//        Vec4f[] edgeNormals = new Vec4f[12];
//        QEFData qef = new QEFData(new LevenQefSolver());
//        for (int i = 0; i < 12 && edgeCount < MAX_CROSSINGS; i++) {
//            int c1 = edgevmap[i][0];
//            int c2 = edgevmap[i][1];
//            int m1 = (corners >> c1) & 1;
//            int m2 = (corners >> c2) & 1;
//            int signChange = m1 != m2 ? 1 : 0;
//            edgeList |= (signChange << i);
//            if ((m1 == meshGen.MATERIAL_AIR && m2 == meshGen.MATERIAL_AIR) || (m1 == meshGen.MATERIAL_SOLID && m2 == meshGen.MATERIAL_SOLID)) {
//                continue; // no zero crossing on this edge
//            }
//            Vec3f p1 = leafMin.add(CHILD_MIN_OFFSETS[c1].mul(leafSize)).toVec3f();
//            Vec3f p2 = leafMin.add(CHILD_MIN_OFFSETS[c2].mul(leafSize)).toVec3f();
//            Vec4f p = ApproximateZeroCrossingPosition(p1, p2);
//            Vec4f n = CalculateSurfaceNormal(p);
//            edgePositions[edgeCount] = p;
//            edgeNormals[edgeCount] = n;
//            averageNormal = averageNormal.add(n);
//            edgeCount++;
//        }
//        qef.qef_create_from_points(edgePositions, edgeNormals, edgeCount);
//
//        d_vertexPositions[current] = qef.solve();
//        int materialIndex = findDominantMaterial(cornerMaterials);
//        d_nodeMaterials[current] = (materialIndex << 8) | corners;
//        int encodedVoxelPos = codeForPosition(pos, meshGen.MAX_OCTREE_DEPTH);
//        d_nodeCodes[current] = encodedVoxelPos;
//        d_vertexNormals[current] = averageNormal.div((float)edgeCount).normalize();
//        return true;
//    }

//    private boolean getLinearLeafHolder(Vec3i pos, int leafSize, Vec3i leafMin, int[] cornerMaterials, int corners, int current,
//                                        int[] d_nodeCodes, int[] d_nodeMaterials, Vec4f[] d_vertexPositions, Vec4f[] d_vertexNormals) {
//        Vec4f nodePos = tryToCreateBoundSeamPseudoNode(leafMin, leafSize, pos, corners, meshGen.leafSizeScale);
//        if(nodePos==null){
//            return false;
//        } else {
//            d_vertexPositions[current] = nodePos;
//            int materialIndex = findDominantMaterial(cornerMaterials);
//            d_nodeMaterials[current] = (materialIndex << 8) | corners;
//            int encodedVoxelPos = codeForPosition(pos, meshGen.MAX_OCTREE_DEPTH);
//            d_nodeCodes[current] = encodedVoxelPos;
//            d_vertexNormals[current] = CalculateSurfaceNormal(nodePos);
//            return true;
//        }
//    }
//
//    public boolean createLeafVoxelNodesTraditional(int chunkSize, Vec3i chunkMin, int voxelsPerChunk,
//                                                   List<OctreeNode> seamNodes, MeshBuffer buffer)
//    {
//        ArrayList<LinearLeafHolder> listHolder = new ArrayList<>();
//        Map<Integer, Integer> octreeNodes = new HashMap<>();
//
//        int current = 0;
//        for (int i = 0; i < voxelsPerChunk*voxelsPerChunk*voxelsPerChunk; i++) {
//            int indexShift = VoxelHelperUtils.log2(voxelsPerChunk); // max octree depth
//            int x = (i >> (indexShift * 0)) & voxelsPerChunk - 1;
//            int y = (i >> (indexShift * 1)) & voxelsPerChunk - 1;
//            int z = (i >> (indexShift * 2)) & voxelsPerChunk - 1;
//            Vec3i pos = new Vec3i(x, y, z);
//            LinearLeafHolder linearLeafHolder = constructLeaf(pos, chunkMin, chunkSize);
//            if(linearLeafHolder!=null){
//                listHolder.add(linearLeafHolder);
//                octreeNodes.put(linearLeafHolder.encodedVoxelPosition, current);
//                ++current;
//            }
//        }
//        if(listHolder.size()==0){
//            return false;
//        }
//
//        int[] d_nodeCodes = listHolder.stream().mapToInt(e->e.encodedVoxelPosition).toArray();
//        int[] d_nodeMaterials = listHolder.stream().mapToInt(e->e.materialIndex).toArray();
//        Vec4f[] d_vertexPositions = listHolder.stream().map(e->e.solvedPosition).toArray(Vec4f[]::new);
//        Vec4f[] d_vertexNormals = listHolder.stream().map(e->e.averageNormal).toArray(Vec4f[]::new);
//
//        processDc(chunkSize, chunkMin, voxelsPerChunk, seamNodes, buffer, octreeNodes,
//                d_nodeCodes, d_nodeMaterials, d_vertexPositions, d_vertexNormals);
//        return true;
//    }
}
