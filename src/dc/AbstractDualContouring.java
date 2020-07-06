package dc;

import core.math.Vec3i;
import core.math.Vec4f;
import core.math.Vec4i;
import core.utils.BufferUtil;
import dc.entities.MeshBuffer;
import dc.entities.MeshVertex;
import dc.utils.Density;

import java.util.*;

import static dc.ChunkOctree.*;
import static dc.OctreeNodeType.Node_Internal;
import static dc.OctreeNodeType.Node_Leaf;
import static dc.VoxelOctree.MATERIAL_AIR;
import static dc.VoxelOctree.edgevmap;
import static dc.impl.LevenLinearOctreeImpl.fieldSize;

public abstract class AbstractDualContouring implements DualContouring{

    private List<PointerBasedOctreeNode> constructParents(List<PointerBasedOctreeNode> nodes, Vec3i rootMin, int parentSize, int chunkSize) {
        Map<Vec3i, PointerBasedOctreeNode> parentsHash = new HashMap<>();
        for (PointerBasedOctreeNode node : nodes) {
            Vec3i localPos = node.min.sub(rootMin);
            Vec3i parentPos = node.min.sub(new Vec3i(localPos.x % parentSize, localPos.y % parentSize, localPos.z % parentSize));
            PointerBasedOctreeNode parent = parentsHash.get(parentPos);
            if (parent == null) {
                parent = new PointerBasedOctreeNode();
                parent.min = parentPos;
                parent.size = parentSize;
                parent.Type = OctreeNodeType.Node_Internal;
                parent.chunkSize = chunkSize;
                parentsHash.put(parentPos, parent);
            }
            for (int j = 0; j < 8; j++) {
                //Vec3i childMin = parentPos.add(Octree.CHILD_MIN_OFFSETS[j].mul(node.size));
                Vec3i childMin = parentPos.add(VoxelOctree.CHILD_MIN_OFFSETS[j].mul(parentSize / 2));
                if (childMin.equals(node.min)) {
                    parent.children[j] = node;
                    break;
                }
            }
        }
        return new ArrayList<>(parentsHash.values());
    }

    private PointerBasedOctreeNode constructTreeUpwards(List<PointerBasedOctreeNode> inputNodes, Vec3i rootMin, int rootNodeSize) {
        List<PointerBasedOctreeNode> sortedNodes = new ArrayList<>(inputNodes);
        sortedNodes.sort(Comparator.comparingInt((PointerBasedOctreeNode lhs) -> lhs.size));
        while (sortedNodes.get(0).size != sortedNodes.get(sortedNodes.size() - 1).size) {
            int iter = 0;
            int size = sortedNodes.get(iter).size;
            do {
                ++iter;
            } while (sortedNodes.get(iter).size == size);

            List<PointerBasedOctreeNode> newNodes = constructParents(sortedNodes.subList(0, iter), rootMin, size * 2, rootNodeSize);
            newNodes.addAll(sortedNodes.subList(iter, sortedNodes.size()));
            sortedNodes.clear();
            sortedNodes.addAll(newNodes);
            newNodes.clear();
        }

        int parentSize = (sortedNodes.get(0).size) * 2;
        while (parentSize <= rootNodeSize) {
            sortedNodes = constructParents(sortedNodes, rootMin, parentSize, rootNodeSize);
            parentSize *= 2;
        }
        if (sortedNodes.size()!=1){
            throw new IllegalStateException("Incorrect octree!");
        }
        if (!(rootMin.x==sortedNodes.get(0).min.x) || !(rootMin.y==sortedNodes.get(0).min.y)|| !(rootMin.z==sortedNodes.get(0).min.z)){
            throw new IllegalStateException("returned root not equal to input root!");
        }
//        int octreeCounts = VoxelHelperUtils.countLeafNodes(sortedNodes.get(0));
//        if (octreeCounts!=inputNodes.size()){
//            throw new IllegalStateException("Octree leafs is not equal to octree counts!");
//        }
        return sortedNodes.get(0);
    }

    private void ContourProcessEdge(PointerBasedOctreeNode[] node, int dir, List<Integer> indexBuffer)
    {
        int minSize = 1000000;		// arbitrary big number
        int minIndex = 0;
        int[] indices = { -1, -1, -1, -1 };
        boolean flip = false;
        boolean[] signChange = { false, false, false, false };

        for (int i = 0; i < 4; i++)
        {
            if (node[i].Type != Node_Internal)
            {
                int edge = processEdgeMask[dir][i];
                int c0 = edgevmap[edge][0];
                int c1 = edgevmap[edge][1];
                int m0 = (node[i].drawInfo.corners >> c0) & 1;
                int m1 = (node[i].drawInfo.corners >> c1) & 1;

                if (node[i].size < minSize)
                {
                    minSize = node[i].size;
                    minIndex = i;
                    flip = m1 != 1;
                }

                indices[i] = node[i].drawInfo.index;
                //signChange[i] = (m0 && !m1) || (!m0 && m1);
                signChange[i] =
                        (m0 == MATERIAL_AIR && m1 != MATERIAL_AIR) ||
                                (m0 != MATERIAL_AIR && m1 == MATERIAL_AIR);
            }
        }

        if (!signChange[minIndex]) {
            return;
        }

        if (!flip) {
            indexBuffer.add(indices[0]);
            indexBuffer.add(indices[1]);
            indexBuffer.add(indices[3]);

            indexBuffer.add(indices[0]);
            indexBuffer.add(indices[3]);
            indexBuffer.add(indices[2]);
        }
        else {
            indexBuffer.add(indices[0]);
            indexBuffer.add(indices[3]);
            indexBuffer.add(indices[1]);

            indexBuffer.add(indices[0]);
            indexBuffer.add(indices[2]);
            indexBuffer.add(indices[3]);
        }
    }

    private void ContourEdgeProc(PointerBasedOctreeNode[] node, int dir, List<Integer> buffer, boolean isSeam) {
        if (node[0] == null || node[1] == null || node[2] == null || node[3] == null) {
            return;
        }

        // bit of a hack but it works: prevent overlapping seams by only processing edges that stradle multiple chunks
        if(isSeam) {
            Set<Vec3i> chunks = new HashSet<>();
            for (int i = 0; i < 4; i++) {
                chunks.add(ChunkOctree.chunkMinForPosition(node[i]));
            }
            if (chunks.size() == 1)
                return;
        }

        boolean[] isBranch = {
                node[0].Type == Node_Internal,
                node[1].Type == Node_Internal,
                node[2].Type == Node_Internal,
                node[3].Type == Node_Internal,
        };

        if (!isBranch[0] && !isBranch[1] && !isBranch[2] && !isBranch[3]) {
            ContourProcessEdge(node, dir, buffer);
        }
        else {
            for (int i = 0; i < 2; i++) {
                PointerBasedOctreeNode[] edgeNodes = new PointerBasedOctreeNode[4];
                int[] c = {
                        edgeProcEdgeMask[dir][i][0],
                        edgeProcEdgeMask[dir][i][1],
                        edgeProcEdgeMask[dir][i][2],
                        edgeProcEdgeMask[dir][i][3],
                };

                for (int j = 0; j < 4; j++) {
                    if (!isBranch[j]) {
                        edgeNodes[j] = node[j];
                    }
                    else {
                        edgeNodes[j] = node[j].children[c[j]];
                    }
                }

                ContourEdgeProc(edgeNodes, edgeProcEdgeMask[dir][i][4], buffer, isSeam);
            }
        }
    }

    private void ContourFaceProc(PointerBasedOctreeNode[] node, int dir, List<Integer> buffer, boolean isSeam) {
        if (node[0] == null || node[1] == null) {
            return;
        }

        // bit of a hack but it works: prevent overlapping seams by only processing edges that stradle multiple chunks
        if (isSeam && ChunkOctree.chunkMinForPosition(node[0]).equals(ChunkOctree.chunkMinForPosition(node[1]))) {
            return;
        }

        boolean[] isBranch = {
                node[0].Type == Node_Internal,
                node[1].Type == Node_Internal,
        };

        if (isBranch[0] || isBranch[1]) {
            for (int i = 0; i < 4; i++) {
                PointerBasedOctreeNode[] faceNodes = new PointerBasedOctreeNode[2];
                int[] c = {
                        faceProcFaceMask[dir][i][0], faceProcFaceMask[dir][i][1],
                };

                for (int j = 0; j < 2; j++) {
                    if (!isBranch[j]) {
                        faceNodes[j] = node[j];
                    }
                    else {
                        faceNodes[j] = node[j].children[c[j]];
                    }
                }
                ContourFaceProc(faceNodes, faceProcFaceMask[dir][i][2], buffer, isSeam);
            }

            int[][] orders = {
                    { 0, 0, 1, 1 },
                    { 0, 1, 0, 1 },
            };

            for (int i = 0; i < 4; i++) {
                PointerBasedOctreeNode[] edgeNodes = new PointerBasedOctreeNode[4];
                int[] c = {
                        faceProcEdgeMask[dir][i][1],
                        faceProcEdgeMask[dir][i][2],
                        faceProcEdgeMask[dir][i][3],
                        faceProcEdgeMask[dir][i][4],
                };

                int[] order = orders[faceProcEdgeMask[dir][i][0]];
                for (int j = 0; j < 4; j++) {
                    if (!isBranch[order[j]]) {
                        edgeNodes[j] = node[order[j]];
                    }
                    else {
                        edgeNodes[j] = node[order[j]].children[c[j]];
                    }
                }

                ContourEdgeProc(edgeNodes, faceProcEdgeMask[dir][i][5], buffer, isSeam);
            }
        }
    }

    @Override
    public void ContourCellProc(PointerBasedOctreeNode node, List<Integer> buffer, boolean isSeam) {
        if (node == null || node.Type == Node_Leaf) {
            return;
        }

        for (int i = 0; i < 8; i++) {
            ContourCellProc(node.children[i], buffer, isSeam);
        }

        for (int i = 0; i < 12; i++) {
            PointerBasedOctreeNode[] faceNodes = new PointerBasedOctreeNode[2];
            int[] c = { cellProcFaceMask[i][0], cellProcFaceMask[i][1] };

            faceNodes[0] = node.children[c[0]];
            faceNodes[1] = node.children[c[1]];

            ContourFaceProc(faceNodes, cellProcFaceMask[i][2], buffer, isSeam);
        }

        for (int i = 0; i < 6; i++) {
            PointerBasedOctreeNode[] edgeNodes = new PointerBasedOctreeNode[4];
            int[] c = {cellProcEdgeMask[i][0], cellProcEdgeMask[i][1], cellProcEdgeMask[i][2], cellProcEdgeMask[i][3]};

            for (int j = 0; j < 4; j++) {
                edgeNodes[j] = node.children[c[j]];
            }

            ContourEdgeProc(edgeNodes, cellProcEdgeMask[i][4], buffer, isSeam);
        }
    }

    private void GenerateVertexIndices(PointerBasedOctreeNode node, List<MeshVertex> vertexBuffer) {
        if (node == null) {
            return;
        }
        if (node.Type != Node_Leaf) {
            for (int i = 0; i < 8; i++) {
                GenerateVertexIndices(node.children[i], vertexBuffer);
            }
        }
        if (node.Type != Node_Internal) {
            node.drawInfo.index = vertexBuffer.size();
            vertexBuffer.add(new MeshVertex(node.drawInfo.position, node.drawInfo.averageNormal, node.drawInfo.color));
        }
    }

    private void GenerateMeshFromOctree(PointerBasedOctreeNode node, boolean isSeam, MeshBuffer buffer) {
        if (node == null) {
            return;
        }
        List<MeshVertex> vertices = new ArrayList<>();
        List<Integer> indcies = new ArrayList<>();
        GenerateVertexIndices(node, vertices);
        ContourCellProc(node, indcies, isSeam);

        buffer.setVertices(BufferUtil.createDcFlippedBufferAOS(vertices));
        buffer.setIndicates(BufferUtil.createFlippedBuffer(indcies));
        buffer.setNumVertices(vertices.size());
        buffer.setNumIndicates(indcies.size());
        vertices.clear();
        indcies.clear();
    }

    public void processNodesToMesh(List<PointerBasedOctreeNode> seamNodes, Vec3i currNodeMin, int rootNodeSize, boolean isSeam, MeshBuffer meshBuffer){
        PointerBasedOctreeNode seamOctreeRoot = constructTreeUpwards(new ArrayList<>(seamNodes), currNodeMin, rootNodeSize);
        GenerateMeshFromOctree(seamOctreeRoot, isSeam, meshBuffer);
    }

    protected Vec3i decodeVoxelIndex(int index) {
        Vec3i p = new Vec4i(0);
        p.x = (index >> (VOXEL_INDEX_SHIFT * 0)) & VOXEL_INDEX_MASK;
        p.y = (index >> (VOXEL_INDEX_SHIFT * 1)) & VOXEL_INDEX_MASK;
        p.z = (index >> (VOXEL_INDEX_SHIFT * 2)) & VOXEL_INDEX_MASK;
        return p;
    }

    protected int encodeVoxelIndex(Vec3i pos) {
        int encoded = 0;
        encoded |= pos.x << (VOXEL_INDEX_SHIFT * 0);
        encoded |= pos.y << (VOXEL_INDEX_SHIFT * 1);
        encoded |= pos.z << (VOXEL_INDEX_SHIFT * 2);
        return encoded;
    }

    protected void inlineInsertionSwap8(int[] data) {
        int i, j;
        for (i = 1; i < 8; i++) {
            int tmp = data[i];
            for (j = i; j >= 1 && tmp < data[j-1]; j--) {
                data[j] = data[j-1];
            }
            data[j] = tmp;
        }
    }

    protected int findDominantMaterial(int[] mv) {
        int MATERIAL_NONE = 200;
        int MATERIAL_AIR  = 201;

        int[] data = { mv[0], mv[1], mv[2], mv[3], mv[4], mv[5], mv[6], mv[7] };
        inlineInsertionSwap8(data);

        int current = data[0];
        int count = 1;
        int maxCount = 0;
        int maxMaterial = 0;

        for (int i = 1; i < 8; i++) {
            int m = data[i];
            if (m == MATERIAL_AIR || m == MATERIAL_NONE) {
                continue;
            }

            if (current != m) {
                if (count > maxCount) {
                    maxCount = count;
                    maxMaterial = current;
                }
                current = m;
                count = 1;
            }
            else {
                count++;
            }
        }

        if (count > maxCount) {
            maxMaterial = current;
        }
        return maxMaterial;
    }

    private Vec3i[] EDGE_OFFSETS = {
            new Vec3i(1, 2, 0), new Vec3i(1, 0, 2),
            new Vec3i(2, 1, 0), new Vec3i(0, 1, 2),
            new Vec3i(2, 0, 1), new Vec3i(0, 2, 1),
            new Vec3i(1, 0, 0), new Vec3i(0, 1, 0), new Vec3i(0, 0, 1),
            new Vec3i(1, 2, 2), new Vec3i(2, 2, 1), new Vec3i(2, 1, 2)
    };

    private Vec3i getChunkBorder(Vec3i pos){
        Vec3i faces = new Vec3i(0,0,0);
        // checks which side this node is facing
        if (pos.x == 0)
            faces.x = -1;
        else if (pos.x == VOXELS_PER_CHUNK-1)
            faces.x = 1;

        if (pos.y == 0)
            faces.y = -1;
        else if (pos.y == VOXELS_PER_CHUNK-1)
            faces.y = 1;

        if (pos.z == 0)
            faces.z = -1;
        else if (pos.z == VOXELS_PER_CHUNK-1)
            faces.z = 1;
        return faces;
    }

    protected Vec4f tryToCreateBoundSeamPseudoNode(Vec3i leafMin, int leafSize, Vec3i pos, int corners,
                                                                    int nodeMinSize, float[] densityField) {
        Vec3i chunkBorders = getChunkBorder(pos);
        // if it is facing no border at all or has the highest amount of detail (LOD 0) skip it and drop the node
        if ((chunkBorders.x != 0 || chunkBorders.y != 0 || chunkBorders.z != 0) && leafSize != nodeMinSize) {
            for (int i = 0; i < 12; i++) {
                if (!(  (chunkBorders.x != 0 && chunkBorders.x + 1 == EDGE_OFFSETS[i].x) ||
                        (chunkBorders.y != 0 && chunkBorders.y + 1 == EDGE_OFFSETS[i].y) ||
                        (chunkBorders.z != 0 && chunkBorders.z + 1 == EDGE_OFFSETS[i].z))) {
                    continue;
                }
                // node size at LOD 0 = 1, LOD 1 = 2, LOD 2 = 4, LOD 3 = 8
                int x = leafMin.x + (EDGE_OFFSETS[i].x) * leafSize / 2;
                int y = leafMin.y + (EDGE_OFFSETS[i].y) * leafSize / 2;
                int z = leafMin.z + (EDGE_OFFSETS[i].z) * leafSize / 2;

                Vec4f nodePos = new Vec4f(x,y,z);
                float density = Density.getNoise(nodePos, densityField);
                if ((density < 0 && corners == 0) || (density >= 0 && corners == 255)) {
                    return nodePos;
                }
            }
        }
        return null;    // voxel is full inside or outside the volume
    }

    protected int field_index(Vec3i pos) {
        return pos.x + (pos.y * fieldSize) + (pos.z * fieldSize * fieldSize);
    }
}
