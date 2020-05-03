package dc;

import core.math.Vec3i;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static dc.OctreeNodeType.*;
import static dc.VoxelOctree.MATERIAL_AIR;
import static dc.VoxelOctree.edgevmap;

public class DualContouringImpl implements DualContouring{

    private void ContourProcessEdge(OctreeNode[] node, int dir, List<Integer> indexBuffer)
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

    private void ContourEdgeProc(OctreeNode[] node, int dir, List<Integer> buffer, boolean isSeam) {
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
                OctreeNode[] edgeNodes = new OctreeNode[4];
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

    private void ContourFaceProc(OctreeNode[] node, int dir, List<Integer> buffer, boolean isSeam) {
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
                OctreeNode[] faceNodes = new OctreeNode[2];
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
                OctreeNode[] edgeNodes = new OctreeNode[4];
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
    public void ContourCellProc(OctreeNode node, List<Integer> buffer, boolean isSeam) {
        if (node == null || node.Type == Node_Leaf) {
            return;
        }

        for (int i = 0; i < 8; i++) {
            ContourCellProc(node.children[i], buffer, isSeam);
        }

        for (int i = 0; i < 12; i++) {
            OctreeNode[] faceNodes = new OctreeNode[2];
		    int[] c = { cellProcFaceMask[i][0], cellProcFaceMask[i][1] };

            faceNodes[0] = node.children[c[0]];
            faceNodes[1] = node.children[c[1]];

            ContourFaceProc(faceNodes, cellProcFaceMask[i][2], buffer, isSeam);
        }

        for (int i = 0; i < 6; i++) {
            OctreeNode[] edgeNodes = new OctreeNode[4];
            int[] c = {cellProcEdgeMask[i][0], cellProcEdgeMask[i][1], cellProcEdgeMask[i][2], cellProcEdgeMask[i][3]};

            for (int j = 0; j < 4; j++) {
                edgeNodes[j] = node.children[c[j]];
            }

            ContourEdgeProc(edgeNodes, cellProcEdgeMask[i][4], buffer, isSeam);
        }
    }

/*
//from old dc
    private static void ContourProcessEdge(OctreeNode[] node, int dir, List<Integer> indexBuffer)
    {
        int minSize = 1000000;		// arbitrary big number
        int minIndex = 0;
        int[] indices = { -1, -1, -1, -1 };
        boolean flip = false;
        boolean[] signChange = { false, false, false, false };

        for (int i = 0; i < 4; i++)
        {
            int edge = processEdgeMask[dir][i];
            int c1 = edgevmap[edge][0];
            int c2 = edgevmap[edge][1];

            int m1 = (node[i].drawInfo.corners >> c1) & 1;
            int m2 = (node[i].drawInfo.corners >> c2) & 1;

            if (node[i].size < minSize)
            {
                minSize = node[i].size;
                minIndex = i;
                flip = m1 != MATERIAL_AIR;
            }

            indices[i] = node[i].drawInfo.index;

            signChange[i] =
                    (m1 == MATERIAL_AIR && m2 != MATERIAL_AIR) ||
                            (m1 != MATERIAL_AIR && m2 == MATERIAL_AIR);
        }

        if (signChange[minIndex])
        {
            if (!flip)
            {
                indexBuffer.add(indices[0]);
                indexBuffer.add(indices[1]);
                indexBuffer.add(indices[3]);

                indexBuffer.add(indices[0]);
                indexBuffer.add(indices[3]);
                indexBuffer.add(indices[2]);
            }
            else
            {
                indexBuffer.add(indices[0]);
                indexBuffer.add(indices[3]);
                indexBuffer.add(indices[1]);

                indexBuffer.add(indices[0]);
                indexBuffer.add(indices[2]);
                indexBuffer.add(indices[3]);
            }
        }
    }

    private static void ContourEdgeProc(OctreeNode[] node, int dir, List<Integer> indexBuffer, boolean isSeam) {
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

        if (node[0].Type != Node_Internal && node[1].Type != Node_Internal && node[2].Type != Node_Internal && node[3].Type != Node_Internal) {
            ContourProcessEdge(node, dir, indexBuffer);
        }
        else
        {
            for (int i = 0; i < 2; i++)
            {
                OctreeNode[] edgeNodes = new OctreeNode[4];
                int[] c = {
                        edgeProcEdgeMask[dir][i][0],
                        edgeProcEdgeMask[dir][i][1],
                        edgeProcEdgeMask[dir][i][2],
                        edgeProcEdgeMask[dir][i][3],
                };

                for (int j = 0; j < 4; j++)
                {
                    if (node[j].Type == Node_Leaf || node[j].Type == Node_Psuedo)
                    {
                        edgeNodes[j] = node[j];
                    }
                    else
                    {
                        edgeNodes[j] = node[j].children[c[j]];
                    }
                }

                ContourEdgeProc(edgeNodes, edgeProcEdgeMask[dir][i][4], indexBuffer, isSeam);
            }
        }
    }

    private static void ContourFaceProc(OctreeNode[] node, int dir, List<Integer> indexBuffer, boolean isSeam) {
        if (node[0] == null || node[1] == null) {
            return;
        }

        // bit of a hack but it works: prevent overlapping seams by only processing edges that stradle multiple chunks
        if (isSeam && ChunkOctree.chunkMinForPosition(node[0]).equals(ChunkOctree.chunkMinForPosition(node[1]))) {
            return;
        }

        if (node[0].Type == Node_Internal || node[1].Type == Node_Internal) {
            for (int i = 0; i < 4; i++) {
                OctreeNode[] faceNodes = new OctreeNode[2];
                int[] c = {
                        faceProcFaceMask[dir][i][0], faceProcFaceMask[dir][i][1],
                };

                for (int j = 0; j < 2; j++) {
                    if (node[j].Type != Node_Internal) {
                        faceNodes[j] = node[j];
                    }
                    else {
                        faceNodes[j] = node[j].children[c[j]];
                    }
                }

                ContourFaceProc(faceNodes, faceProcFaceMask[dir][i][2], indexBuffer, isSeam);
            }

            int[][] orders = {
                    { 0, 0, 1, 1 },
                    { 0, 1, 0, 1 },
            };

            for (int i = 0; i < 4; i++) {
                OctreeNode[] edgeNodes = new OctreeNode[4];
                int[] c = {
                        faceProcEdgeMask[dir][i][1],
                        faceProcEdgeMask[dir][i][2],
                        faceProcEdgeMask[dir][i][3],
                        faceProcEdgeMask[dir][i][4],
                };

                int[] order = orders[faceProcEdgeMask[dir][i][0]];
                for (int j = 0; j < 4; j++) {
                    if (node[order[j]].Type == Node_Leaf || node[order[j]].Type == Node_Psuedo) {
                        edgeNodes[j] = node[order[j]];
                    }
                    else {
                        edgeNodes[j] = node[order[j]].children[c[j]];
                    }
                }

                ContourEdgeProc(edgeNodes, faceProcEdgeMask[dir][i][5], indexBuffer, isSeam);
            }
        }
    }

    public static void ContourCellProc(OctreeNode node, List<Integer> indexBuffer, boolean isSeam) {
        if (node == null) {
            return;
        }

        if (node.Type == Node_Internal) {
            for (int i = 0; i < 8; i++) {
                ContourCellProc(node.children[i], indexBuffer, isSeam);
            }

            for (int i = 0; i < 12; i++) {
                OctreeNode[] faceNodes = new OctreeNode[2];
                int[] c = { cellProcFaceMask[i][0], cellProcFaceMask[i][1] };

                faceNodes[0] = node.children[c[0]];
                faceNodes[1] = node.children[c[1]];

                ContourFaceProc(faceNodes, cellProcFaceMask[i][2], indexBuffer, isSeam);
            }

            for (int i = 0; i < 6; i++) {
                OctreeNode[] edgeNodes = new OctreeNode[4];
                int[] c = {
                        cellProcEdgeMask[i][0],
                        cellProcEdgeMask[i][1],
                        cellProcEdgeMask[i][2],
                        cellProcEdgeMask[i][3],
                };

                for (int j = 0; j < 4; j++) {
                    edgeNodes[j] = node.children[c[j]];
                }

                ContourEdgeProc(edgeNodes, cellProcEdgeMask[i][4], indexBuffer, isSeam);
            }
        }
    }
    */
}