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
import dc.utils.SimplexNoise;
import dc.entities.IntHolder;

import java.util.*;

import static dc.OctreeNodeType.Node_Leaf;

public class ManifoldDCOctreeImpl extends AbstractDualContouring implements VoxelOctree{
    public static boolean EnforceManifold = true;

    public ManifoldDCOctreeImpl(MeshGenerationContext meshGenerationContext) {
        super(meshGenerationContext);
    }

    @Override
    protected MdcOctreeNode createParent(Vec3i min, int size, OctreeNodeType type) {
        return new MdcOctreeNode(min, size, OctreeNodeType.Node_Internal);
    }

    @Override
    public boolean createLeafVoxelNodes(int chunkSize, Vec3i chunkMin, List<OctreeNode> seamNodes, MeshBuffer meshBuffer, GPUDensityField field) {
        List<OctreeNode> chunkNodes = new ArrayList<>();
        if (!simpleDebugCreateLeafVoxelNodes(chunkSize, chunkMin, chunkNodes, seamNodes)){
            return false;
        }
        processNodesToMesh(chunkNodes, chunkMin, chunkSize, false, meshBuffer);
        return true;
    }

    private boolean nodeIsSeam(int zi, int yi, int xi) {
        return xi==0||xi==meshGen.getVoxelsPerChunk()-1 || yi==0||yi==meshGen.getVoxelsPerChunk()-1 || zi==0||zi==meshGen.getVoxelsPerChunk()-1;
    }

    private boolean simpleDebugCreateLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                                    List<OctreeNode> chunkNodes, List<OctreeNode> seamNodes) {
        IntHolder n_index = new IntHolder(1);
        for (int zi = 0; zi < meshGen.getVoxelsPerChunk(); zi++) {
            for (int yi = 0; yi < meshGen.getVoxelsPerChunk(); yi++) {
                for (int xi = 0; xi < meshGen.getVoxelsPerChunk(); xi++) {
                    Vec3i pos = new Vec3i(xi, yi, zi);
                    int leafSize = (chunkSize / meshGen.getVoxelsPerChunk());
                    Vec3i leafMin = pos.mul(leafSize).add(chunkMin);
                    MdcOctreeNode leaf = ConstructLeaf(new MdcOctreeNode(leafMin, leafSize, OctreeNodeType.Node_Leaf), n_index,
                            pos, meshGen.leafSizeScale);
                    if (leaf != null) {
                        chunkNodes.add(leaf);
                        if(nodeIsSeam(zi, yi, xi)) {
                            seamNodes.add(leaf);
                        }
                    }
                }
            }
        }
        return !chunkNodes.isEmpty() && !seamNodes.isEmpty();
    }

    protected void GenerateMeshFromOctree(OctreeNode node, boolean isSeam, MeshBuffer buffer, int chunkSize) {
        if (node == null) {
            return;
        }
        List<MeshVertex> vertices = new ArrayList<>();
        List<Integer> indcies = new ArrayList<>();
        //ClusterCellBase(node, 0);
        GenerateVertexBuffer((MdcOctreeNode) node, vertices);
        List<Integer> tri_count = new ArrayList<>();
        ProcessCell(node, indcies, isSeam, tri_count, 0, chunkSize);
        for (int i = 0; i < indcies.size(); i++) {
            indcies.set(i, indcies.get(i) & 0xFFFFFFF);
        }

        buffer.setVertices(BufferUtil.createDcFlippedBufferAOS(vertices));
        buffer.setIndicates(BufferUtil.createFlippedBuffer(indcies));
        buffer.setNumVertices(vertices.size());
        buffer.setNumIndicates(indcies.size());
        vertices.clear();
        indcies.clear();
    }

    public void GenerateVertexBuffer(MdcOctreeNode node, List<MeshVertex> vertices) {
        if (node.Type != OctreeNodeType.Node_Leaf) {
            for (int i = 0; i < 8; i++) {
                if (node.children[i] != null)
                    GenerateVertexBuffer((MdcOctreeNode) node.children[i], vertices);
            }
        }

        if (vertices == null || node.vertices.length == 0)
            return;

        for (int i = 0; i < node.vertices.length; i++) {
            if (node.vertices[i] == null)
                continue;
            node.vertices[i].index = vertices.size();
            Vec3f nc = node.vertices[i].normal.mul(0.5f).add(new Vec3f(1).mul(0.5f));
            nc.normalize();

            Vec3f color = node.vertices[i].debugFlag ? Constants.Red : nc;
            if(node.vertices[i].qef==null){
                vertices.add(new MeshVertex(node.vertices[i].pos, node.vertices[i].normal, color));
            } else {
                vertices.add(new MeshVertex(node.vertices[i].qef.solve().getVec3f(), node.vertices[i].normal, color));
            }
        }
    }

    public MdcOctreeNode ConstructLeaf(MdcOctreeNode leaf, IntHolder n_index, Vec3i pos, int leafSizeScale) {
        leaf.index = n_index.value++;
        leaf.Type = OctreeNodeType.Node_Leaf;
        int corners = 0;
        float[] samples = new float[8];
        for (int i = 0; i < 8; i++) {
            Vec3f vertexPos = leaf.min.add(Utilities.TCornerDeltas[i].mul(leaf.size)).toVec3f();
            if ((samples[i] = SimplexNoise.getNoise(vertexPos)) < 0)
                corners |= 1 << i;
        }
        leaf.corners = corners;
//        if (corners == 0 || corners == 255) {
//            return null;
//        }
        if (corners == 0 || corners == 255) {
            // to avoid holes in seams between chunks with different resolution we creating some other nodes only in seams
            //https://www.reddit.com/r/VoxelGameDev/comments/6kn8ph/dual_contouring_seam_stitching_problem/
            Vec4f nodePos = tryToCreateBoundSeamPseudoNode(leaf.min, leaf.size, pos, corners, leafSizeScale);
            if(nodePos==null){
                return null;
            } else {
                leaf.corners = corners;
                leaf.Type = Node_Leaf;
                MdcVertex mdcVertex = new MdcVertex();
                mdcVertex.pos = nodePos.getVec3f();
                mdcVertex.normal = CalculateSurfaceNormal(nodePos).getVec3f();
                leaf.vertices = new MdcVertex[1];
                leaf.vertices[0] = mdcVertex;
                return leaf;
            }
        }

        int[][] v_edges = new int[Utilities.VerticesNumberTable[corners]][];
        leaf.vertices = new MdcVertex[Utilities.VerticesNumberTable[corners]];

        int v_index = 0;
        int e_index = 0;
        v_edges[0] = new int[] { -1, -1, -1, -1, -1, -1, -1, -1 };

        for (int e = 0; e < 16; e++) {
            int code = Utilities.TransformedEdgesTable[corners][e];
            if (code == -2) {
                v_index++;
                break;
            }

            if (code == -1) {
                v_index++;
                e_index = 0;
                v_edges[v_index] = new int[] { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
                continue;
            }
            v_edges[v_index][e_index++] = code;
        }

        for (int i = 0; i < v_index; i++) {
            int k = 0;
            leaf.vertices[i] = new MdcVertex();
            leaf.vertices[i].qef = new QEFData(new LevenQefSolver());
            Vec3f normal = new Vec3f();
            int[] ei = new int[12];
            while (v_edges[i][k] != -1) {
                ei[v_edges[i][k]] = 1;
                Vec3f a = leaf.min.add(Utilities.TCornerDeltas[Utilities.TEdgePairs[v_edges[i][k]][0]].mul(leaf.size)).toVec3f();
                Vec3f b = leaf.min.add(Utilities.TCornerDeltas[Utilities.TEdgePairs[v_edges[i][k]][1]].mul(leaf.size)).toVec3f();
                Vec3f intersect = GetIntersection(a, b, samples[Utilities.TEdgePairs[v_edges[i][k]][0]],
                                                        samples[Utilities.TEdgePairs[v_edges[i][k]][1]]);
                Vec3f n  = CalculateSurfaceNormal(intersect);
                normal = normal.add(n);
                leaf.vertices[i].qef.qef_add_point(intersect, n);
                k++;
            }
            normal = normal.div(k);
            normal.normalize();
            leaf.vertices[i].index = 0;
            leaf.vertices[i].parent = null;
            leaf.vertices[i].collapsible = true;
            leaf.vertices[i].normal = normal;
            leaf.vertices[i].euler = 1;
            leaf.vertices[i].eis = ei;
            leaf.vertices[i].in_cell = leaf.child_index;
            leaf.vertices[i].face_prop2 = true;
            leaf.vertices[i].pos = leaf.vertices[i].qef.solve().getVec3f();
            leaf.vertices[i].error = leaf.vertices[i].qef.getError();
        }
        return leaf;
    }

    public static Vec3f GetIntersection(Vec3f p1, Vec3f p2, float d1, float d2) {
        //do a simple linear interpolation return p1 + (-d1) * (p2 - p1) / (d2 - d1);
        return p1.add(p2.sub(p1).mul(-d1).div(d2 - d1));
    }

    public Vec3f CalculateSurfaceNormal(Vec3f p) {
        float H = 1f;
        Vec3f xOffcet = new Vec3f(H, 0.f, 0.f);
        Vec3f yOffcet = new Vec3f(0.f, H, 0.f);
        Vec3f zOffcet = new Vec3f(0.f, 0.f, H);
        float dx = SimplexNoise.getNoise(p.add(xOffcet)) - SimplexNoise.getNoise(p.sub(xOffcet));
        float dy = SimplexNoise.getNoise(p.add(yOffcet)) - SimplexNoise.getNoise(p.sub(yOffcet));
        float dz = SimplexNoise.getNoise(p.add(zOffcet)) - SimplexNoise.getNoise(p.sub(zOffcet));

        Vec3f v = new Vec3f(dx, dy, dz);
        v.normalize();
        return v;
    }

    public void ProcessCell(OctreeNode node, List<Integer> indexes, boolean isSeam, List<Integer> tri_count, float threshold, int chunkSize) {
        if (node.Type == OctreeNodeType.Node_Internal) {
            for (int i = 0; i < 8; i++) {
                if (node.children[i] != null) {
                    ProcessCell(node.children[i], indexes, isSeam, tri_count, threshold, chunkSize);
                }
            }

            for (int i = 0; i < 12; i++) {
                OctreeNode[] face_nodes = new MdcOctreeNode[2];

                int c1 = Utilities.TEdgePairs[i][0];
                int c2 = Utilities.TEdgePairs[i][1];

                face_nodes[0] = node.children[c1];
                face_nodes[1] = node.children[c2];

                ProcessFace(face_nodes, Utilities.TEdgePairs[i][2], indexes, tri_count, threshold, isSeam, chunkSize);
            }

            for (int i = 0; i < 6; i++)
            {
                OctreeNode[] edge_nodes = {
                        node.children[Utilities.TCellProcEdgeMask[i][0]],
                        node.children[Utilities.TCellProcEdgeMask[i][1]],
                        node.children[Utilities.TCellProcEdgeMask[i][2]],
                        node.children[Utilities.TCellProcEdgeMask[i][3]]
                };

                ProcessEdge(edge_nodes, Utilities.TCellProcEdgeMask[i][4], indexes, tri_count, threshold, isSeam, chunkSize);
            }
        }
    }

    public void ProcessFace(OctreeNode[] nodes, int direction, List<Integer> indexes, List<Integer> tri_count, float threshold, boolean isSeam, int chunkSize)
    {
        if (nodes[0] == null || nodes[1] == null)
            return;

        // bit of a hack but it works: prevent overlapping seams by only processing edges that stradle multiple chunks
        if (isSeam && chunkMinForPosition(nodes[0].min, chunkSize).equals(chunkMinForPosition(nodes[1].min, chunkSize))) {
            return;
        }

        if (nodes[0].Type != OctreeNodeType.Node_Leaf || nodes[1].Type != OctreeNodeType.Node_Leaf) {
            for (int i = 0; i < 4; i++) {
                OctreeNode[] face_nodes = new MdcOctreeNode[2];

                for (int j = 0; j < 2; j++) {
                    if (nodes[j].Type == OctreeNodeType.Node_Leaf)
                        face_nodes[j] = nodes[j];
                    else
                        face_nodes[j] = nodes[j].children[Utilities.TFaceProcFaceMask[direction][i][j]];
                }

                ProcessFace(face_nodes, Utilities.TFaceProcFaceMask[direction][i][2], indexes, tri_count, threshold, isSeam, chunkSize);
            }

            int[][] orders = {
                    { 0, 0, 1, 1 },
                    { 0, 1, 0, 1 },
            };

            for (int i = 0; i < 4; i++) {
                OctreeNode[] edge_nodes = new MdcOctreeNode[4];

                for (int j = 0; j < 4; j++) {
                    if (nodes[orders[Utilities.TFaceProcEdgeMask[direction][i][0]][j]].Type == OctreeNodeType.Node_Leaf)
                        edge_nodes[j] = nodes[orders[Utilities.TFaceProcEdgeMask[direction][i][0]][j]];
                    else
                        edge_nodes[j] = nodes[orders[Utilities.TFaceProcEdgeMask[direction][i][0]][j]].children[Utilities.TFaceProcEdgeMask[direction][i][1 + j]];
                }

                ProcessEdge(edge_nodes, Utilities.TFaceProcEdgeMask[direction][i][5], indexes, tri_count, threshold, isSeam, chunkSize);
            }
        }
    }

    public void ProcessEdge(OctreeNode[] nodes, int direction, List<Integer> indexes, List<Integer> tri_count, float threshold, boolean isSeam, int chunkSize)
    {
        if (nodes[0] == null || nodes[1] == null || nodes[2] == null || nodes[3] == null)
            return;

        // bit of a hack but it works: prevent overlapping seams by only processing edges that stradle multiple chunks
        if(isSeam) {
            Set<Vec3i> chunks = new HashSet<>();
            for (int i = 0; i < 4; i++) {
                chunks.add(chunkMinForPosition(nodes[i].min, chunkSize));
            }
            if (chunks.size() == 1)
                return;
        }

        if (nodes[0].Type == OctreeNodeType.Node_Leaf && nodes[1].Type == OctreeNodeType.Node_Leaf && nodes[2].Type == OctreeNodeType.Node_Leaf && nodes[3].Type == OctreeNodeType.Node_Leaf)
        {
            // To avoid overlap seam mesh with chunk mesh. If all 4 nodes of seam belong to only one chunk, then this is not a seam.
            if(isSeam &&
                    (nodes[0].getChunk().equals(nodes[1].getChunk()) &&
                            nodes[1].getChunk().equals(nodes[2].getChunk()) &&
                            nodes[2].getChunk().equals(nodes[3].getChunk()))
            ){
                return;
            }
            ProcessIndexes(nodes, direction, indexes, tri_count, threshold);
        }
        else
        {
            for (int i = 0; i < 2; i++) {
                OctreeNode[] edge_nodes = new MdcOctreeNode[4];
                for (int j = 0; j < 4; j++) {
                    if (nodes[j].Type == OctreeNodeType.Node_Leaf)
                        edge_nodes[j] = nodes[j];
                    else
                        edge_nodes[j] = nodes[j].children[Utilities.TEdgeProcEdgeMask[direction][i] [j]];
                }
                ProcessEdge(edge_nodes, Utilities.TEdgeProcEdgeMask[direction][i][4], indexes, tri_count, threshold, isSeam, chunkSize);
            }
        }
    }

    public static void ProcessIndexes(OctreeNode[] nodes, int direction, List<Integer> indexes, List<Integer> tri_count, float threshold) {
        int min_size = 10000000;
        int min_index = 0;
        int[] indices = { -1, -1, -1, -1 };
        boolean flip = false;
        boolean sign_changed = false;
        int v_count = 0;
        //return;

        for (int i = 0; i < 4; i++) {
            int edge = Utilities.TProcessEdgeMask[direction][i];
            int c1 = Utilities.TEdgePairs[edge][0];
            int c2 = Utilities.TEdgePairs[edge][1];

            int m1 = (nodes[i].corners >> c1) & 1;
            int m2 = (nodes[i].corners >> c2) & 1;

            if (nodes[i].size < min_size) {
                min_size = nodes[i].size;
                min_index = i;
                flip = m1 == 1;
                sign_changed = ((m1 == 0 && m2 != 0) || (m1 != 0 && m2 == 0));
            }

            //if (!((m1 == 0 && m2 != 0) || (m1 != 0 && m2 == 0)))
            //	continue;

            //find the vertex index
            int index = 0;
            boolean skip = false;
            for (int k = 0; k < 16; k++) {
                int e = Utilities.TransformedEdgesTable[nodes[i].corners][k];
                if (e == -1)
                {
                    index++;
                    continue;
                }
                if (e == -2)
                {
                    skip = true;
                    break;
                }
                if (e == edge)
                    break;
            }

            if (skip)
                continue;

            v_count++;
            MdcOctreeNode mdcOctreeNode = (MdcOctreeNode) nodes[i];
            if (index >= mdcOctreeNode.vertices.length)
                return;
            MdcVertex v = mdcOctreeNode.vertices[index];
            MdcVertex highest = v;
            while (highest.parent != null)
            {
                if ((highest.parent.error <= threshold && (!EnforceManifold || (highest.parent.euler == 1 && highest.parent.face_prop2))))
                    highest = v = highest.parent;
                else
                    highest = highest.parent;
            }
            indices[i] = v.index;
            //sign_changed = true;
        }

        /*
         * Next generate the triangles.
         * Because we're generating from the finest levels that were collapsed, many triangles will collapse to edges or vertices.
         * That's why we check if the indices are different and discard the triangle, as mentioned in the paper.
         */
        if (sign_changed) {
            int count = 0;
            if (!flip) {
                if (indices[0] != -1 && indices[1] != -1 && indices[2] != -1 && indices[0] != indices[1] && indices[1] != indices[3]) {
                    indexes.add(indices[0]);
                    indexes.add(indices[1]);
                    indexes.add(indices[3]);
                    count++;
                }

                if (indices[0] != -1 && indices[2] != -1 && indices[3] != -1 && indices[0] != indices[2] && indices[2] != indices[3]) {
                    indexes.add(indices[0]);
                    indexes.add(indices[3]);
                    indexes.add(indices[2]);
                    count++;
                }
            }
            else {
                if (indices[0] != -1 && indices[3] != -1 && indices[1] != -1 && indices[0] != indices[1] && indices[1] != indices[3]) {
                    indexes.add(0x10000000 | indices[0]);
                    indexes.add(0x10000000 | indices[3]);
                    indexes.add(0x10000000 | indices[1]);
                    count++;
                }

                if (indices[0] != -1 && indices[2] != -1 && indices[3] != -1 && indices[0] != indices[2] && indices[2] != indices[3]) {
                    indexes.add(0x10000000 | indices[0]);
                    indexes.add(0x10000000 | indices[2]);
                    indexes.add(0x10000000 | indices[3]);
                    count++;
                }
            }

            if (count > 0) {
                tri_count.add(count);
            }
        }
    }

    public void ClusterCellBase(OctreeNode node, float error) {
        if (node.Type != OctreeNodeType.Node_Internal)
            return;

        for (int i = 0; i < 8; i++) {
            if (node.children[i] == null)
                continue;
            ClusterCell(node.children[i], error);
        }
    }

    /*
     * Cell stage
     */
    public void ClusterCell(OctreeNode node, float error)
    {
        if (node.Type != OctreeNodeType.Node_Internal)
            return;

        /*
         * First cluster all the children nodes
         */

        int[] signs = { -1, -1, -1, -1, -1, -1, -1, -1 };
        int mid_sign = -1;

        boolean is_collapsible = true;
        for (int i = 0; i < 8; i++)
        {
            if (node.children[i] == null)
                continue;

            ClusterCell(node.children[i], error);
            if (node.children[i].Type == OctreeNodeType.Node_Internal) //Can't cluster if the child has children
                is_collapsible = false;
            else
            {
                mid_sign = (node.children[i].corners >> (7 - i)) & 1;
                signs[i] = (node.children[i].corners >> i) & 1;
            }
        }

        node.corners = 0;
        for (int i = 0; i < 8; i++) {
            if (signs[i] == -1)
                node.corners |= (byte)(mid_sign << i);
            else
                node.corners |= (byte)(signs[i] << i);
        }

        IntHolder surface_index = new IntHolder(0);
        List<MdcVertex> collected_vertices = new ArrayList<>();
        List<MdcVertex> new_vertices = new ArrayList<>();

        /*
         * Find all the surfaces inside the children that cross the 6 Euclidean edges and the vertices that connect to them
         */
        for (int i = 0; i < 12; i++)
        {
            OctreeNode[] face_nodes = new MdcOctreeNode[2];

            int c1 = Utilities.TEdgePairs[i][0];
            int c2 = Utilities.TEdgePairs[i][1];

            face_nodes[0] = node.children[c1];
            face_nodes[1] = node.children[c2];

            ClusterFace(face_nodes, Utilities.TEdgePairs[i][2], surface_index, collected_vertices);
        }

        for (int i = 0; i < 6; i++)
        {
            OctreeNode[] edge_nodes = {
                    node.children[Utilities.TCellProcEdgeMask[i][0]],
                    node.children[Utilities.TCellProcEdgeMask[i][1]],
                    node.children[Utilities.TCellProcEdgeMask[i][2]],
                    node.children[Utilities.TCellProcEdgeMask[i][3]]
            };
            ClusterEdge(edge_nodes, Utilities.TCellProcEdgeMask[i][4], surface_index, collected_vertices);
        }

        int highest_index = surface_index.value;

        if (highest_index == -1)
            highest_index = 0;
        /*
         * Gather the stray vertices
         */
        for(OctreeNode n : node.children) {
            if (n == null)
                continue;
            for(MdcVertex v : ((MdcOctreeNode)n).vertices) {
                if (v == null)
                    continue;
                if (v.surface_index == -1)
                {
                    v.surface_index = highest_index++;
                    collected_vertices.add(v);
                }
            }
        }

        int clustered_count = 0;
        if (collected_vertices.size() > 0)
        {

            for (int i = 0; i <= highest_index; i++)
            {
                QEFData qef = new QEFData(new LevenQefSolver());
                Vec3f normal = new Vec3f(0);
                int count = 0;
                int[] edges = new int[12];
                int euler = 0;
                int e = 0;
                for(MdcVertex v : collected_vertices)
                {
                    if (v.surface_index == i) {
                        /* Calculate ei(Sv) */
                        for (int k = 0; k < 3; k++) {
                            int edge = Utilities.TExternalEdges[v.in_cell][k];
                            edges[edge] += v.eis[edge];
                        }
                        /* Calculate e(Svk) */
                        for (int k = 0; k < 9; k++) {
                            int edge = Utilities.TInternalEdges[v.in_cell][k];
                            e += v.eis[edge];
                        }

                        euler += v.euler;
                        qef.add(v.qef);
                        normal = normal.add(v.normal);
                        count++;
                    }
                }

                /*
                 * One vertex might have an error greater than the threshold, preventing simplification.
                 * When it's just one, we can ignore the error and proceed.
                 */
                if (count == 0) {
                    continue;
                }

                boolean face_prop2 = true;
                for (int f = 0; f < 6 && face_prop2; f++)
                {
                    int intersections = 0;
                    for (int ei = 0; ei < 4; ei++)
                    {
                        intersections += edges[Utilities.TFaces[f][ei]];
                    }
                    if (!(intersections == 0 || intersections == 2))
                        face_prop2 = false;
                }

                MdcVertex new_vertex = new MdcVertex();
                normal = normal.div((float)count);
                normal.normalize();
                new_vertex.normal = normal;
                new_vertex.qef = qef;
                new_vertex.eis = edges;
                new_vertex.euler = euler - e / 4;

                new_vertex.in_cell = node.child_index;
                new_vertex.face_prop2 = face_prop2;

                new_vertices.add(new_vertex);
                new_vertex.pos = qef.solve().getVec3f();
                float err = qef.getError();

                new_vertex.collapsible = err <= error/* && new_vertex.euler == 1 && face_prop2*/;
                new_vertex.error = err;
                clustered_count++;

                for(MdcVertex v : collected_vertices)
                {
                    if (v.surface_index == i)
                    {
                        if (v != new_vertex)
                            v.parent = new_vertex;
                        else
                            v.parent = null;
                    }
                }
            }
        }
        else
        {
            return;
        }

        //if (clustered_count <= 0)
        {
            for(MdcVertex v2 : collected_vertices) {
                v2.surface_index = -1;
            }
        }

        ((MdcOctreeNode)node).vertices = new_vertices.toArray(new MdcVertex[0]);
    }

    public static void ClusterFace(OctreeNode[] nodes, int direction, IntHolder intHolder, List<MdcVertex> collected_vertices)
    {
        if (nodes[0] == null || nodes[1] == null)
            return;

        if (nodes[0].Type != OctreeNodeType.Node_Leaf || nodes[1].Type != OctreeNodeType.Node_Leaf) {
            for (int i = 0; i < 4; i++) {
                OctreeNode[] face_nodes = new MdcOctreeNode[2];

                for (int j = 0; j < 2; j++) {
                    if (nodes[j] == null)
                        continue;
                    if (nodes[j].Type != OctreeNodeType.Node_Internal)
                        face_nodes[j] = nodes[j];
                    else
                        face_nodes[j] = nodes[j].children[Utilities.TFaceProcFaceMask[direction][i][j]];
                }

                ClusterFace(face_nodes, Utilities.TFaceProcFaceMask[direction][i][2], intHolder, collected_vertices);
            }
        }

        int[][] orders = {
                { 0, 0, 1, 1 },
                { 0, 1, 0, 1 },
        };

        for (int i = 0; i < 4; i++)
        {
            OctreeNode[] edge_nodes = new MdcOctreeNode[4];

            for (int j = 0; j < 4; j++)
            {
                if (nodes[orders[Utilities.TFaceProcEdgeMask[direction][i][0]][j]] == null)
                    continue;
                if (nodes[orders[Utilities.TFaceProcEdgeMask[direction][i][0]][j]].Type != OctreeNodeType.Node_Internal)
                    edge_nodes[j] = nodes[orders[Utilities.TFaceProcEdgeMask[direction][i][0]] [j]];
                else
                    edge_nodes[j] = nodes[orders[Utilities.TFaceProcEdgeMask[direction][i][0]] [j]].children[Utilities.TFaceProcEdgeMask[direction][i][1 + j]];
            }

            ClusterEdge(edge_nodes, Utilities.TFaceProcEdgeMask[direction][i][5], intHolder, collected_vertices);
        }
    }

    public static void ClusterEdge(OctreeNode[] nodes, int direction, IntHolder intHolder, List<MdcVertex> collected_vertices)
    {
        if ((nodes[0] == null || nodes[0].Type != OctreeNodeType.Node_Internal) && (nodes[1] == null || nodes[1].Type != OctreeNodeType.Node_Internal) && (nodes[2] == null || nodes[2].Type != OctreeNodeType.Node_Internal) && (nodes[3] == null || nodes[3].Type != OctreeNodeType.Node_Internal))
        {
            ClusterIndexes(nodes, direction, intHolder, collected_vertices);
        }
        else
        {
            for (int i = 0; i < 2; i++) {
                OctreeNode[] edge_nodes = new MdcOctreeNode[4];
                for (int j = 0; j < 4; j++) {
                    if (nodes[j] == null)
                        continue;
                    if (nodes[j].Type == OctreeNodeType.Node_Leaf)
                        edge_nodes[j] = nodes[j];
                    else
                        edge_nodes[j] = nodes[j].children[Utilities.TEdgeProcEdgeMask[direction][i][j]];
                }

                ClusterEdge(edge_nodes, Utilities.TEdgeProcEdgeMask[direction][i][4], intHolder, collected_vertices);
            }
        }
    }

    public static void ClusterIndexes(OctreeNode[] nodes, int direction, IntHolder intHolder, List<MdcVertex> collected_vertices) {
        if (nodes[0] == null && nodes[1] == null && nodes[2] == null && nodes[3] == null)
            return;

        MdcVertex[] vertices = new MdcVertex[4];
        int v_count = 0;
        int node_count = 0;

        for (int i = 0; i < 4; i++) {
            if (nodes[i] == null)
                continue;

            node_count++;

            int edge = Utilities.TProcessEdgeMask[direction][i];
            int c1 = Utilities.TEdgePairs[edge][0];
            int c2 = Utilities.TEdgePairs[edge][1];

            int m1 = (nodes[i].corners >> c1) & 1;
            int m2 = (nodes[i].corners >> c2) & 1;

            //find the vertex index
            int index = 0;
            boolean skip = false;
            for (int k = 0; k < 16; k++) {
                int e = Utilities.TransformedEdgesTable[nodes[i].corners][k];
                if (e == -1) {
                    index++;
                    continue;
                }
                if (e == -2) {
                    if (!((m1 == 0 && m2 != 0) || (m1 != 0 && m2 == 0)))
                        skip = true;
                    break;
                }
                if (e == edge)
                    break;
            }

            MdcOctreeNode mdcOctreeNode = (MdcOctreeNode) nodes[i];
            if (!skip && index < mdcOctreeNode.vertices.length) {
                vertices[i] = mdcOctreeNode.vertices[index];
                while (vertices[i].parent != null)
                    vertices[i] = vertices[i].parent;
                v_count++;
            }
        }

        if (v_count == 0)
            return;

        int surface_index = -1;

        for (int i = 0; i < 4; i++) {
            MdcVertex v = vertices[i];
            if (v == null)
                continue;
            if (v.surface_index != -1) {
                if (surface_index != -1 && surface_index != v.surface_index) {
                    AssignSurface(collected_vertices, v.surface_index, surface_index);
                }
                else if (surface_index == -1)
                    surface_index = v.surface_index;
            }
        }

        if (surface_index == -1)
            surface_index = intHolder.value++;

        for (int i = 0; i < 4; i++)
        {
            MdcVertex v = vertices[i];
            if (v == null)
                continue;
            if (v.surface_index == -1) {
                collected_vertices.add(v);
            }
            v.surface_index = surface_index;
        }
    }

    private static void AssignSurface(List<MdcVertex> vertices, int from, int to) {
        for(MdcVertex v : vertices) {
            if (v != null && v.surface_index == from)
                v.surface_index = to;
        }
    }
}
