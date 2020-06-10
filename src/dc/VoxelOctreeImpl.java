package dc;

import core.math.Vec3f;
import core.math.Vec3i;
import core.utils.BufferUtil;
import core.utils.Constants;
import dc.entities.MeshBuffer;
import dc.entities.MeshVertex;
import dc.entities.VoxelTypes;
import dc.svd.QefSolver;
import dc.utils.Density;
import dc.utils.VoxelHelperUtils;

import java.util.*;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static dc.ChunkOctree.log2;
import static dc.OctreeNodeType.Node_Internal;
import static dc.OctreeNodeType.Node_Leaf;

public class VoxelOctreeImpl implements VoxelOctree{

    DualContouring dualContouring;
    private boolean multiThreadCalculation;
    private float[][] densityField;

    public VoxelOctreeImpl(DualContouring dualContouring, boolean multiThreadCalculation) {
        this.dualContouring = dualContouring;
        this.multiThreadCalculation = multiThreadCalculation;
    }

    private boolean nodeIsSeam(int zi, int yi, int xi, Vec3i leafMin, int voxelPerChunk) {
        //return (leafMin.x==1016 || leafMin.x==1024) && (leafMin.z < -395 && leafMin.z >-433); //show path of seams to debug
        return xi==0||xi==voxelPerChunk-1 || yi==0||yi==voxelPerChunk-1 || zi==0||zi==voxelPerChunk-1;
    }

    @Override
    public EnumMap<VoxelTypes, List<OctreeNode>> createLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                                                      int voxelsPerChunk, int clipmapLeafSize, int leafSizeScale, float[][] densityField) {
        this.densityField = densityField;
        if (multiThreadCalculation) {
            try {
                return multiThreadCreateLeafVoxelNodes(chunkSize, chunkMin, voxelsPerChunk, clipmapLeafSize, leafSizeScale);
            } catch (Exception e) {
                e.printStackTrace();
            }
            return null;
        } else {
            return simpleDebugCreateLeafVoxelNodes(chunkSize, chunkMin, voxelsPerChunk, clipmapLeafSize, leafSizeScale);
        }
    }

    public EnumMap<VoxelTypes, List<OctreeNode>> simpleDebugCreateLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                                                      int voxelsPerChunk, int clipmapLeafSize, int leafSizeScale) {
        List<OctreeNode> voxels = new ArrayList<>();
        List<OctreeNode> seamNodes = new ArrayList<>();
        Vec3i faces = new Vec3i();
        for (int zi = 0; zi < voxelsPerChunk; zi++) {
            for (int yi = 0; yi < voxelsPerChunk; yi++) {
                for (int xi = 0; xi < voxelsPerChunk; xi++) {
                    faces.set(0, 0, 0);
                    // checks which side this node is facing
                    if (xi == 0)
                        faces.x = -1;
                    else if (xi == voxelsPerChunk-1)
                        faces.x = 1;

                    if (yi == 0)
                        faces.y = -1;
                    else if (yi == voxelsPerChunk-1)
                        faces.y = 1;

                    if (zi == 0)
                        faces.z = -1;
                    else if (zi == voxelsPerChunk-1)
                        faces.z = 1;

                    int leafSize = (chunkSize / voxelsPerChunk);
                    Vec3i leafMin = new Vec3i(xi, yi, zi).mul(leafSize).add(chunkMin);
                    OctreeNode leaf = ConstructLeaf(new OctreeNode(leafMin, leafSize, chunkMin, chunkSize), faces, leafSizeScale);
                    if (leaf != null) {
                        if(!leaf.drawInfo.color.equals(Constants.Blue)) {
                            voxels.add(leaf);
                        }
                        if(nodeIsSeam(zi, yi, xi, leafMin, voxelsPerChunk)) {
                            seamNodes.add(leaf);
                        }
                    }
                }
            }
        }
        EnumMap<VoxelTypes, List<OctreeNode>> res = new EnumMap<>(VoxelTypes.class);
        res.put(VoxelTypes.NODES, voxels);
        res.put(VoxelTypes.SEAMS, seamNodes);
        return res;
    }

    public EnumMap<VoxelTypes, List<OctreeNode>> createPathLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                                                          int voxelsPerChunk, int clipmapLeafSize, int leafSizeScale,
                                                                          int from, int to) {
        List<OctreeNode> voxels = new ArrayList<>();
        List<OctreeNode> seamNodes = new ArrayList<>();
        Vec3i chunkBorders = new Vec3i();
        for (int i=from; i < to; i++){
            int indexShift = log2(voxelsPerChunk); // max octree depth
            int x = (i >> (indexShift * 0)) & voxelsPerChunk-1;
            int y = (i >> (indexShift * 1)) & voxelsPerChunk-1;
            int z = (i >> (indexShift * 2)) & voxelsPerChunk-1;

            // checks which side this node is facing
            chunkBorders.x = (x == 0) ? -1 : (x == voxelsPerChunk-1 ? 1 : 0);
            chunkBorders.y = (y == 0) ? -1 : (y == voxelsPerChunk-1 ? 1 : 0);
            chunkBorders.z = (z == 0) ? -1 : (z == voxelsPerChunk-1 ? 1 : 0);

            int leafSize = (chunkSize / clipmapLeafSize) * leafSizeScale;
            Vec3i leafMin = new Vec3i(x, y, z).mul(leafSize).add(chunkMin);
            OctreeNode leaf = ConstructLeaf(new OctreeNode(leafMin, leafSize, chunkMin, chunkSize), chunkBorders, leafSizeScale);
            if (leaf != null) {
                if(!leaf.drawInfo.color.equals(Constants.Blue)) {
                    voxels.add(leaf);
                }
                if(nodeIsSeam(z, y, x, leafMin, voxelsPerChunk)) {
                    seamNodes.add(leaf);
                }
            }
        }
        EnumMap<VoxelTypes, List<OctreeNode>> res = new EnumMap<>(VoxelTypes.class);
        res.put(VoxelTypes.NODES, voxels);
        res.put(VoxelTypes.SEAMS, seamNodes);
        return res;
    }

    private EnumMap<VoxelTypes, List<OctreeNode>> multiThreadCreateLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                                                                  int voxelsPerChunk, int clipmapLeafSize, int leafSizeScale) throws Exception {
        int availableProcessors = Runtime.getRuntime().availableProcessors();
        int threadBound = (voxelsPerChunk*voxelsPerChunk*voxelsPerChunk) / availableProcessors;

        ExecutorService service = Executors.newFixedThreadPool(availableProcessors);
        List<Callable<EnumMap<VoxelTypes, List<OctreeNode>>>> tasks = new ArrayList<>();
        for (int i=0; i<availableProcessors; i++){
            int finalI = i;
            Callable<EnumMap<VoxelTypes, List<OctreeNode>>> task = () -> {
                int from = finalI * threadBound;
                int to = from + threadBound;
                return createPathLeafVoxelNodes(chunkSize, chunkMin, voxelsPerChunk, clipmapLeafSize, leafSizeScale, from, to);
            };
            tasks.add(task);
        }
        List<Future<EnumMap<VoxelTypes, List<OctreeNode>>>> futures = service.invokeAll(tasks);
        service.shutdown();

        List<OctreeNode> voxels = new ArrayList<>();
        List<OctreeNode> seamNodes = new ArrayList<>();
        for (Future<EnumMap<VoxelTypes, List<OctreeNode>>> future : futures){
                EnumMap<VoxelTypes, List<OctreeNode>> map = future.get();
                voxels.addAll(map.get(VoxelTypes.NODES));
                seamNodes.addAll(map.get(VoxelTypes.SEAMS));
        }
        EnumMap<VoxelTypes, List<OctreeNode>> res = new EnumMap<>(VoxelTypes.class);
        res.put(VoxelTypes.NODES, voxels);
        res.put(VoxelTypes.SEAMS, seamNodes);
        return res;
    }

    private OctreeNode ConstructLeaf(OctreeNode leaf, Vec3i chunkBorders, int leafSizeScale) {
        int corners = 0;
        for (int i = 0; i < 8; i++) {
            Vec3f cornerPos = leaf.min.add(CHILD_MIN_OFFSETS[i].mul(leaf.size)).toVec3f();
            float density = Density.getNoise(cornerPos, densityField);
		    int material = density < 0.f ? MATERIAL_SOLID : MATERIAL_AIR;
            corners |= (material << i);
        }
        if (corners == 0 || corners == 255) {
            // to avoid holes in seams between chunks with different resolution we creating some other nodes only in seams
            //https://www.reddit.com/r/VoxelGameDev/comments/6kn8ph/dual_contouring_seam_stitching_problem/
            return tryToCreateBoundSeamPseudoNode(leaf, chunkBorders, corners, leafSizeScale);
        }

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
            if ((m1 == MATERIAL_AIR && m2 == MATERIAL_AIR) || (m1 == MATERIAL_SOLID && m2 == MATERIAL_SOLID)) {
                continue; // no zero crossing on this edge
            }
            Vec3f p1 = leaf.min.add(CHILD_MIN_OFFSETS[c1].mul(leaf.size)).toVec3f();
            Vec3f p2 = leaf.min.add(CHILD_MIN_OFFSETS[c2].mul(leaf.size)).toVec3f();
            Vec3f p = VoxelHelperUtils.ApproximateZeroCrossingPosition(p1, p2, densityField);
            Vec3f n = VoxelHelperUtils.CalculateSurfaceNormal(p, densityField);
            qef.add(p, n);
            averageNormal = averageNormal.add(n);
            edgeCount++;
        }

        Vec3f qefPosition = new Vec3f(qef.getMassPoint());
        qef.solve(qefPosition, QEF_ERROR, QEF_SWEEPS, QEF_ERROR);

        OctreeDrawInfo drawInfo = new OctreeDrawInfo();
        drawInfo.position = VoxelHelperUtils.isOutFromBounds(qefPosition, leaf.min.toVec3f(), leaf.size) ? qef.getMassPoint(): qefPosition;
        drawInfo.color = Constants.Red;//isSeamNode(drawInfo.position, leaf.rootMin, leaf.chunkSize, leaf.min, leaf.size);
        drawInfo.averageNormal = averageNormal.div((float)edgeCount);
        drawInfo.averageNormal.normalize();
        drawInfo.corners = corners;

        leaf.Type = Node_Leaf;
        leaf.drawInfo = drawInfo;
        return leaf;
    }

    private Vec3i[] EDGE_OFFSETS = {
            new Vec3i(1, 2, 0), new Vec3i(1, 0, 2),
            new Vec3i(2, 1, 0), new Vec3i(0, 1, 2),
            new Vec3i(2, 0, 1), new Vec3i(0, 2, 1),
            new Vec3i(1, 0, 0), new Vec3i(0, 1, 0), new Vec3i(0, 0, 1),
            new Vec3i(1, 2, 2), new Vec3i(2, 2, 1), new Vec3i(2, 1, 2) };

    private OctreeNode tryToCreateBoundSeamPseudoNode(OctreeNode leaf, Vec3i chunkBorders, int corners, int nodeMinSize) {
        // if it is facing no border at all or has the highest amount of detail (LOD 0) skip it and drop the node
        if ((chunkBorders.x != 0 || chunkBorders.y != 0 || chunkBorders.z != 0) && leaf.size != nodeMinSize) {
            for (int i = 0; i < 12; i++) {
                if (!(  (chunkBorders.x != 0 && chunkBorders.x + 1 == EDGE_OFFSETS[i].x) ||
                        (chunkBorders.y != 0 && chunkBorders.y + 1 == EDGE_OFFSETS[i].y) ||
                        (chunkBorders.z != 0 && chunkBorders.z + 1 == EDGE_OFFSETS[i].z))) {
                    continue;
                }
                // node size at LOD 0 = 1, LOD 1 = 2, LOD 2 = 4, LOD 3 = 8
                int x = leaf.min.x + (EDGE_OFFSETS[i].x) * leaf.size / 2;
                int y = leaf.min.y + (EDGE_OFFSETS[i].y) * leaf.size / 2;
                int z = leaf.min.z + (EDGE_OFFSETS[i].z) * leaf.size / 2;

                Vec3f nodePos = new Vec3f(x,y,z);
                float density = Density.getNoise(nodePos, densityField);
                if ((density < 0 && corners == 0) || (density >= 0 && corners == 255)) {
                    leaf.drawInfo = new OctreeDrawInfo();
                    leaf.drawInfo.position = nodePos;
                    leaf.drawInfo.averageNormal = VoxelHelperUtils.CalculateSurfaceNormal(nodePos, densityField);
                    leaf.drawInfo.corners = corners;
                    leaf.drawInfo.color = Constants.Blue;
                    leaf.Type = Node_Leaf;
                    return leaf;
                }
            }
        }
        return null;    // voxel is full inside or outside the volume
    }

    private Vec3f isSeamNode(Vec3f pos, Vec3i chunkMin, int chunkSize){
        //pos.Z == chunkMin.z+(chunkSize/100)
        if ((pos.X == chunkMin.x || pos.X == chunkMin.x + chunkSize) || (pos.Z == chunkMin.z || pos.Z == chunkMin.z + chunkSize))
            return new Vec3f(0.f, 0.7f, 0.f);
        else
            return new Vec3f(0.7f, 0.f, 0.f);
    }

    private List<OctreeNode> constructParents(List<OctreeNode> nodes, Vec3i rootMin, int parentSize, int chunkSize) {
        Map<Vec3i, OctreeNode> parentsHash = new HashMap<>();
        for (OctreeNode node : nodes) {
            Vec3i localPos = node.min.sub(rootMin);
            Vec3i parentPos = node.min.sub(new Vec3i(localPos.x % parentSize, localPos.y % parentSize, localPos.z % parentSize));
            OctreeNode parent = parentsHash.get(parentPos);
            if (parent == null) {
                parent = new OctreeNode();
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

    @Override
    public OctreeNode constructTreeUpwards(List<OctreeNode> inputNodes, Vec3i rootMin, int rootNodeSize) {
        List<OctreeNode> sortedNodes = new ArrayList<>(inputNodes);
        sortedNodes.sort(Comparator.comparingInt((OctreeNode lhs) -> lhs.size));
        while (sortedNodes.get(0).size != sortedNodes.get(sortedNodes.size() - 1).size) {
            int iter = 0;
            int size = sortedNodes.get(iter).size;
            do {
                ++iter;
            } while (sortedNodes.get(iter).size == size);

            List<OctreeNode> newNodes = constructParents(sortedNodes.subList(0, iter), rootMin, size * 2, rootNodeSize);
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

    private void GenerateVertexIndices(OctreeNode node, List<MeshVertex> vertexBuffer) {
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

    @Override
    public MeshBuffer GenerateMeshFromOctree(OctreeNode node, boolean isSeam) {
        if (node == null) {
            return null;
        }
        List<MeshVertex> vertices = new ArrayList<>();
        List<Integer> indcies = new ArrayList<>();
        GenerateVertexIndices(node, vertices);
        dualContouring.ContourCellProc(node, indcies, isSeam);
        MeshBuffer buffer = new MeshBuffer(BufferUtil.createDcFlippedBufferAOS(vertices), BufferUtil.createFlippedBuffer(indcies),
                vertices.size(), indcies.size());
        vertices.clear();
        indcies.clear();
        return buffer;
    }

    public void DestroyOctree(OctreeNode node) {
        if (node == null) {
            return;
        }
        for (int i = 0; i < 8; i++) {
            DestroyOctree(node.children[i]);
        }

        if (node.drawInfo != null) {
            node.drawInfo = null;
        }
    }
}