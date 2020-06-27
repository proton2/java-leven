package dc;

import core.math.Vec3f;
import core.math.Vec3i;
import core.utils.Constants;
import dc.entities.MeshBuffer;
import dc.entities.VoxelTypes;
import dc.svd.QefSolver;
import dc.utils.Density;
import dc.utils.VoxelHelperUtils;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static dc.ChunkOctree.VOXELS_PER_CHUNK;
import static dc.ChunkOctree.log2;
import static dc.OctreeNodeType.Node_Leaf;

public class PointerBasedOctreeImpl extends AbstractDualContouring implements VoxelOctree{
    private boolean multiThreadCalculation;
    private float[] densityField;

    public PointerBasedOctreeImpl(){
    }

    public PointerBasedOctreeImpl(boolean multiThreadCalculation) {
        this.multiThreadCalculation = multiThreadCalculation;
    }

    private boolean nodeIsSeam(int zi, int yi, int xi) {
        //return (leafMin.x==1016 || leafMin.x==1024) && (leafMin.z < -395 && leafMin.z >-433); //show path of seams to debug
        return xi==0||xi==VOXELS_PER_CHUNK-1 || yi==0||yi==VOXELS_PER_CHUNK-1 || zi==0||zi==VOXELS_PER_CHUNK-1;
    }

    @Override
    public boolean createLeafVoxelNodes(int chunkSize, Vec3i chunkMin, int voxelsPerChunk,
                                        int clipmapLeafSize, int leafSizeScale,
                                        float[] densityField,
                                        List<PointerBasedOctreeNode> seamNodes, MeshBuffer meshBuffer) {
        this.densityField = densityField;
        boolean result;
        List<PointerBasedOctreeNode> chunkNodes = new ArrayList<>();

        if (multiThreadCalculation) {
            try {
                result = multiThreadCreateLeafVoxelNodes(chunkSize, chunkMin, voxelsPerChunk, clipmapLeafSize, leafSizeScale, chunkNodes, seamNodes);
            } catch (Exception e) {
                result = false;
            }
        } else {
            result = simpleDebugCreateLeafVoxelNodes(chunkSize, chunkMin, voxelsPerChunk, clipmapLeafSize, leafSizeScale, chunkNodes, seamNodes);
        }
        if (!result){
            return false;
        }
        processNodesToMesh(chunkNodes, chunkMin, chunkSize, false, meshBuffer);
        return true;
    }

    private boolean simpleDebugCreateLeafVoxelNodes(int chunkSize, Vec3i chunkMin, int voxelsPerChunk, int clipmapLeafSize, int leafSizeScale,
                                                   List<PointerBasedOctreeNode> chunkNodes, List<PointerBasedOctreeNode> seamNodes) {
        for (int zi = 0; zi < voxelsPerChunk; zi++) {
            for (int yi = 0; yi < voxelsPerChunk; yi++) {
                for (int xi = 0; xi < voxelsPerChunk; xi++) {
                    Vec3i pos = new Vec3i(xi, yi, zi);
                    int leafSize = (chunkSize / voxelsPerChunk);
                    Vec3i leafMin = pos.mul(leafSize).add(chunkMin);
                    PointerBasedOctreeNode leaf = ConstructLeaf(new PointerBasedOctreeNode(leafMin, leafSize, chunkMin, chunkSize), pos, leafSizeScale);
                    if (leaf != null) {
                        if(!leaf.drawInfo.color.equals(Constants.Blue)) {
                            chunkNodes.add(leaf);
                        }
                        if(nodeIsSeam(zi, yi, xi)) {
                            seamNodes.add(leaf);
                        }
                    }
                }
            }
        }
        return !chunkNodes.isEmpty() && !seamNodes.isEmpty();
    }

    private EnumMap<VoxelTypes, List<PointerBasedOctreeNode>> createPathLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                                                                      int voxelsPerChunk, int clipmapLeafSize, int leafSizeScale,
                                                                                      int from, int to) {
        List<PointerBasedOctreeNode> voxels = new ArrayList<>();
        List<PointerBasedOctreeNode> seamNodes = new ArrayList<>();

        for (int i=from; i < to; i++){
            int indexShift = log2(voxelsPerChunk); // max octree depth
            int x = (i >> (indexShift * 0)) & voxelsPerChunk-1;
            int y = (i >> (indexShift * 1)) & voxelsPerChunk-1;
            int z = (i >> (indexShift * 2)) & voxelsPerChunk-1;

            Vec3i pos = new Vec3i(x,y,z);
            int leafSize = (chunkSize / clipmapLeafSize) * leafSizeScale;
            Vec3i leafMin = pos.mul(leafSize).add(chunkMin);
            PointerBasedOctreeNode leaf = ConstructLeaf(new PointerBasedOctreeNode(leafMin, leafSize, chunkMin, chunkSize), pos, leafSizeScale);
            if (leaf != null) {
                if(!leaf.drawInfo.color.equals(Constants.Blue)) {
                    voxels.add(leaf);
                }
                if(nodeIsSeam(z, y, x)) {
                    seamNodes.add(leaf);
                }
            }
        }
        EnumMap<VoxelTypes, List<PointerBasedOctreeNode>> res = new EnumMap<>(VoxelTypes.class);
        res.put(VoxelTypes.NODES, voxels);
        res.put(VoxelTypes.SEAMS, seamNodes);
        return res;
    }

    private boolean multiThreadCreateLeafVoxelNodes(int chunkSize, Vec3i chunkMin, int voxelsPerChunk,
                                                    int clipmapLeafSize, int leafSizeScale,
                                                    List<PointerBasedOctreeNode> chunkNodes,
                                                    List<PointerBasedOctreeNode> seamNodes) throws Exception {
        int availableProcessors = Runtime.getRuntime().availableProcessors();
        int threadBound = (voxelsPerChunk*voxelsPerChunk*voxelsPerChunk) / availableProcessors;

        ExecutorService service = Executors.newFixedThreadPool(availableProcessors);
        List<Callable<EnumMap<VoxelTypes, List<PointerBasedOctreeNode>>>> tasks = new ArrayList<>();
        for (int i=0; i<availableProcessors; i++){
            int finalI = i;
            Callable<EnumMap<VoxelTypes, List<PointerBasedOctreeNode>>> task = () -> {
                int from = finalI * threadBound;
                int to = from + threadBound;
                return createPathLeafVoxelNodes(chunkSize, chunkMin, voxelsPerChunk, clipmapLeafSize, leafSizeScale, from, to);
            };
            tasks.add(task);
        }
        List<Future<EnumMap<VoxelTypes, List<PointerBasedOctreeNode>>>> futures = service.invokeAll(tasks);
        service.shutdown();

        for (Future<EnumMap<VoxelTypes, List<PointerBasedOctreeNode>>> future : futures){
                EnumMap<VoxelTypes, List<PointerBasedOctreeNode>> map = future.get();
                chunkNodes.addAll(map.get(VoxelTypes.NODES));
                seamNodes.addAll(map.get(VoxelTypes.SEAMS));
        }
        return !chunkNodes.isEmpty() && !seamNodes.isEmpty();
    }

    private PointerBasedOctreeNode ConstructLeaf(PointerBasedOctreeNode leaf, Vec3i pos, int leafSizeScale) {
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
            return tryToCreateBoundSeamPseudoNode(leaf, pos, corners, leafSizeScale);
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
            Vec3f p = VoxelHelperUtils.ApproximateZeroCrossingPosition(p1, p2, densityField).getVec3f();
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
            new Vec3i(1, 2, 2), new Vec3i(2, 2, 1), new Vec3i(2, 1, 2)
    };

    protected Vec3i getChunkBorder(Vec3i pos){
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

    private PointerBasedOctreeNode tryToCreateBoundSeamPseudoNode(PointerBasedOctreeNode leaf, Vec3i pos, int corners, int nodeMinSize) {
        Vec3i chunkBorders = getChunkBorder(pos);
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

    public void DestroyOctree(PointerBasedOctreeNode node) {
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