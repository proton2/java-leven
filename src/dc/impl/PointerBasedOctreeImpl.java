package dc.impl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.utils.Constants;
import dc.AbstractDualContouring;
import dc.OctreeDrawInfo;
import dc.PointerBasedOctreeNode;
import dc.VoxelOctree;
import dc.entities.MeshBuffer;
import dc.entities.VoxelTypes;
import dc.solver.QefSolver;
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

public class PointerBasedOctreeImpl extends AbstractDualContouring implements VoxelOctree {
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
            Vec4f nodePos = tryToCreateBoundSeamPseudoNode(leaf.min, leaf.size, pos, corners, leafSizeScale, densityField);
            if(nodePos==null){
                return null;
            } else {
                leaf.drawInfo = new OctreeDrawInfo();
                leaf.drawInfo.position = nodePos.getVec3f();
                leaf.drawInfo.averageNormal = VoxelHelperUtils.CalculateSurfaceNormal(nodePos, densityField).getVec3f();
                leaf.drawInfo.corners = corners;
                leaf.drawInfo.color = Constants.Blue;
                leaf.Type = Node_Leaf;
                return leaf;
            }
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
            Vec4f p = VoxelHelperUtils.ApproximateZeroCrossingPosition(p1, p2, densityField);
            Vec4f n = VoxelHelperUtils.CalculateSurfaceNormal(p, densityField);
            qef.qef_add_point(p, n);
            averageNormal = averageNormal.add(n.getVec3f());
            edgeCount++;
        }

        Vec3f qefPosition = qef.solve().getVec3f();

        OctreeDrawInfo drawInfo = new OctreeDrawInfo();
        drawInfo.position = VoxelHelperUtils.isOutFromBounds(qefPosition, leaf.min.toVec3f(), leaf.size) ? qef.getMasspoint().getVec3f(): qefPosition;
        drawInfo.color = Constants.Red;//isSeamNode(drawInfo.position, leaf.rootMin, leaf.chunkSize, leaf.min, leaf.size);
        drawInfo.averageNormal = averageNormal.div((float)edgeCount);
        drawInfo.averageNormal.normalize();
        drawInfo.corners = corners;

        leaf.Type = Node_Leaf;
        leaf.drawInfo = drawInfo;
        return leaf;
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