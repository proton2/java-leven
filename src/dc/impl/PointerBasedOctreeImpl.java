package dc.impl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.math.Vec4i;
import core.utils.Constants;
import dc.*;
import dc.entities.MeshBuffer;
import dc.entities.VoxelTypes;
import dc.solver.LevenQefSolver;
import dc.solver.QEFData;
import dc.utils.VoxelHelperUtils;

import java.util.*;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static dc.OctreeNodeType.Node_Leaf;
import static dc.utils.SimplexNoise.getNoise;
import static java.lang.Math.max;

public class PointerBasedOctreeImpl extends AbstractDualContouring implements VoxelOctree {
    private final boolean multiThreadCalculation;

    public PointerBasedOctreeImpl(boolean multiThreadCalculation, MeshGenerationContext meshGenerationContext, ICSGOperations csgOperations,
                                  Map<Vec4i, GpuOctree> octreeCache) {
        super(meshGenerationContext, csgOperations, octreeCache, null);
        this.multiThreadCalculation = multiThreadCalculation;
    }

    private boolean nodeIsSeam(int zi, int yi, int xi) {
        //return (leafMin.x==1016 || leafMin.x==1024) && (leafMin.z < -395 && leafMin.z >-433); //show path of seams to debug
        return xi==0||xi==meshGen.getVoxelsPerChunk()-1 || yi==0||yi==meshGen.getVoxelsPerChunk()-1 || zi==0||zi==meshGen.getVoxelsPerChunk()-1;
    }

    @Override
    public boolean createLeafVoxelNodes(ChunkNode node, List<OctreeNode> seamNodes, MeshBuffer meshBuffer) {
        boolean result;
        List<OctreeNode> chunkNodes = new ArrayList<>();

        if (multiThreadCalculation) {
            try {
                result = multiThreadCreateLeafVoxelNodes(node.size, node.min, chunkNodes, seamNodes);
            } catch (Exception e) {
                result = false;
            }
        } else {
            result = simpleDebugCreateLeafVoxelNodes(node.size, node.min, chunkNodes, seamNodes);
        }
        if (!result){
            return false;
        }
        processNodesToMesh(chunkNodes, node.min, node.size, false, meshBuffer);
        return true;
    }

    private boolean simpleDebugCreateLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                                   List<OctreeNode> chunkNodes, List<OctreeNode> seamNodes) {
        Set<Integer> seamNodeCodes = new HashSet<>();
        for (int zi = 0; zi < meshGen.getVoxelsPerChunk(); zi++) {
            for (int yi = 0; yi < meshGen.getVoxelsPerChunk(); yi++) {
                for (int xi = 0; xi < meshGen.getVoxelsPerChunk(); xi++) {
                    Vec3i pos = new Vec3i(xi, yi, zi);
                    int leafSize = (chunkSize / meshGen.getVoxelsPerChunk());
                    Vec3i leafMin = pos.mul(leafSize).add(chunkMin);
                    PointerBasedOctreeNode leaf = ConstructLeaf(new PointerBasedOctreeNode(leafMin, leafSize, Node_Leaf), pos, meshGen.leafSizeScale);
                    if (leaf != null) {
                        if(!leaf.drawInfo.color.equals(Constants.Blue)) {
                            chunkNodes.add(leaf);
                        }
                        if(nodeIsSeam(zi, yi, xi)) {
                            seamNodes.add(leaf);
                            if (leaf.size > meshGen.leafSizeScale) {
                                seamNodeCodes.add(meshGen.codeForPosition(leaf.nodeNum));
                            }
                        }
                    }
                }
            }
        }
        List<OctreeNode> addedNodes = findAndCreateBorderNodes(seamNodeCodes, chunkMin, chunkSize / meshGen.getVoxelsPerChunk());
        seamNodes.addAll(addedNodes);
        return !chunkNodes.isEmpty() && !seamNodes.isEmpty();
    }

    private EnumMap<VoxelTypes, List<OctreeNode>> createPathLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                                                                      int from, int to) {
        List<OctreeNode> voxels = new ArrayList<>();
        List<OctreeNode> seamNodes = new ArrayList<>();
        Set<Integer> seamNodeCodes = new HashSet<>();
        for (int i=from; i < to; i++){
            int indexShift = VoxelHelperUtils.log2(meshGen.getVoxelsPerChunk()); // max octree depth
            int x = (i >> (indexShift * 0)) & meshGen.getVoxelsPerChunk()-1;
            int y = (i >> (indexShift * 1)) & meshGen.getVoxelsPerChunk()-1;
            int z = (i >> (indexShift * 2)) & meshGen.getVoxelsPerChunk()-1;

            Vec3i pos = new Vec3i(x,y,z);
            int leafSize = (chunkSize / meshGen.clipmapLeafSize) * meshGen.leafSizeScale;
            Vec3i leafMin = pos.mul(leafSize).add(chunkMin);
            PointerBasedOctreeNode leaf = ConstructLeaf(new PointerBasedOctreeNode(leafMin, leafSize, Node_Leaf), pos, meshGen.leafSizeScale);
            if (leaf != null) {
                if(!leaf.drawInfo.color.equals(Constants.Blue)) {
                    voxels.add(leaf);
                }
                if(nodeIsSeam(z, y, x)) {
                    seamNodes.add(leaf);
                    if (leaf.size > meshGen.leafSizeScale) {
                        seamNodeCodes.add(meshGen.codeForPosition(leaf.nodeNum));
                    }
                }
            }
        }
        List<OctreeNode> addedNodes = findAndCreateBorderNodes(seamNodeCodes, chunkMin, chunkSize / meshGen.getVoxelsPerChunk());
        seamNodes.addAll(addedNodes);

        EnumMap<VoxelTypes, List<OctreeNode>> res = new EnumMap<>(VoxelTypes.class);
        res.put(VoxelTypes.NODES, voxels);
        res.put(VoxelTypes.SEAMS, seamNodes);
        return res;
    }

    private boolean multiThreadCreateLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                                    List<OctreeNode> chunkNodes,
                                                    List<OctreeNode> seamNodes) throws Exception {
        int availableProcessors = max(1, Runtime.getRuntime().availableProcessors() / 2);
        int threadBound = (meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()) / availableProcessors;

        ExecutorService service = Executors.newFixedThreadPool(availableProcessors);
        List<Callable<EnumMap<VoxelTypes, List<OctreeNode>>>> tasks = new ArrayList<>();
        for (int i=0; i<availableProcessors; i++){
            int finalI = i;
            Callable<EnumMap<VoxelTypes, List<OctreeNode>>> task = () -> {
                int from = finalI * threadBound;
                int to = from + threadBound;
                return createPathLeafVoxelNodes(chunkSize, chunkMin, from, to);
            };
            tasks.add(task);
        }
        List<Future<EnumMap<VoxelTypes, List<OctreeNode>>>> futures = service.invokeAll(tasks);
        service.shutdown();

        for (Future<EnumMap<VoxelTypes, List<OctreeNode>>> future : futures){
                EnumMap<VoxelTypes, List<OctreeNode>> map = future.get();
                chunkNodes.addAll(map.get(VoxelTypes.NODES));
                seamNodes.addAll(map.get(VoxelTypes.SEAMS));
        }
        return !chunkNodes.isEmpty() && !seamNodes.isEmpty();
    }

    private PointerBasedOctreeNode ConstructLeaf(PointerBasedOctreeNode leaf, Vec3i pos, int leafSizeScale) {
        int corners = 0;
        for (int i = 0; i < 8; i++) {
            Vec3i cornerPos = leaf.min.add(CHILD_MIN_OFFSETS[i].mul(leaf.size));
            float density = getNoise(cornerPos);
		    int material = density < 0.f ? meshGen.MATERIAL_SOLID : meshGen.MATERIAL_AIR;
            corners |= (material << i);
        }
        if (corners == 0 || corners == 255) {
           return null;
        }

        // otherwise the voxel contains the surface, so find the edge intersections
	    int MAX_CROSSINGS = 6;
        int edgeCount = 0;
        Vec3f averageNormal = new Vec3f(0.f);
        Vec4f[] edgePositions = new Vec4f[12];
        Vec4f[] edgeNormals = new Vec4f[12];
        QEFData qef = new QEFData(new LevenQefSolver());
        for (int i = 0; i < 12 && edgeCount < MAX_CROSSINGS; i++) {
		    int c1 = edgevmap[i][0];
		    int c2 = edgevmap[i][1];
		    int m1 = (corners >> c1) & 1;
		    int m2 = (corners >> c2) & 1;
            if ((m1 == meshGen.MATERIAL_AIR && m2 == meshGen.MATERIAL_AIR) || (m1 == meshGen.MATERIAL_SOLID && m2 == meshGen.MATERIAL_SOLID)) {
                continue; // no zero crossing on this edge
            }
            Vec3f p1 = leaf.min.add(CHILD_MIN_OFFSETS[c1].mul(leaf.size)).toVec3f();
            Vec3f p2 = leaf.min.add(CHILD_MIN_OFFSETS[c2].mul(leaf.size)).toVec3f();
            Vec4f p = ApproximateZeroCrossingPosition(p1, p2);
            Vec4f n = CalculateSurfaceNormal(p);
            edgePositions[edgeCount] = p;
            edgeNormals[edgeCount] = n;
            averageNormal = averageNormal.add(n.getVec3f());
            edgeCount++;
        }
        qef.qef_create_from_points(edgePositions, edgeNormals, edgeCount);

        OctreeDrawInfo drawInfo = new OctreeDrawInfo();
        if(leaf.size == meshGen.leafSizeScale) {
            drawInfo.position = qef.solve().getVec3f();
        } else {
            drawInfo.position = qef.getMasspoint().getVec3f();// for other LOD's get masspoint - to increase performance
        }
        drawInfo.color = Constants.Red;
        drawInfo.averageNormal = averageNormal.div((float)edgeCount);
        drawInfo.averageNormal.normalize();
        leaf.corners = corners;
        leaf.Type = Node_Leaf;
        leaf.drawInfo = drawInfo;
        leaf.nodeNum = pos;
        return leaf;
    }
}