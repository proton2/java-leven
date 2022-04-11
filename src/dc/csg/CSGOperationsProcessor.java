package dc.csg;

import core.kernel.Camera;
import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import dc.ChunkNode;
import dc.ReduceStateEnum;
import dc.VoxelOctree;
import dc.entities.CSGOperationInfo;
import dc.impl.MeshGenerationContext;
import dc.utils.Aabb;
import dc.utils.RenderShape;

import java.util.*;
import java.util.concurrent.ConcurrentLinkedDeque;

import static dc.utils.VoxelHelperUtils.checkNodeForSelection;

public class CSGOperationsProcessor {
    private final VoxelOctree voxelOctree;
    private final MeshGenerationContext meshGen;
    private final Camera camera;
    private final Map<Long, ChunkNode> mortonCodesChunksMap;
    private final ConcurrentLinkedDeque<CSGOperationInfo> g_operationQueue = new ConcurrentLinkedDeque<>();

    public CSGOperationsProcessor(VoxelOctree voxelOctree, MeshGenerationContext ctx, Camera cam, Map<Long, ChunkNode> chunks) {
        this.voxelOctree = voxelOctree;
        this.meshGen = ctx;
        this.camera = cam;
        this.mortonCodesChunksMap = chunks;
    }

    public void queueCSGOperation(Vec3f origin, Vec3f brushSize, RenderShape brushShape, int brushMaterial, boolean isAddOperation) {
        CSGOperationInfo opInfo = new CSGOperationInfo();
        opInfo.setOrigin(new Vec4f(origin.div((float)meshGen.leafSizeScale).add(meshGen.CSG_OFFSET), 0.f));
        opInfo.setDimensions(new Vec4f(brushSize.div(2.f), 0.f).div((float)meshGen.leafSizeScale));
        opInfo.setBrushShape(brushShape);
        opInfo.setMaterial(isAddOperation ? brushMaterial : meshGen.MATERIAL_AIR);
        opInfo.setType(isAddOperation ? 0 : 1);

        g_operationQueue.addLast(opInfo);
    }

    public void processCSGOperations() {
        if (!g_operationQueue.isEmpty()) {
            processCSGOperationsImpl();
        }
    }

    private void processCSGOperationsImpl(){
        Set<CSGOperationInfo> operations = new HashSet<>(g_operationQueue);
        CSGOperationInfo lastOperation = g_operationQueue.getLast();
        g_operationQueue.clear();
        if (operations.isEmpty()) {
            return;
        }

        Set<ChunkNode> touchedNodes = new HashSet<>();
        for (CSGOperationInfo opInfo: operations){
            touchedNodes.addAll(findNodesInsideAABB(calcCSGOperationBounds(opInfo)));
        }
        CSGReduceOperations(touchedNodes, lastOperation);
    }

    private void CSGReduceOperations(Set<ChunkNode> touchedNodes, CSGOperationInfo lastOperation){
        List<ChunkNode> nodes = new ArrayList<>(touchedNodes);
        nodes.sort(Comparator.comparingInt((ChunkNode lhs) -> lhs.size));
        int activeNodeNumber = 0;
        for (int i=nodes.size()-1; i>-1; i--){
            if(checkNodeForSelection(nodes.get(i), camera.getPosition()) || nodes.get(i).size==meshGen.clipmapLeafSize) {
                activeNodeNumber = i;
                break;
            }
        }
        for(ChunkNode node : nodes) {
            if(!node.active){
                node.reduceStatus = ReduceStateEnum.NEED_TO_REDUCE;
            }
            voxelOctree.computeFreeChunkOctree(node.min, node.size); // free the current octree to force a reconstruction
            node.invalidated = true;
            node.empty = false;
        }
        List<ChunkNode> subList = nodes.subList(0, activeNodeNumber+1);
        for(ChunkNode node : subList) {
            voxelOctree.computeApplyCSGOperations(lastOperation, node);
        }
    }

    private Aabb calcCSGOperationBounds(CSGOperationInfo opInfo) {
        Vec3i boundsHalfSize = (opInfo.getDimensions().mul3f(meshGen.leafSizeScale)).add(meshGen.CSG_BOUNDS_FUDGE);
        Vec3i scaledOrigin = (opInfo.getOrigin().sub(meshGen.CSG_OFFSET)).mul3i((float)meshGen.leafSizeScale);
        return new Aabb(scaledOrigin.sub(boundsHalfSize), scaledOrigin.add(boundsHalfSize));
    }

    private List<ChunkNode> findNodesInsideAABB(Aabb aabb) {
        ArrayList<ChunkNode> nodes = new ArrayList<>();
        findNodesInsideAABB(mortonCodesChunksMap.get(1L), aabb, nodes);
        return nodes;
    }

    void findNodesInsideAABB(ChunkNode node, Aabb aabb, List<ChunkNode> nodes) {
        if (node==null) {
            return;
        }

        Aabb nodeBB = new Aabb(node.min, node.size);
        if (!aabb.overlaps(nodeBB)) {
            return;
        }

        for (int i = 0; i < 8; i++) {
            long locCodeChild = (node.chunkCode<<3)|i;
            ChunkNode child = mortonCodesChunksMap.get(locCodeChild);
            findNodesInsideAABB(child, aabb, nodes);
        }

        // traversal order is arbitrary
        if (node.size <= meshGen.LOD_MAX_NODE_SIZE) {
            nodes.add(node);
        }
    }
}