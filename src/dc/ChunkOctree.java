package dc;

import core.kernel.Camera;
import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.physics.Physics;
import core.physics.WorldCollisionNode;
import dc.entities.CSGOperationInfo;
import dc.entities.MeshBuffer;
import dc.impl.MeshGenerationContext;
import dc.impl.Morton3D;
import dc.utils.Aabb;
import dc.utils.Frustum;
import dc.utils.RenderShape;
import dc.utils.VoxelHelperUtils;

import java.text.NumberFormat;
import java.util.*;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.logging.Logger;

public class ChunkOctree {
    final public static Logger logger = Logger.getLogger(ChunkOctree.class.getName());
    private final ExecutorService service;
    private ChunkNode root;
    private Camera camera;
    private final VoxelOctree voxelOctree;
    private final MeshGenerationContext meshGen;
    private List<RenderMesh> renderMeshes;
    private List<RenderMesh> invalidateMeshes;
    private ConcurrentLinkedDeque<CSGOperationInfo> g_operationQueue = new ConcurrentLinkedDeque<>();
    private final Physics physics;
    private static final NumberFormat INT_FORMATTER = NumberFormat.getIntegerInstance();
    private ArrayList<ChunkNode> prevSelectedNodes;
    private final Map<Long, ChunkNode> chunks;
    public Vec3f getRayCollisionPos(){
        return physics.getCollisionPos();
    }

    public List<RenderMesh> getInvalidateMeshes() {
        return invalidateMeshes;
    }

    public ChunkOctree(VoxelOctree voxelOctree, MeshGenerationContext meshGen, Physics physics, Camera cam,
                       boolean enablePhysics, Map<Long, ChunkNode> chunks) {
        this.chunks = chunks;
        this.meshGen = meshGen;
        this.physics = physics;
        this.voxelOctree = voxelOctree;
        service = Executors.newSingleThreadExecutor();
        buildChunkOctree();
        update(cam);
        if(enablePhysics) {
            physics.Physics_SpawnPlayer(cam.getPosition());
        }
    }

    public List<RenderMesh> getRenderMeshes(boolean frustumCulling) {
        if (!frustumCulling){
            return renderMeshes;
        } else {
            List<RenderMesh> meshes = new ArrayList<>();
            if(renderMeshes!=null) {
                for (RenderMesh renderMesh : renderMeshes) {
                    Aabb aabb = new Aabb(renderMesh.min, renderMesh.size);
                    if (Frustum.getFrustum().AABBInsideFrustum(aabb)) {
                        meshes.add(renderMesh);
                    }
                }
            }
            return meshes;
        }
    }

    private void buildChunkOctree() {
        Vec3i worldBoundsMin = meshGen.worldOrigin.sub(meshGen.worldSize.div(2));
        Vec3i worldBoundsMax = meshGen.worldOrigin.add(meshGen.worldSize.div(2));
        Vec3i boundsSize = worldBoundsMax.sub(worldBoundsMin);
        Vec3i boundsCentre = worldBoundsMin.add(boundsSize.div(2));
        float maxSize = Math.max(boundsSize.x, Math.max(boundsSize.y, boundsSize.z));

        int factor = (int) maxSize / meshGen.clipmapLeafSize;
        factor = 1 << VoxelHelperUtils.log2(factor);
        while ((factor * meshGen.clipmapLeafSize) < maxSize) {
            factor *= 2;
        }

        root = new ChunkNode();
        root.size = factor * meshGen.clipmapLeafSize;
        root.min = boundsCentre.sub(new Vec3i(root.size / 2));
        root.min.set(root.min.x & ~(factor), root.min.y & ~(factor), root.min.z & ~(factor));
        root.worldNode = new WorldCollisionNode();
        root.chunkCode = Morton3D.codeForPosition(root.min, root.size, meshGen.worldSizeXZ/2);
        chunks.put(root.chunkCode, root);
        int count = constructChildrens(root);
    }

    private int constructChildrens(ChunkNode node) {
        if (node.size == meshGen.clipmapLeafSize) {
            return 1;
        }
        for (int i = 0; i < 8; i++) {
            ChunkNode child = new ChunkNode();
            child.size = node.size / 2;
            child.min = node.min.add(meshGen.offset(i, child.size));
            child.worldNode = new WorldCollisionNode();
            //node.children[i] = child;
            child.chunkCode = Morton3D.codeForPosition(child.min, child.size, meshGen.worldSizeXZ/2);
            ChunkNode v = chunks.put(child.chunkCode, child);
            if(v!=null){
                throw new IllegalStateException("Incorrect linear three state - Morton code collision!");
            }
        }
        int count = 1;
        for (int i = 0; i < 8; i++) {
            long locCodeChild = (node.chunkCode<<3)|i;
            ChunkNode child = chunks.get(locCodeChild);
            if(child==null){
                throw new IllegalStateException("Incorrect linear three state - children not found!");
            }
            long parentCode = child.chunkCode>>3;
            ChunkNode parent = chunks.get(parentCode);
            if(!node.equals(parent)){
                throw new IllegalStateException("Incorrect linear three state - parent not found!");
            }
            count += constructChildrens(child);
        }
        return count;
    }

    private boolean checkNodeForSelection(ChunkNode node, Vec3f camPos) {
        float splitDistanceFactor = 1.5f;
        float distance = VoxelHelperUtils.ChebyshevDistance(node, camPos);
        // чанк надо разбивать, если расстояние меньше, чем размер чанка, умноженный на split_distance_factor
        boolean canBeSelected = distance > node.size * splitDistanceFactor;
        return canBeSelected;
    }

    private void selectActiveChunkNodes(ChunkNode node, boolean parentActive, Vec3f camPos, ArrayList<ChunkNode> selectedNodes){
        if (node==null || parentActive) {
            return;
        }
        if (node.canBeSelected = checkNodeForSelection(node, camPos) || node.size==meshGen.clipmapLeafSize){
            selectedNodes.add(node);
        }
        for (int i = 0; i < 8; i++) {
            long locCodeChild = (node.chunkCode<<3)|i;
            ChunkNode child = chunks.get(locCodeChild);
            selectActiveChunkNodes(child, node.canBeSelected, camPos, selectedNodes);
        }
    }

    private void ReleaseClipmapNodeData(ChunkNode node, ArrayList<RenderMesh> invalidatedMeshes) {
        node.active = false;

        if (node.renderMesh!=null) {
            invalidatedMeshes.add(node.renderMesh);
            node.renderMesh = null;
        }
        if (node.seamMesh!=null) {
            invalidatedMeshes.add(node.seamMesh);
            node.seamMesh = null;
        }
        if(node.worldNode!=null){
            if(node.worldNode.mainMesh!=null){
                physics.RemoveMeshData(node.worldNode.mainMesh);
                node.worldNode.mainMesh = null;
            }
            if(node.worldNode.seamMesh!=null){
                physics.RemoveMeshData(node.worldNode.seamMesh);
                node.worldNode.seamMesh = null;
            }
        }
    }

    private ArrayList<RenderMesh> ReleaseInvalidatedNodes(ArrayList<ChunkNode> chunkNodes){
        ArrayList<RenderMesh> invalidatedMeshes = new ArrayList<>();
        if(chunkNodes!=null) {
            for (ChunkNode node : chunkNodes) {
                if (node.invalidated || (node.active && !node.canBeSelected)) {
                    ReleaseClipmapNodeData(node, invalidatedMeshes);
                    node.invalidated = false;
                }
            }
        }
        return invalidatedMeshes;
    }

    public void update(Camera cam, Vec3f rayStart, Vec3f rayEnd){
        this.camera = cam;
        service.submit(() -> {
            try {
                update(camera);
            } catch (Throwable e){
                e.printStackTrace();
            }
        });
        physics.Physics_CastRay(rayStart, rayEnd);
    }

    public void clean(){
        service.shutdown();
        physics.Physics_Shutdown();
        voxelOctree.computeClearCSGOperations();
    }

    public void update(Camera camera) {
        ArrayList<ChunkNode> selectedNodes = new ArrayList<>();
        selectActiveChunkNodes(root, false, camera.getPosition(), selectedNodes);
        ArrayList<RenderMesh> invalidatedMeshes = ReleaseInvalidatedNodes(prevSelectedNodes);
        prevSelectedNodes = selectedNodes;

        ArrayList<ChunkNode> filteredNodes = new ArrayList<>();
        ArrayList<ChunkNode> reserveNodes = new ArrayList<>();
        ArrayList<ChunkNode> activeNodes = new ArrayList<>();
        for (ChunkNode selectedNode : selectedNodes) {
            if (!selectedNode.active && !selectedNode.empty) {
                Aabb aabb = new Aabb(selectedNode.min, selectedNode.size);
                if (Frustum.getFrustum().AABBInsideFrustum(aabb)) {
                    filteredNodes.add(selectedNode);
                } else {
                    reserveNodes.add(selectedNode);
                }
            } else {
                activeNodes.add(selectedNode);
            }
            selectedNode.canBeSelected = false;
        }
        if (filteredNodes.isEmpty()) {
            if (!reserveNodes.isEmpty()) {          // no nodes in the frustum need updated so update outside nodes
                filteredNodes = reserveNodes;
            } else {
                return; // no nodes to update so no work to do
            }
        }

        ArrayList<ChunkNode> emptyNodes = new ArrayList<>();
        ArrayList<ChunkNode> constructedNodes = new ArrayList<>();
        for (ChunkNode filteredNode : filteredNodes) {
            long time1 = System.nanoTime();
            boolean result = //filterNodesForDebug(filteredNode) &&
                    ConstructChunkNodeData(filteredNode);
            long time2 = System.nanoTime();
            if(result) {
                System.out.println("created chunk " + filteredNode + " in " + INT_FORMATTER.format((time2 - time1) / (long) 1E3));
            }

            if (filteredNode.renderMesh !=null || (filteredNode.chunkBorderNodes !=null && filteredNode.chunkBorderNodes.size()> 0)) {
                constructedNodes.add(filteredNode);
                activeNodes.add(filteredNode);
                physics.Physics_UpdateWorldNodeMainMesh(true, filteredNode);
            } else {
                filteredNode.empty = true; // no meshes in chunk - empty chunk
                emptyNodes.add(filteredNode);
            }
        }
        for (ChunkNode node : emptyNodes) {
            propagateEmptyStateDownward(node);
        }

        // -----------------------------------construct seams begin-----------------------------------
        Set<ChunkNode> seamUpdateNodes = new HashSet<>();
        // 1. for each constructed Node make list of neighbour active Nodes (or neighbour child's) - make list nodes for seam update
        for (ChunkNode constructedNode : constructedNodes) {
            for (int i = 0; i < 8; i++) {
                Vec3i neighbourMin = constructedNode.min.sub(meshGen.offset(i, constructedNode.size));
                long neighbourMortonCode = Morton3D.codeForPosition(neighbourMin, constructedNode.size, meshGen.worldSizeXZ/2);
                ChunkNode candidateNeighbour = chunks.get(neighbourMortonCode); //findNode(root, constructedNode.size, neighbourMin);
                if (candidateNeighbour!=null){
                    List<ChunkNode> neighbourActiveNodes = new ArrayList<>();
                    findActiveNodes(root, candidateNeighbour, neighbourActiveNodes);
                    seamUpdateNodes.addAll(neighbourActiveNodes);
                }
            }
        }
        for (ChunkNode seamUpdateNode : seamUpdateNodes) {
            if(seamUpdateNode.seamMesh!=null) {
                invalidatedMeshes.add(seamUpdateNode.seamMesh);
                seamUpdateNode.seamMesh = null;
            }
            generateClipmapSeamMesh(seamUpdateNode, root);
            if(seamUpdateNode.seamMesh!=null) {
                physics.Physics_UpdateWorldNodeMainMesh(false, seamUpdateNode);
            }
        }
        this.renderMeshes = getRenderMeshes(activeNodes);
        this.invalidateMeshes = invalidatedMeshes;
    }

    private void generateClipmapSeamMesh(ChunkNode node, ChunkNode root){
        Set<OctreeNode> seamNodes = new HashSet<>(2048);
        for (int i = 0; i < 8; i++) {
            Vec3i neighbourMin = node.min.add(VoxelOctree.CHILD_MIN_OFFSETS[i].mul(node.size));
            //Vec3i neighbourMin = node.min.add(meshGen.offset(i, node.size));
            long neighbourMortonCode = Morton3D.codeForPosition(neighbourMin, node.size, meshGen.worldSizeXZ/2);
            ChunkNode candidateNeighbour = chunks.get(neighbourMortonCode);//findNode(root, node.size, neighbourMin);
            if (candidateNeighbour==null) {
                continue;
            }
            List<ChunkNode> neighbourActiveNodes = new ArrayList<>();
            findActiveNodes(root, candidateNeighbour, neighbourActiveNodes);
            for(ChunkNode neighbour : neighbourActiveNodes){
                seamNodes.addAll(selectSeamNodes(node, neighbour, i));
            }
        }
        if(seamNodes.isEmpty()) {
            return;
        }
        ArrayList<OctreeNode> nodes = new ArrayList<>(seamNodes.size());
        nodes.addAll(seamNodes);
        MeshBuffer meshBuffer = new MeshBuffer();
        voxelOctree.processNodesToMesh(nodes, node.min, node.size, true, meshBuffer);
        if(meshBuffer.getNumIndicates() < 1){
            return;
        }
        node.seamMesh = new RenderMesh(node.min, node.size, meshBuffer);
    }

    private List<OctreeNode> selectSeamNodes(ChunkNode node, ChunkNode neighbour, int neighbourIndex){
        Vec3i chunkMax = node.min.add(node.size);
        Aabb aabb = new Aabb(node.min, node.size * 2);
        int neighbourScaleSize = neighbour.size / (meshGen.getVoxelsPerChunk() * meshGen.leafSizeScale);
        List<OctreeNode> selectedSeamNodes = new ArrayList<>();
        for (OctreeNode octreeSeamNode : neighbour.chunkBorderNodes) {
            Vec3i max = octreeSeamNode.min.add(neighbourScaleSize * meshGen.leafSizeScale);
            if (!filterSeamNode(neighbourIndex, chunkMax, octreeSeamNode.min, max) || !aabb.pointIsInside(octreeSeamNode.min)) {
                continue;
            }
            octreeSeamNode.setChunk(neighbour);
            selectedSeamNodes.add(octreeSeamNode);
        }
        return selectedSeamNodes;
    }

    private boolean filterSeamNode(int childIndex, Vec3i seamBounds, Vec3i min, Vec3i max) {
        switch (childIndex) {
            case 0: return max.x == seamBounds.x || max.y == seamBounds.y || max.z == seamBounds.z;
            case 1: return min.z == seamBounds.z;
            case 2: return min.y == seamBounds.y;
            case 3: return min.y == seamBounds.y || min.z == seamBounds.z;
            case 4: return min.x == seamBounds.x;
            case 5: return min.x == seamBounds.x || min.z == seamBounds.z;
            case 6: return min.x == seamBounds.x || min.y == seamBounds.y;
            case 7: return min.equals(seamBounds);
        }
        return false;
    }

    private void findActiveNodes(ChunkNode node, ChunkNode referenceNode, List<ChunkNode> neighbourActiveNodes){
        if (node == null || referenceNode == null) {
            return;
        }
        Aabb bbox = new Aabb(node.min, node.size);
	    Aabb referenceBBox = new Aabb(referenceNode.min, referenceNode.size);
        if (bbox.pointIsInside(referenceNode.min) || referenceBBox.pointIsInside(node.min)) {
            if (node.active) {
                neighbourActiveNodes.add(node);
            }
            else if (node.size > meshGen.clipmapLeafSize) {
                for (int i = 0; i < 8; i++) {
                    long locCodeChild = (node.chunkCode<<3)|i;
                    ChunkNode child = chunks.get(locCodeChild);
                    findActiveNodes(child, referenceNode, neighbourActiveNodes);
                }
            }
        }
    }

    private boolean filterNodesForDebug(ChunkNode filteredNode){
        Aabb bbox = new Aabb(filteredNode.min, filteredNode.size);
//        if (bbox.pointIsInside(Camera.getInstance().getPosition())) {
//            return true;
//        }
        if((filteredNode.size==128 && filteredNode.min.equals(new Vec3i(-384,-128,-1664)))
                || (filteredNode.size==512 && filteredNode.min.equals(new Vec3i(-512,-512,-1536)))){
            return true;
        }
        return false;
    }

    private List<RenderMesh> getRenderMeshes(List<ChunkNode> chunkNodes){
        List<RenderMesh> renderMeshes = new ArrayList<>(chunkNodes.size());
        for(ChunkNode node : chunkNodes){
            if(!node.empty) {
                if(node.renderMesh!=null) {
                    renderMeshes.add(node.renderMesh);
                }
                if(node.seamMesh!=null) {
                    renderMeshes.add(node.seamMesh);
                }
            }
        }
        return renderMeshes;
    }

    private boolean ConstructChunkNodeData(ChunkNode chunk) {
        List<OctreeNode> seamNodes = new ArrayList<>();
        MeshBuffer meshBuffer = new MeshBuffer();
        chunk.active = voxelOctree.createLeafVoxelNodes(chunk, seamNodes, meshBuffer);
        chunk.chunkBorderNodes = seamNodes;
        if(!chunk.active){
            return false;
        }

        if (meshBuffer.getNumIndicates() > 0) {
            chunk.renderMesh = new RenderMesh(chunk.min, chunk.size, meshBuffer);
        }
        return chunk.active;
    }

    private void propagateEmptyStateDownward(ChunkNode node) {
        if (node==null) {
            return;
        }
        node.empty = true;
        for (int i = 0; i < 8; i++) {
            long locCodeChild = (node.chunkCode<<3)|i;
            ChunkNode child = chunks.get(locCodeChild);
            propagateEmptyStateDownward(child);
        }
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
            service.submit(this::processCSGOperationsImpl);
        }
    }

    void processCSGOperationsImpl(){
        Set<CSGOperationInfo> operations = new HashSet<>(g_operationQueue);
        g_operationQueue.clear();

        if (operations.isEmpty()) {
            return;
        }

        Set<ChunkNode> touchedNodes = new HashSet<>();
        for (CSGOperationInfo opInfo: operations){
            touchedNodes.addAll(findNodesInsideAABB(calcCSGOperationBounds(opInfo)));
        }

        if(voxelOctree.getCsgOperationsProcessor().isReduceChunk()) {
            CSGReduceOperations(touchedNodes, operations);
        } else{
            for(ChunkNode node : touchedNodes) {
                performCSGQueueOperations(node, operations);
            }
        }

        for (CSGOperationInfo opInfo: operations) {
            voxelOctree.computeStoreCSGOperation(opInfo, calcCSGOperationBounds(opInfo));
        }
    }

    private void CSGReduceOperations(Set<ChunkNode> touchedNodes, Set<CSGOperationInfo> operations){
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
            voxelOctree.computeApplyCSGOperations(operations, node);
        }
    }

    private void performCSGQueueOperations(ChunkNode clipmapNode, Set<CSGOperationInfo> operations){
        if (clipmapNode.active) {
            voxelOctree.computeApplyCSGOperations(operations, clipmapNode);
        }
        voxelOctree.computeFreeChunkOctree(clipmapNode.min, clipmapNode.size); // free the current octree to force a reconstruction
        clipmapNode.invalidated = true;
        clipmapNode.empty = false;
    }

    private Aabb calcCSGOperationBounds(CSGOperationInfo opInfo) {
        Vec3i boundsHalfSize = (opInfo.getDimensions().mul3f(meshGen.leafSizeScale)).add(meshGen.CSG_BOUNDS_FUDGE);
        Vec3i scaledOrigin = (opInfo.getOrigin().sub(meshGen.CSG_OFFSET)).mul3i((float)meshGen.leafSizeScale);
        return new Aabb(scaledOrigin.sub(boundsHalfSize), scaledOrigin.add(boundsHalfSize));
    }

    private List<ChunkNode> findNodesInsideAABB(Aabb aabb) {
        ArrayList<ChunkNode> nodes = new ArrayList<>();
        findNodesInsideAABB(root, aabb, nodes);
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
            ChunkNode child = chunks.get(locCodeChild);
            findNodesInsideAABB(child, aabb, nodes);
        }

        // traversal order is arbitrary
        if (node.size <= meshGen.LOD_MAX_NODE_SIZE) {
            nodes.add(node);
        }
    }
}