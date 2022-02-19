package dc;

import core.kernel.Camera;
import core.math.Vec2f;
import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4i;
import core.physics.Physics;
import core.physics.WorldCollisionNode;
import dc.entities.MeshBuffer;
import dc.impl.MeshGenerationContext;
import dc.impl.Morton3D;
import dc.impl.simplifier.MeshSimplificationOptions;
import dc.impl.simplifier.NgMeshSimplify;
import dc.utils.Aabb;
import dc.utils.Frustum;
import dc.utils.Ray;
import dc.utils.VoxelHelperUtils;

import java.text.NumberFormat;
import java.util.*;
import java.util.logging.Logger;

import static dc.utils.VoxelHelperUtils.checkNodeForSelection;

public class ChunkOctree {
    final public static Logger logger = Logger.getLogger(ChunkOctree.class.getName());
    private ChunkNode root;
    private final Camera camera;
    private final VoxelOctree voxelOctree;
    private final MeshGenerationContext meshGen;
    private List<RenderMesh> renderMeshes;
    private List<RenderMesh> invalidateMeshes;
    private List<ChunkNode> currActiveNodes;
    private final Physics physics;
    private static final NumberFormat INT_FORMATTER = NumberFormat.getIntegerInstance();
    private ArrayList<ChunkNode> prevSelectedNodes;
    private final Map<Long, ChunkNode> mortonCodesChunksMap;
    private final boolean useMeshSimplifier = false;

    public Vec3f getRayCollisionPos(){
        return physics.getCollisionPos();
    }

    public List<RenderMesh> getInvalidateMeshes() {
        return invalidateMeshes;
    }

    public ChunkOctree(VoxelOctree voxelOctree, MeshGenerationContext meshGen, Physics ph, Camera cam,
                       boolean enablePhysics, Map<Long, ChunkNode> chunks) {
        this.mortonCodesChunksMap = chunks;
        this.meshGen = meshGen;
        this.physics = ph;
        this.voxelOctree = voxelOctree;
        this.camera = cam;
        buildChunkOctree();
        update();
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
        mortonCodesChunksMap.put(root.chunkCode, root);
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
            child.chunkCode = Morton3D.codeForPosition(child.min, child.size, meshGen.worldSizeXZ/2);
            ChunkNode v = mortonCodesChunksMap.put(child.chunkCode, child);
            if(v!=null){
                throw new IllegalStateException("Incorrect linear three state - Morton code collision!");
            }
        }
        int count = 1;
        for (int i = 0; i < 8; i++) {
            long locCodeChild = (node.chunkCode<<3)|i;
            ChunkNode child = mortonCodesChunksMap.get(locCodeChild);
            if(child==null){
                throw new IllegalStateException("Incorrect linear three state - children not found!");
            }
            long parentCode = child.chunkCode>>3;
            ChunkNode parent = mortonCodesChunksMap.get(parentCode);
            if(!node.equals(parent)){
                throw new IllegalStateException("Incorrect linear three state - parent not found!");
            }
            count += constructChildrens(child);
        }
        return count;
    }

    private void selectActiveChunkNodes(ChunkNode node, boolean parentActive, Vec3f camPos, ArrayList<ChunkNode> selectedNodes){
        if (node==null || parentActive) {
            return;
        }
        node.canBeSelected = checkNodeForSelection(node, camPos) || node.size==meshGen.clipmapLeafSize;
        if (node.canBeSelected){
            selectedNodes.add(node);
        }
        for (int i = 0; i < 8; i++) {
            long locCodeChild = (node.chunkCode<<3)|i;
            ChunkNode child = mortonCodesChunksMap.get(locCodeChild);
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
                    voxelOctree.computeFreeChunkOctree(node.min, node.size);
                    ReleaseClipmapNodeData(node, invalidatedMeshes);
                    node.invalidated = false;
                }
            }
        }
        return invalidatedMeshes;
    }

    public void clean(){
        physics.Physics_Shutdown();
    }

    public void update() {
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
                ChunkNode candidateNeighbour = mortonCodesChunksMap.get(neighbourMortonCode); //findNode(root, constructedNode.size, neighbourMin);
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
        this.currActiveNodes = activeNodes;
    }

    public List<ChunkNode> getRayIntersected(Ray ray){
        ArrayList<ChunkNode> selectedChunks = new ArrayList<>();
        for(ChunkNode chunk : currActiveNodes){
            if(chunk.active && VoxelHelperUtils.intersectRayAab(ray, new Aabb(chunk.min, chunk.size), new Vec2f())){
                selectedChunks.add(chunk);
            }
        }
        return selectedChunks;
    }

    private void generateClipmapSeamMesh(ChunkNode node, ChunkNode root){
        Set<OctreeNode> seamNodes = new HashSet<>(2048);
        for (int i = 0; i < 8; i++) {
            Vec3i neighbourMin = node.min.add(VoxelOctree.CHILD_MIN_OFFSETS[i].mul(node.size));
            //Vec3i neighbourMin = node.min.add(meshGen.offset(i, node.size));
            long neighbourMortonCode = Morton3D.codeForPosition(neighbourMin, node.size, meshGen.worldSizeXZ/2);
            ChunkNode candidateNeighbour = mortonCodesChunksMap.get(neighbourMortonCode);//findNode(root, node.size, neighbourMin);
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
                    ChunkNode child = mortonCodesChunksMap.get(locCodeChild);
                    findActiveNodes(child, referenceNode, neighbourActiveNodes);
                }
            }
        }
    }

    private boolean filterNodesForDebug(ChunkNode node){
        return  //(node.size == 32 && node.min.equals(new Vec3i(32, 0, -1408))) ||
                (node.size == 64 && node.min.equals(new Vec3i(0,-64,-1408))) || // родитель чанка под дырой
                (node.size == 32 && node.min.equals(new Vec3i(32, -32, -1408)));// || // чанк 32 под дырой
    }

    private List<RenderMesh> getRenderMeshes(List<ChunkNode> chunkNodes){
        List<RenderMesh> renderMeshes = new ArrayList<>(chunkNodes.size());
        for(ChunkNode node : chunkNodes){
            if(!node.empty && node.active) {
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

        if(useMeshSimplifier) {
            Vec4i centrePos = new Vec4i(chunk.min.add(new Vec3i(chunk.size / 2)), 0);
            float leafSize = meshGen.leafSizeScale * (chunk.size / meshGen.clipmapLeafSize);
            MeshSimplificationOptions options = new MeshSimplificationOptions();
            options.maxError = 5.f * leafSize;
            options.maxEdgeSize = 2.5f * leafSize;
            options.minAngleCosine = 0.7f;
            NgMeshSimplify simplifyer = new NgMeshSimplify();
            simplifyer.ngMeshSimplifier(meshBuffer, centrePos, options);
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
            ChunkNode child = mortonCodesChunksMap.get(locCodeChild);
            propagateEmptyStateDownward(child);
        }
    }
}