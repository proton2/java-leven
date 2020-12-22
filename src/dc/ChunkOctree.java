package dc;

import core.kernel.Camera;
import core.math.Vec3f;
import core.math.Vec3i;
import dc.entities.MeshBuffer;
import dc.impl.GPUDensityField;
import dc.impl.MeshGenerationContext;
import dc.utils.Aabb;
import dc.utils.Frustum;
import dc.utils.SimplexNoise;
import dc.utils.VoxelHelperUtils;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class ChunkOctree {
    private final ExecutorService service;
    private ChunkNode root;
    private Camera camera;
    VoxelOctree voxelOctree;
    private final MeshGenerationContext meshGen;
    private List<RenderMesh> renderMeshes;
    private List<RenderMesh> invalidateMeshes;

    public List<RenderMesh> getInvalidateMeshes() {
        return invalidateMeshes;
    }

    public ChunkOctree(VoxelOctree voxelOctree, MeshGenerationContext meshGen) {
        this.meshGen = meshGen;
        this.voxelOctree = voxelOctree;
        service = Executors.newFixedThreadPool(1);
        buildChunkOctree();
        update(Camera.getInstance());
    }

    public List<RenderMesh> getRenderMeshes(boolean frustumCulling) {
        if (!frustumCulling){
            return renderMeshes;
        } else {
            List<RenderMesh> meshes = new ArrayList<>();
            if(renderMeshes!=null) {
                for (RenderMesh renderMesh : renderMeshes) {
                    if (Frustum.cubeIntoFrustum(camera.getFrustumPlanes(), renderMesh.min, renderMesh.size)) {
                        meshes.add(renderMesh);
                    }
                }
            }
            return meshes;
        }
    }

    private void buildChunkOctree() {
        root = new ChunkNode();

        Vec3i worldSize = new Vec3i(meshGen.worldSizeXZ, meshGen.worldSizeY, meshGen.worldSizeXZ);
        Vec3i worldOrigin = new Vec3i(0);
        Vec3i worldBoundsMin = worldOrigin.sub(worldSize.div(2));
        Vec3i worldBoundsMax = worldOrigin.add(worldSize.div(2));

        Vec3i boundsSize = worldBoundsMax.sub(worldBoundsMin);
        float maxSize = Math.max(boundsSize.x, Math.max(boundsSize.y, boundsSize.z));
        int factor = (int) maxSize / meshGen.clipmapLeafSize;
        factor = 1 << VoxelHelperUtils.log2(factor);
        while ((factor * meshGen.clipmapLeafSize) < maxSize) {
            factor *= 2;
        }
        root.size = factor * meshGen.clipmapLeafSize;
        Vec3i boundsCentre = worldBoundsMin.add(boundsSize.div(2));
        root.min = boundsCentre.sub(new Vec3i(root.size / 2));
        root.min.set(root.min.x & ~(factor), root.min.y & ~(factor), root.min.z & ~(factor));

        SimplexNoise.getInstance("./res/textures/floatArray.dat", root.size, meshGen.worldSizeXZ);
        constructChildrens(root);
    }

    private void constructChildrens(ChunkNode node) {
        if (node.size == meshGen.clipmapLeafSize) {
            return;
        }
        for (int i = 0; i < 8; i++) {
            ChunkNode child = new ChunkNode();
            child.size = node.size / 2;
            child.min = node.min.add(VoxelOctree.CHILD_MIN_OFFSETS[i].mul(child.size));
            node.children[i] = child;
        }
        for (int i = 0; i < 8; i++) {
            constructChildrens(node.children[i]);
        }
    }

    private void selectActiveChunkNodes(ChunkNode node, boolean parentActive, Vec3f camPos, ArrayList<ChunkNode> selectedNodes){
        float[] LOD_ACTIVE_DISTANCES = {0.f,
                meshGen.clipmapLeafSize * 1.5f,
                meshGen.clipmapLeafSize * 3.5f,
                meshGen.clipmapLeafSize * 5.5f,
                meshGen.clipmapLeafSize * 7.5f,
                meshGen.clipmapLeafSize * 13.5f
        };
        if (node==null) {
            return;
        }
        boolean nodeActive = false;
        int lodMaxNodeSize = meshGen.clipmapLeafSize * (1 << (meshGen.numLods - 1));
        if (!parentActive && node.size <= lodMaxNodeSize) {
            int size = node.size / (meshGen.getVoxelsPerChunk() * meshGen.leafSizeScale);
            int distanceIndex = VoxelHelperUtils.log2(size);
            float d = LOD_ACTIVE_DISTANCES[distanceIndex];
            float nodeDistance = VoxelHelperUtils.DistanceToNode(node, camPos);
            if (nodeDistance >= d) {
                selectedNodes.add(node);
                nodeActive = true;
            }
        }
        if (node.active && !nodeActive) {
            node.invalidated = true;
        }
        for (int i = 0; i < 8; i++) {
            selectActiveChunkNodes(node.children[i], parentActive || nodeActive, camPos, selectedNodes);
        }
    }

    private void ReleaseInvalidatedNodes(ChunkNode node, ArrayList<RenderMesh> invalidatedMeshes) {
        if (node==null)
            return;

        if(node.invalidated){
            ReleaseClipmapNodeData(node, invalidatedMeshes);
            node.invalidated=false;
        }

        for (int i = 0; i < 8; i++) {
            ReleaseInvalidatedNodes(node.children[i], invalidatedMeshes);
        }
    }

    void ReleaseClipmapNodeData(ChunkNode node, ArrayList<RenderMesh> invalidatedMeshes) {
        node.active = false;

        if (node.renderMesh!=null) {
            invalidatedMeshes.add(node.renderMesh);
            node.renderMesh = null;
        }
        if (node.seamMesh!=null) {
            invalidatedMeshes.add(node.seamMesh);
            node.seamMesh = null;
        }
    }

    public void update(Camera cam, boolean multiTread){
        this.camera = cam;
        if (multiTread) {
            service.submit(() -> update(camera));
        } else {
            update(camera);
        }
    }

    public void clean(){
        service.shutdown();
    }

    public void update(Camera camera) {
        ArrayList<ChunkNode> selectedNodes = new ArrayList<>();
        selectActiveChunkNodes(root, false, camera.getPosition(), selectedNodes);

        ArrayList<RenderMesh> invalidatedMeshes = new ArrayList<>();
        ReleaseInvalidatedNodes(root, invalidatedMeshes);

        ArrayList<ChunkNode> filteredNodes = new ArrayList<>();
        ArrayList<ChunkNode> reserveNodes = new ArrayList<>();
        ArrayList<ChunkNode> activeNodes = new ArrayList<>();
        for (ChunkNode selectedNode : selectedNodes) {
            if (!selectedNode.active && !selectedNode.empty) {
                if (Frustum.cubeIntoFrustum(camera.getFrustumPlanes(), selectedNode.min, selectedNode.size)) {
                    filteredNodes.add(selectedNode);
                } else {
                    reserveNodes.add(selectedNode);
                }
            } else {
                activeNodes.add(selectedNode);
            }
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
            boolean result = //filterNodesForDebug(filteredNode) &&
                    ConstructChunkNodeData(filteredNode);
            if (filteredNode.renderMesh !=null || (filteredNode.chunkBorderNodes !=null && filteredNode.chunkBorderNodes.size()> 0)) {
                constructedNodes.add(filteredNode);
                activeNodes.add(filteredNode);
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
                Vec3i neighbourMin = constructedNode.min.sub(VoxelOctree.CHILD_MIN_OFFSETS[i].mul(constructedNode.size));
                ChunkNode candidateNeighbour = findNode(root, constructedNode.size, neighbourMin);
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
        }
        this.renderMeshes = getRenderMeshes(activeNodes);
        this.invalidateMeshes = invalidatedMeshes;
    }

    private void generateClipmapSeamMesh(ChunkNode node, ChunkNode root){
        Set<PointerBasedOctreeNode> seamNodes = new HashSet<>(2048);
        for (int i = 0; i < 8; i++) {
            Vec3i neighbourMin = node.min.add(VoxelOctree.CHILD_MIN_OFFSETS[i].mul(node.size));
            ChunkNode candidateNeighbour = findNode(root, node.size, neighbourMin);
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
        ArrayList<PointerBasedOctreeNode> nodes = new ArrayList<>(seamNodes.size());
        nodes.addAll(seamNodes);
        MeshBuffer meshBuffer = new MeshBuffer();
        voxelOctree.processNodesToMesh(nodes, node.min, node.size, true, meshBuffer);
        node.seamMesh = new RenderMesh(node.min, node.size, meshBuffer);
    }

    private List<PointerBasedOctreeNode> selectSeamNodes(ChunkNode node, ChunkNode neighbour, int neighbourIndex){
        Vec3i chunkMax = node.min.add(node.size);
        Aabb aabb = new Aabb(node.min, node.size * 2);
        int neighbourScaleSize = neighbour.size / (meshGen.getVoxelsPerChunk() * meshGen.leafSizeScale);
        List<PointerBasedOctreeNode> selectedSeamNodes = new ArrayList<>();
        for (PointerBasedOctreeNode octreeSeamNode : neighbour.chunkBorderNodes) {
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

    private ChunkNode findNode(ChunkNode node, int size, Vec3i min) {
        if (node==null) {
            return null;
        }
        if (node.size == size && node.min.equals(min)) {
            return node;
        }
        Aabb bbox = new Aabb(node.min, node.size);
        if (bbox.pointIsInside(min)) {
            for (int i = 0; i < 8; i++) {
                ChunkNode n = findNode(node.children[i], size, min);
                if (n!=null){
                    return n;
                }
            }
        }
        return null;
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
                    findActiveNodes(node.children[i], referenceNode, neighbourActiveNodes);
                }
            }
        }
    }



    private boolean filterNodesForDebug(ChunkNode filteredNode){
        boolean res =
//                filteredNode.min.x > -256 && filteredNode.min.x < 512 &&
//                filteredNode.min.z > -1536 && filteredNode.min.z < 128;

//                filteredNode.min.x > 2816 && filteredNode.min.x < 3584 &&
//                        filteredNode.min.z > 1024 && filteredNode.min.z < 2048
//                &&
                        (
                        //(filteredNode.min.x==3072 && filteredNode.min.y==-2048 && filteredNode.min.z==1280) ||
                        //(filteredNode.min.x==3328 && filteredNode.min.y==-2048 && filteredNode.min.z==1280) ||
                        //(filteredNode.min.x==3072 && filteredNode.min.y==-2048 && filteredNode.min.z==1536) ||
                        //(filteredNode.min.x==3072 && filteredNode.min.y==-2048 && filteredNode.min.z==1792) ||
                        (filteredNode.min.x==3328 && filteredNode.min.y==-2048 && filteredNode.min.z==1536) ||
                        (filteredNode.min.x==3072 && filteredNode.min.y==-2560 && filteredNode.min.z==1536)
                );
        return res;
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
//        if (chunk.renderMesh !=null || (chunk.borderNodes !=null && chunk.borderNodes.size()> 0)){
//            chunk.active = true;
//            return chunk.active;
//        }
        List<PointerBasedOctreeNode> seamNodes = new ArrayList<>();
        MeshBuffer meshBuffer = new MeshBuffer();
        GPUDensityField field = new GPUDensityField();
        chunk.active = voxelOctree.createLeafVoxelNodes(chunk.size, chunk.min, seamNodes, meshBuffer, field);
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
            propagateEmptyStateDownward(node.children[i]);
        }
    }
}