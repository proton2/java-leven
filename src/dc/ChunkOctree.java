package dc;

import core.buffers.MeshDcVBO;
import core.configs.CW;
import core.kernel.Camera;
import core.math.Vec3f;
import core.math.Vec3i;
import core.renderer.RenderInfo;
import core.renderer.Renderer;
import core.utils.Constants;
import dc.entities.MeshBuffer;
import dc.entities.VoxelTypes;
import dc.shaders.DcSimpleShader;
import dc.utils.Aabb;
import dc.utils.Frustum;
import dc.utils.RenderDebugCmdBuffer;
import dc.utils.VoxelHelperUtils;

import java.util.*;

public class ChunkOctree {
    static int LEAF_SIZE_LOG2 = 2;
    public static int LEAF_SIZE_SCALE = 1 << LEAF_SIZE_LOG2;
    public static int VOXELS_PER_CHUNK = 64;
    static int CLIPMAP_LEAF_SIZE = LEAF_SIZE_SCALE * VOXELS_PER_CHUNK;

    int NUM_LODS = 6;
    int LOD_MAX_NODE_SIZE = CLIPMAP_LEAF_SIZE * (1 << (NUM_LODS - 1));

    int worldBrickCountXZ = 4;//2
    int BRICK_SIZE = 8;
    int worldSizeXZ = worldBrickCountXZ * BRICK_SIZE * CLIPMAP_LEAF_SIZE;
    int worldSizeY = 4 * BRICK_SIZE * CLIPMAP_LEAF_SIZE;
    Vec3i worldSize = new Vec3i(worldSizeXZ, worldSizeY, worldSizeXZ);
    Vec3i worldOrigin = new Vec3i(0);
    Vec3i worldBoundsMin = worldOrigin.sub(worldSize.div(2));
    Vec3i worldBoundsMax = worldOrigin.add(worldSize.div(2));

    private float[] LOD_ACTIVE_DISTANCES = {0.f,
            CLIPMAP_LEAF_SIZE * 1.5f,
            CLIPMAP_LEAF_SIZE * 3.5f,
            CLIPMAP_LEAF_SIZE * 5.5f,
            CLIPMAP_LEAF_SIZE * 7.5f,
            CLIPMAP_LEAF_SIZE * 13.5f
    };

    VoxelOctree voxelOctree;

    public ChunkOctree(VoxelOctree voxelOctree) {
        this.voxelOctree = voxelOctree;
    }

    public static int log2(int N) {
        return (int) (Math.log(N) / Math.log(2));
    }

    public ChunkNode buildChunkOctree() {
        ChunkNode root = new ChunkNode();
        Vec3i boundsSize = worldBoundsMax.sub(worldBoundsMin);
        float maxSize = Math.max(boundsSize.x, Math.max(boundsSize.y, boundsSize.z));
        int factor = (int) maxSize / CLIPMAP_LEAF_SIZE;
        factor = 1 << log2(factor);
        while ((factor * CLIPMAP_LEAF_SIZE) < maxSize) {
            factor *= 2;
        }
        root.size = factor * CLIPMAP_LEAF_SIZE;
        Vec3i boundsCentre = worldBoundsMin.add(boundsSize.div(2));
        //root.min = (boundsCentre.sub(new Vec3f(root.size / 2))) & ~(factor - 1);
        root.min = boundsCentre.sub(new Vec3i(root.size / 2));
        //root.min.set(root.min.x & ~(factor - 1), root.min.y & ~(factor - 1), root.min.z & ~(factor - 1));
        root.min.set(root.min.x & ~(factor), root.min.y & ~(factor), root.min.z & ~(factor));

        constructChildrens(root);
        return root;
    }

    private void constructChildrens(ChunkNode node) {
        if (node.size == CLIPMAP_LEAF_SIZE) {
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
        if (node==null) {
            return;
        }
        boolean nodeActive = false;
        if (!parentActive && node.size <= LOD_MAX_NODE_SIZE) {
            int size = node.size / (VOXELS_PER_CHUNK * LEAF_SIZE_SCALE);
            int distanceIndex = log2(size);
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

    private void ReleaseInvalidatedNodes(ChunkNode node, ArrayList<Renderer> invalidatedMeshes) {
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

    void ReleaseClipmapNodeData(ChunkNode node, ArrayList<Renderer> invalidatedMeshes) {
        node.active = false;
        /*
        if (node.renderMesh !=null) {
            // collect the invalidated mesh indices so the meshes can be removed after
            // the replacement mesh(es) have been generated, which prevents flickering
            invalidatedMeshes.add(node.renderMesh);
            node.renderMesh.cleanMesh();
        }
        */
        if (node.seamMesh!=null) {
            invalidatedMeshes.add(node.seamMesh);
            node.seamMesh.cleanMesh();
            node.seamMesh = null;
        }
        /*
        for (int i = 0; i < node.numSeamNodes; i++) {
            OctreeNode n = node.seamNodes.get(i);
            n.drawInfo = null;
        }
        node.seamNodes.clear();
        node.seamNodes = null;
        node.numSeamNodes = 0;
         */
    }

    public ArrayList<ChunkNode> update(ChunkNode root, Camera camera, RenderDebugCmdBuffer renderCmds) {
        ArrayList<ChunkNode> selectedNodes = new ArrayList<>();
        selectActiveChunkNodes(root, false, camera.getPosition(), selectedNodes);

        ArrayList<Renderer> invalidatedMeshes = new ArrayList<>();
        ReleaseInvalidatedNodes(root, invalidatedMeshes);

        ArrayList<ChunkNode> filteredNodes = new ArrayList<>();
        ArrayList<ChunkNode> reserveNodes = new ArrayList<>();
        ArrayList<ChunkNode> activeNodes = new ArrayList<>();
        for (ChunkNode selectedNode : selectedNodes) {
            if (selectedNode.active==false && selectedNode.empty==false) {
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
                return activeNodes; // no nodes to update so no work to do
            }
        }

        ArrayList<ChunkNode> emptyNodes = new ArrayList<>();
        ArrayList<ChunkNode> constructedNodes = new ArrayList<>();
        for (ChunkNode filteredNode : filteredNodes) {
            boolean result = //filterNodesForDebug(filteredNode) &&
                    ConstructChunkNodeData(filteredNode);
            if (filteredNode.renderMesh !=null || (filteredNode.seamNodes!=null && filteredNode.seamNodes.size()> 0)) {
                constructedNodes.add(filteredNode);
                activeNodes.add(filteredNode);
                Vec3f colour = filteredNode.size == CLIPMAP_LEAF_SIZE ? Constants.Blue : Constants.Green;
                renderCmds.addCube(colour, 0.2f, filteredNode.min, filteredNode.size);
            } else {
                filteredNode.empty = true; // no meshes in chunk - empty chunk
                emptyNodes.add(filteredNode);
            }
        }
        for (ChunkNode node : emptyNodes) {
            propagateEmptyStateDownward(node);
        }

        // construct seams begin
        Set<ChunkNode> seamUpdateNodes = new HashSet<>();
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
            if (seamUpdateNode.seamMesh!=null){
                invalidatedMeshes.add(seamUpdateNode.seamMesh);
                seamUpdateNode.seamMesh.cleanMesh();
                seamUpdateNode.seamMesh = null;
            }
            generateClipmapSeamMesh(seamUpdateNode, root);
        }
        // construct seams end
        return activeNodes;
    }

    private void generateClipmapSeamMesh(ChunkNode node, ChunkNode root){
        Set<OctreeNode> seamNodes = new HashSet<>(2048);
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
        if(seamNodes.isEmpty())
            return;
        OctreeNode seamOctreeRoot = voxelOctree.constructTreeUpwards(new ArrayList<>(seamNodes), node.min, node.size * 2);
        MeshBuffer meshBuffer = voxelOctree.GenerateMeshFromOctree(seamOctreeRoot, true);
        Renderer renderer = new Renderer(new MeshDcVBO(meshBuffer));
        meshBuffer.getVertices().clear();
        meshBuffer.getIndicates().clear();
        renderer.setRenderInfo(new RenderInfo(new CW(), DcSimpleShader.getInstance()));
        node.seamMesh = renderer;
    }

    private List<OctreeNode> selectSeamNodes(ChunkNode node, ChunkNode neighbour, int neighbourIndex){
        Vec3i chunkMax = node.min.add(node.size);
        Aabb aabb = new Aabb(node.min, node.size * 2);
        int neighbourScaleSize = neighbour.size / (VOXELS_PER_CHUNK * LEAF_SIZE_SCALE);
        List<OctreeNode> selectedSeamNodes = new ArrayList<>();
        for (OctreeNode octreeSeamNode : neighbour.seamNodes) {
            Vec3i max = octreeSeamNode.min.add(neighbourScaleSize * LEAF_SIZE_SCALE);
            if (octreeSeamNode.size!=neighbourScaleSize * LEAF_SIZE_SCALE){
                int t=4;
            }
            if (!filterSeamNode(neighbourIndex, chunkMax, octreeSeamNode.min, max) || !aabb.pointIsInside(octreeSeamNode.min)) {
                continue;
            }
            octreeSeamNode.drawInfo.color = Constants.Yellow;
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
            else if (node.size > CLIPMAP_LEAF_SIZE) {
                for (int i = 0; i < 8; i++) {
                    findActiveNodes(node.children[i], referenceNode, neighbourActiveNodes);
                }
            }
        }
    }

    static Vec3i chunkMinForPosition(OctreeNode p) {
        //int mask = ~(p.chunkSize-1);
	    int mask = ~(CLIPMAP_LEAF_SIZE-1);
        return new Vec3i(p.min.x & mask, p.min.y & mask, p.min.z & mask);
    }

    private boolean filterNodesForDebug(ChunkNode filteredNode){
        boolean res =
                filteredNode.min.equals(new Vec3i(512,0,-512)) ||
                        filteredNode.min.equals(new Vec3i(1024,0,-1024));
        return res;
    }

    private boolean ConstructChunkNodeData(ChunkNode chunk) {
        if (chunk.renderMesh !=null || (chunk.seamNodes!=null && chunk.seamNodes.size()> 0)){
            chunk.active = true;
            return chunk.active;
        }
        EnumMap<VoxelTypes, List<OctreeNode>> res = voxelOctree.createLeafVoxelNodes(chunk.size, chunk.min,
                VOXELS_PER_CHUNK, CLIPMAP_LEAF_SIZE, LEAF_SIZE_SCALE);
        chunk.seamNodes = res.get(VoxelTypes.SEAMS);
        chunk.numSeamNodes = chunk.seamNodes.size();
        if (res.get(VoxelTypes.NODES).isEmpty() || res.get(VoxelTypes.SEAMS).isEmpty()) {
            return false;
        }
        OctreeNode octreeRoot = voxelOctree.constructTreeUpwards(res.get(VoxelTypes.NODES), chunk.min, chunk.size);
        chunk.active = true;

        MeshBuffer meshBuffer = voxelOctree.GenerateMeshFromOctree(octreeRoot,false);

        Renderer renderer = new Renderer(new MeshDcVBO(meshBuffer));
        meshBuffer.getVertices().clear();
        meshBuffer.getIndicates().clear();
        renderer.setRenderInfo(new RenderInfo(new CW(), DcSimpleShader.getInstance()));
        chunk.renderMesh = renderer;
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
