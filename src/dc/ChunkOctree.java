package dc;

import core.kernel.Camera;
import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.model.Vertex;
import utils.Frustum;

import java.util.*;

public class ChunkOctree {
    static int LEAF_SIZE_LOG2 = 2;
    static int LEAF_SIZE_SCALE = 1 << LEAF_SIZE_LOG2;
    static int VOXELS_PER_CHUNK = 64;
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

    public Vec3f RenderColour_Blue = new Vec3f(0.f, 0.f, 1.f);
    public Vec3f RenderColour_Green = new Vec3f(0.f, 1.f, 0.f);

    float[] LOD_ACTIVE_DISTANCES = {0.f,
            CLIPMAP_LEAF_SIZE * 1.5f,
            CLIPMAP_LEAF_SIZE * 3.5f,
            CLIPMAP_LEAF_SIZE * 5.5f,
            CLIPMAP_LEAF_SIZE * 7.5f,
            CLIPMAP_LEAF_SIZE * 13.5f
    };

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
            child.min = node.min.add(Octree.CHILD_MIN_OFFSETS[i].mul(child.size));
            node.children[i] = child;
        }
        for (int i = 0; i < 8; i++) {
            constructChildrens(node.children[i]);
        }
    }

    public ArrayList<ChunkNode> update(ChunkNode root, Camera camera, RenderDebugCmdBuffer renderCmds) {
        ArrayList<ChunkNode> selectedNodes = new ArrayList<>();
        selectActiveChunkNodes(root, false, camera.getPosition(), selectedNodes);
//        ArrayList<ChunkNode> invalidatedMeshes = new ArrayList<>();
//        ReleaseInvalidatedNodes(root, invalidatedMeshes);
        ArrayList<ChunkNode> filteredNodes = new ArrayList<>();
        ArrayList<ChunkNode> reserveNodes = new ArrayList<>();
        ArrayList<ChunkNode> activeNodes = new ArrayList<>();
        for (ChunkNode selectedNode : selectedNodes) {
            if (!selectedNode.active && !selectedNode.empty) {
                if (Frustum.cubeInFrustum(camera.getFrustumPlanes(),
                        selectedNode.min.x, selectedNode.min.y, selectedNode.min.z, selectedNode.size)) {
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
                return null; // no nodes to update so no work to do
            }
        }

        ArrayList<ChunkNode> emptyNodes = new ArrayList<>();
        ArrayList<ChunkNode> constructedNodes = new ArrayList<>();
        for (ChunkNode filteredNode : filteredNodes) {
            boolean result = (filterNodesForDebug(filteredNode) && ConstructChunkNodeData(filteredNode, camera.getFrustumPlanes()));
            if (!result){
                continue;
            }
            if (filteredNode.vertArray!=null || filteredNode.seamNodes.size()> 0) {
                constructedNodes.add(filteredNode);
                activeNodes.add(filteredNode);
                Vec3f colour = filteredNode.size == CLIPMAP_LEAF_SIZE ? RenderColour_Blue : RenderColour_Green;
                renderCmds.addCube(colour, 0.2f, filteredNode.min, filteredNode.size);
            } else {
                filteredNode.empty = true;
                emptyNodes.add(filteredNode);
            }
        }

        for (ChunkNode node : emptyNodes) {
            propagateEmptyStateDownward(node);
        }

        constructSeams(root, constructedNodes);

        return constructedNodes;
    }

    boolean filterNodesForDebug(ChunkNode filteredNode){
        //(filteredNode.min.x==0 && filteredNode.min.y==0 && filteredNode.min.z==0) &&
        //(filteredNode.min.x==0 && filteredNode.min.y==-256 && filteredNode.min.z==-256) &&
        //(filteredNode.min.x==256 && filteredNode.min.y==0 && filteredNode.min.z==-256) &&
                    /*
                    [1024,-1024,-2048]
                    [1024,-1024,-1024]
                    [1024,-1024,1024]

                    [2048,-2048,-4096]
                    [2048,-2048,-2048]
                    [2048,-2048,0]
                    [2048,-2048,2048]
                     */
        //(filteredNode.min.x==2048 && filteredNode.min.y==-2048 && filteredNode.min.z==0) &&
        return true;
//                        (filteredNode.min.x==256 && filteredNode.min.y==0 && filteredNode.min.z==0) ||
//                        (filteredNode.min.x==256 && filteredNode.min.y==0 && filteredNode.min.z==-256);
    }

    private void constructSeams(ChunkNode root, List<ChunkNode> constructedNodes) {
        Set<ChunkNode> seamUpdateNodes = new HashSet<>();
        for (ChunkNode constructedNode : constructedNodes) {
            for (int i = 0; i < 8; i++) {
                Vec3i neighbourMin = constructedNode.min.sub(Octree.CHILD_MIN_OFFSETS[i].mul(constructedNode.size));
                ChunkNode candidateNeighbour = findNode(root, constructedNode.size, neighbourMin);
                if (candidateNeighbour!=null){
                    List<ChunkNode> neighbourActiveNodes = new ArrayList<>();
                    findActiveNodes(root, candidateNeighbour, neighbourActiveNodes);
                    seamUpdateNodes.addAll(neighbourActiveNodes);
                }
            }
        }
        for (ChunkNode seamUpdateNode : seamUpdateNodes) {
            if (seamUpdateNode.seamVertArray!=null && seamUpdateNode.seamVertArray.length>0){
                seamUpdateNode.seamVertArray = null;
                seamUpdateNode.seamIndices = null;
            }
            generateClipmapSeamMesh(seamUpdateNode, root);
        }
    }

    void generateClipmapSeamMesh(ChunkNode node, ChunkNode root){
        Set<OctreeNode> seamNodes = new HashSet<>(2048);
        for (int i = 0; i < 8; i++) {
            Vec3i neighbourMin = node.min.add(Octree.CHILD_MIN_OFFSETS[i].mul(node.size));
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
        node.seamOctreeRoot = constructTreeUpwards(new ArrayList<>(seamNodes), node.min, node.size * 2);

        List<MeshVertex> seamVertices = new ArrayList<>();
        List<Integer> seamIndcies = new ArrayList<>();
        Octree.GenerateMeshFromOctree(node.seamOctreeRoot, seamVertices, seamIndcies);
        node.seamVertArray = seamVertices.toArray(new Vertex[0]);
        node.seamIndices = seamIndcies.stream().mapToInt(x -> x).toArray();
    }

    List<OctreeNode> selectSeamNodes(ChunkNode node, ChunkNode neighbour, int neighbourIndex){
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
            octreeSeamNode.drawInfo.color = new Vec3f(0.f, 0.7f, 0.f);
            selectedSeamNodes.add(octreeSeamNode);
        }
        return selectedSeamNodes;
    }

    private int getOctreeSizeByChunkSize(int chunkSize){
        int chunkScaleSize = chunkSize / (VOXELS_PER_CHUNK * LEAF_SIZE_SCALE);
        return chunkScaleSize * LEAF_SIZE_SCALE;
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

    ChunkNode findNode(ChunkNode node, int size, Vec3i min) {
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

    void findActiveNodes(ChunkNode node, ChunkNode referenceNode, List<ChunkNode> neighbourActiveNodes){
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
	    int mask = ~(p.chunkSize);
	    //int mask = ~(CLIPMAP_LEAF_SIZE-1);
        return new Vec3i(p.min.x & mask, p.min.y & mask, p.min.z & mask);
    }

    boolean ConstructChunkNodeData(ChunkNode chunk, Vec4f[] frustumPlanes) {
        chunk.active = createLeafVoxelNodes(chunk, frustumPlanes);
        if (!chunk.active){
            return false;
        }
        List<MeshVertex> vertices = new ArrayList<>();
        List<Integer> indcies = new ArrayList<>();
        Octree.GenerateMeshFromOctree(chunk.octreeRoot, vertices, indcies);
        chunk.vertArray = vertices.toArray(new Vertex[0]);
        chunk.indices = indcies.stream().mapToInt(x -> x).toArray();
        return chunk.active;
    }

    public boolean createLeafVoxelNodes(ChunkNode chunk, Vec4f[] frustumPlanes) {
        List<OctreeNode> voxels = new ArrayList<>();
        List<OctreeNode> seamNodes = new ArrayList<>();
        for (int zi = 0; zi < VOXELS_PER_CHUNK; zi++) {
            for (int yi = 0; yi < VOXELS_PER_CHUNK; yi++) {
                for (int xi = 0; xi < VOXELS_PER_CHUNK; xi++) {
                    int leafSize = (chunk.size / CLIPMAP_LEAF_SIZE) * LEAF_SIZE_SCALE;
                    Vec3i leafMin = new Vec3i(xi, yi, zi).mul(leafSize).add(chunk.min);
                    if (Frustum.cubeInFrustum(frustumPlanes, leafMin.x, leafMin.y, leafMin.z, leafSize)) {
                        OctreeNode leaf = Octree.ConstructLeaf(new OctreeNode(leafMin, leafSize, chunk.min, chunk.size));
                        if (leaf != null) {
                            if(xi==0||xi==VOXELS_PER_CHUNK-1 || yi==0||yi==VOXELS_PER_CHUNK-1 ||zi==0||zi==VOXELS_PER_CHUNK-1){
                                seamNodes.add(leaf);
                            }
                            voxels.add(leaf);
                        }
                    }
                }
            }
        }
        if (voxels.isEmpty() || seamNodes.isEmpty()) {
            return false;
        }
        chunk.octreeRoot = constructTreeUpwards(voxels, chunk.min, chunk.size);
        chunk.seamNodes = seamNodes;
        chunk.numSeamNodes = seamNodes.size();
        return true;
    }

    private OctreeNode constructTreeUpwards(List<OctreeNode> inputNodes, Vec3i rootMin, int rootNodeSize) {
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
        int octreeCounts = VoxelHelperUtils.countLeafNodes(sortedNodes.get(0));
        if (octreeCounts!=inputNodes.size()){
            throw new IllegalStateException("Octree leafs is not equal to octree counts!");
        }
        return sortedNodes.get(0);
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
                Vec3i childMin = parentPos.add(Octree.CHILD_MIN_OFFSETS[j].mul(parentSize / 2));
                if (childMin.equals(node.min)) {
                    parent.children[j] = node;
                    break;
                }
            }
        }
        return new ArrayList<>(parentsHash.values());
    }

    void propagateEmptyStateDownward(ChunkNode node) {
        if (node==null) {
            return;
        }
        node.empty = true;
        for (int i = 0; i < 8; i++) {
            propagateEmptyStateDownward(node.children[i]);
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
}
