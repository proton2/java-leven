package dc.impl;

import dc.utils.VoxelHelperUtils;

public class MeshGenerationContext {
    private final int voxelsPerChunk;
    public final int hermiteIndexSize;
    public final int fieldSize;
    public final int indexShift;
    public final int indexMask;
    private final int LEAF_SIZE_LOG2 = 2;
    public final int leafSizeScale = 1 << LEAF_SIZE_LOG2;
    public final int clipmapLeafSize;
    public final int worldSizeXZ;
    public final int worldSizeY;
    public final int numLods = 6;
    public final int MATERIAL_AIR = 0;
    public final int MATERIAL_SOLID = 1;
    public final int MAX_OCTREE_DEPTH;

    public final long CUCKOO_EMPTY_VALUE = 0;
    public final int CUCKOO_STASH_HASH_INDEX = 4;
    public final int CUCKOO_HASH_FN_COUNT = CUCKOO_STASH_HASH_INDEX + 1;
    public final int CUCKOO_STASH_SIZE = 101;
    public final int CUCKOO_MAX_ITERATIONS = 32;

    public MeshGenerationContext(int voxelsPerChunk) {
        this.voxelsPerChunk = voxelsPerChunk;
        this.indexShift = VoxelHelperUtils.log2(voxelsPerChunk) + 1;
        this.hermiteIndexSize = voxelsPerChunk + 1;
        this.fieldSize = hermiteIndexSize + 1;
        this.indexMask = (1 << indexShift) - 1;
        this.clipmapLeafSize = leafSizeScale * voxelsPerChunk;
        int worldBrickCountXZ = 4;
        int BRICK_SIZE = 8;
        this.worldSizeXZ = worldBrickCountXZ * BRICK_SIZE * clipmapLeafSize;
        this.worldSizeY = 4 * BRICK_SIZE * clipmapLeafSize;
        this.MAX_OCTREE_DEPTH = VoxelHelperUtils.log2(voxelsPerChunk);
    }

    public int getVoxelsPerChunk() {
        return voxelsPerChunk;
    }

    public int getHermiteIndexSize() {
        return hermiteIndexSize;
    }

    public int getFieldSize() {
        return fieldSize;
    }

    public int getIndexShift() {
        return indexShift;
    }

    public int getIndexMask() {
        return indexMask;
    }
}
