package dc.impl;

import core.math.Vec3f;
import core.math.Vec3i;
import dc.utils.Aabb;
import dc.utils.VoxelHelperUtils;

public class MeshGenerationContext {
    private final int voxelsPerChunk;
    public final int hermiteIndexSize;
    public final int fieldSize;
    public final int indexShift;
    public final int indexMask;
    public final int leafSizeScale = 1;
    public final int clipmapLeafSize;
    public final int worldSizeXZ;
    public final int worldSizeY;
    public final int MATERIAL_AIR = 0;
    public final int MATERIAL_SOLID = 1;
    public final int MAX_OCTREE_DEPTH;
    public final int LOD_MAX_NODE_SIZE;

    public final long CUCKOO_EMPTY_VALUE = 0;
    public final int CUCKOO_STASH_HASH_INDEX = 4;
    public final int CUCKOO_HASH_FN_COUNT = CUCKOO_STASH_HASH_INDEX + 1;
    public final int CUCKOO_STASH_SIZE = 101;
    public final int CUCKOO_MAX_ITERATIONS = 32;
    public final int MAX_MESH_VERTICES = 1 << 23;			// i.e. 8 million
    public final int MAX_MESH_TRIANGLES = MAX_MESH_VERTICES * 2;

    public final Vec3f CSG_OFFSET = new Vec3f(0.0f);
    public final Vec3i CSG_BOUNDS_FUDGE = new Vec3i(2);

    // TODO there is an implicit leaf size scaling here that should be handled explicitly
// think that requires removal of LEAF_SIZE_SCALE from the compute_ files (i.e.
// the compute module should have no knowledge of the sizing, which can be handled
// separately by the calling code)
    public final int COLLISION_VOXELS_PER_CHUNK = 128 / 2;
    public final int COLLISION_NODE_SIZE;

    public Vec3i worldSize;
    public Vec3i worldOrigin = new Vec3i(0);
    public Aabb worldBounds;

    public MeshGenerationContext(int voxelsPerChunk) {
        this.voxelsPerChunk = voxelsPerChunk;
        this.indexShift = VoxelHelperUtils.log2(voxelsPerChunk) + 1;
        this.hermiteIndexSize = voxelsPerChunk + 1;
        this.fieldSize = hermiteIndexSize + 1;
        this.indexMask = (1 << indexShift) - 1;
        this.clipmapLeafSize = leafSizeScale * voxelsPerChunk;
        this.COLLISION_NODE_SIZE = clipmapLeafSize * (4 / 2);
        this.MAX_OCTREE_DEPTH = VoxelHelperUtils.log2(voxelsPerChunk)+2;
        LOD_MAX_NODE_SIZE = clipmapLeafSize * (1 << (MAX_OCTREE_DEPTH - 1));
        int worldBrickCountXZ = 8;
        int BRICK_SIZE = 16;
        this.worldSizeXZ = worldBrickCountXZ * BRICK_SIZE * clipmapLeafSize;
        this.worldSizeY = worldBrickCountXZ * BRICK_SIZE * clipmapLeafSize;
        this.worldSize = new Vec3i(worldSizeXZ, worldSizeY, worldSizeXZ);
        this.worldBounds = new Aabb(worldOrigin.sub(worldSize.div(2)), worldOrigin.add(worldSize.div(2)));
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

    public int getArrayPosByEdgeCode(int edgeCode){
        int voxelIndex = edgeCode >> 2;
        int x = (voxelIndex >> (indexShift * 0)) & indexMask;
        int y = (voxelIndex >> (indexShift * 1)) & indexMask;
        int z = (voxelIndex >> (indexShift * 2)) & indexMask;
        int index = (x + (y * hermiteIndexSize) + (z * hermiteIndexSize * hermiteIndexSize));
        int edgeIndex = index * 3;
        int edgeNumber = edgeCode & 3;
        return edgeIndex + edgeNumber;
    }

    public int getArrayPosByPosition(Vec3i pos, int axisNum){
        int voxelIndex = pos.x | (pos.y << indexShift) | (pos.z << (indexShift * 2));
        int x = (voxelIndex >> (indexShift * 0)) & indexMask;
        int y = (voxelIndex >> (indexShift * 1)) & indexMask;
        int z = (voxelIndex >> (indexShift * 2)) & indexMask;
        int index = (x + (y * hermiteIndexSize) + (z * hermiteIndexSize * hermiteIndexSize));
        int edgeIndex = index * 3;
        return edgeIndex + axisNum;
    }

    public int getEdgeCodeByPosition(int x, int y, int z, int axisNum){
        int voxelIndex = x | (y << indexShift) | (z << (indexShift * 2));
        return (voxelIndex << 2) | axisNum;
    }

    public int getMaterialIndex(Vec3i pos) {
        return pos.x + (pos.y * getFieldSize()) + (pos.z * getFieldSize() * getFieldSize());
    }

    public int getMaterialIndex(int x, int y, int z){
        return x + (y * getFieldSize()) + (z * getFieldSize() * getFieldSize());
    }

    public int getHermiteIndex(int x, int y, int z){
        return (x + (y * getHermiteIndexSize()) + (z * getHermiteIndexSize() * getHermiteIndexSize())) * 3;
    }
}
