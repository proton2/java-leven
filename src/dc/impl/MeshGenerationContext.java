package dc.impl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4i;
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

    public Vec3i offset(int i, int size){
        return new Vec3i(
                (i & (1 << (0))) > 0 ? size : 0,
                (i & (1 << (1))) > 0 ? size : 0,
                (i & (1 << (2))) > 0 ? size : 0
        );
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

    public int getEdgeCodeByPos(int x, int y, int z, int axisNum){
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

    public Vec3i decodeVoxelIndex(int index) {
        Vec3i p = new Vec4i(0);
        p.x = (index >> (getIndexShift() * 0)) & getIndexMask();
        p.y = (index >> (getIndexShift() * 1)) & getIndexMask();
        p.z = (index >> (getIndexShift() * 2)) & getIndexMask();
        return p;
    }

    public int encodeVoxelIndex(Vec3i pos) {
        int encoded = 0;
        encoded |= pos.x << (getIndexShift() * 0);
        encoded |= pos.y << (getIndexShift() * 1);
        encoded |= pos.z << (getIndexShift() * 2);
        return encoded;
    }

    public Vec3i positionForCode(int code) {
        int nodeDepth = getMsb(code)/3;
        Vec3i pos = new Vec3i();
        for (int i = MAX_OCTREE_DEPTH - nodeDepth; i < MAX_OCTREE_DEPTH; i++) {
            int c = code & 7;
            code >>= 3;

            int x = (c >> 2) & 1;
            int y = (c >> 1) & 1;
            int z = (c >> 0) & 1;

            pos.x |= (x << i);
            pos.y |= (y << i);
            pos.z |= (z << i);
        }
        return pos;
    }

    public int codeForPosition(Vec3i p) {
        return codeForPosition(p, MAX_OCTREE_DEPTH);
    }

    private int codeForPosition(Vec3i p, int nodeDepth) {
        int code = 1;
        for (int depth = MAX_OCTREE_DEPTH - 1; depth >= (MAX_OCTREE_DEPTH - nodeDepth); depth--) {
            int x = (p.x >> depth) & 1;
            int y = (p.y >> depth) & 1;
            int z = (p.z >> depth) & 1;
            int c = (x << 2) | (y << 1) | z;
            code = (code << 3) | c;
        }
        return code;
    }

    private int getMsb(int value) {
        for (int i = 0, maxBits = 31, test = ~(~0 >>> 1); 0 != test; ++i, test >>>= 1) {
            if (test == (value & test)) {
                return (maxBits - i);
            }
        }
        return -1;
    }
}
