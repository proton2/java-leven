package dc.impl;

public class MeshGenerationContext {
    private final int voxelsPerChunk;
    private int hermiteIndexSize;
    private int fieldSize;
    private int indexShift;
    private int indexMask;
    private final int LEAF_SIZE_LOG2 = 2;
    public final int leafSizeScale = 1 << LEAF_SIZE_LOG2;
    public final int clipmapLeafSize;
    public final int worldSizeXZ;
    public final int worldSizeY;
    public final int numLods = 6;

    public MeshGenerationContext(int voxelsPerChunk) {
        this.voxelsPerChunk = voxelsPerChunk;
        this.clipmapLeafSize = leafSizeScale * voxelsPerChunk;
        int worldBrickCountXZ = 4;
        int BRICK_SIZE = 8;
        this.worldSizeXZ = worldBrickCountXZ * BRICK_SIZE * clipmapLeafSize;
        this.worldSizeY = 4 * BRICK_SIZE * clipmapLeafSize;
    }

    public int getVoxelsPerChunk() {
        return voxelsPerChunk;
    }

    public int getHermiteIndexSize() {
        return hermiteIndexSize;
    }

    public void setHermiteIndexSize(int hermiteIndexSize) {
        this.hermiteIndexSize = hermiteIndexSize;
    }

    public int getFieldSize() {
        return fieldSize;
    }

    public void setFieldSize(int fieldSize) {
        this.fieldSize = fieldSize;
    }

    public int getIndexShift() {
        return indexShift;
    }

    public void setIndexShift(int indexShift) {
        this.indexShift = indexShift;
    }

    public int getIndexMask() {
        return indexMask;
    }

    public void setIndexMask(int indexMask) {
        this.indexMask = indexMask;
    }
}
