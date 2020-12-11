package dc;

import core.math.Vec3i;

public class PointerBasedOctreeNode {
    public OctreeNodeType Type;
    public Vec3i min;
    public int size;
    public PointerBasedOctreeNode[] children;
    public OctreeDrawInfo drawInfo;
    private ChunkNode chunk;

    public ChunkNode getChunk() {
        return chunk;
    }

    public void setChunk(ChunkNode chunk) {
        this.chunk = chunk;
    }

    public PointerBasedOctreeNode() {
        Type = OctreeNodeType.Node_None;
        min = new Vec3i(0, 0, 0);
        size = 0;
        drawInfo = new OctreeDrawInfo();
        children = new PointerBasedOctreeNode[8];
    }

    public PointerBasedOctreeNode(Vec3i min, int size) {
        this.min = min;
        this.size = size;
        drawInfo = new OctreeDrawInfo();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        PointerBasedOctreeNode that = (PointerBasedOctreeNode) o;

        if (size != that.size) return false;
        if (Type != that.Type) return false;
        return min.equals(that.min);
    }

    @Override
    public int hashCode() {
        int result = Type.hashCode();
        result = 31 * result + min.hashCode();
        result = 31 * result + size;
        return result;
    }
}
