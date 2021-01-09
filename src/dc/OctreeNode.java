package dc;

import core.math.Vec3i;

public abstract class OctreeNode {
    public OctreeNodeType Type;
    public Vec3i min;
    public int size;
    public int index;
    public int corners;
    public OctreeNode[] children;
    private ChunkNode chunk;
    public int child_index;

    public ChunkNode getChunk() {
        return chunk;
    }

    public void setChunk(ChunkNode chunk) {
        this.chunk = chunk;
    }

    public OctreeNode(Vec3i min, int size, OctreeNodeType type) {
        this.min = min;
        this.size = size;
        this.Type = type;
        this.children = new OctreeNode[8];
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        OctreeNode that = (OctreeNode) o;

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
