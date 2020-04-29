package dc;

import core.math.Vec3i;

import java.util.Objects;

public class OctreeNode {
    public OctreeNodeType Type;
    public Vec3i min;
    public int size;
    public OctreeNode[] children;
    public OctreeDrawInfo drawInfo;
    public Vec3i rootMin;
    public int chunkSize;

    public OctreeNode() {
        Type = OctreeNodeType.Node_None;
        min = new Vec3i(0, 0, 0);
        size = 0;
        drawInfo = new OctreeDrawInfo();

        children = new OctreeNode[8];
        for (int i = 0; i < 8; i++) {
            children[i] = null;
        }
    }

    public OctreeNode(Vec3i min, int size, Vec3i rootMin, int rootSize) {
        this.rootMin = rootMin;
        this.chunkSize = rootSize;
        this.min = min;
        this.size = size;
        drawInfo = new OctreeDrawInfo();
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
