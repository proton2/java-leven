package dc.impl.notused.gpu;

import core.math.Vec3i;
import dc.OctreeNode;
import dc.OctreeNodeType;

public class MdcOctreeNode extends OctreeNode {
    public MdcVertex[] vertices;

    public MdcOctreeNode(Vec3i position, int size, OctreeNodeType type) {
        super(position, size, type);
        this.vertices = new MdcVertex[0];
    }
}
