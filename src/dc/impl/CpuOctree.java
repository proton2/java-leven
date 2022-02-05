package dc.impl;

import core.math.Vec4f;

import java.util.Map;

public class CpuOctree {
    public int numNodes = 0;
    public int[] d_nodeCodesCpu;
    public int[] d_nodeMaterialsCpu;
    public Vec4f[] d_vertexPositionsCpu;
    public Vec4f[] d_vertexNormalsCpu;
    public Map<Integer, Integer> octreeNodes;
}
