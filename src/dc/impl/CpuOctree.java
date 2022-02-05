package dc.impl;

import core.math.Vec4f;

import java.util.Map;

public class CpuOctree {
    public int numNodes = 0;
    public int[] nodeCodes;
    public int[] nodeMaterials;
    public Vec4f[] vertexPositions;
    public Vec4f[] vertexNormals;
    public Map<Integer, Integer> octreeNodes;
}
