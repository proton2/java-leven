package dc.impl;

import core.math.Vec4f;
import dc.impl.opencl.BufferGpu;

import java.util.Map;

public class GpuOctree {
    public int numNodes = 0;
    private BufferGpu nodeCodesBuf, nodeMaterialsBuf;
    private BufferGpu vertexPositionsBuf, vertexNormalsBuf;

    public int[] d_nodeCodesCpu;
    public int[] d_nodeMaterialsCpu;
    public Vec4f[] d_vertexPositionsCpu;
    public Vec4f[] d_vertexNormalsCpu;
    public Map<Integer, Integer> octreeNodes;

    public int getNumNodes() {
        return numNodes;
    }

    public void setNumNodes(int numNodes) {
        this.numNodes = numNodes;
    }

    public BufferGpu getNodeCodesBuf() {
        return nodeCodesBuf;
    }

    public void setNodeCodesBuf(BufferGpu nodeCodesBuf) {
        this.nodeCodesBuf = nodeCodesBuf;
    }

    public BufferGpu getNodeMaterialsBuf() {
        return nodeMaterialsBuf;
    }

    public void setNodeMaterialsBuf(BufferGpu nodeMaterialsBuf) {
        this.nodeMaterialsBuf = nodeMaterialsBuf;
    }

    public BufferGpu getVertexPositionsBuf() {
        return vertexPositionsBuf;
    }

    public void setVertexPositionsBuf(BufferGpu vertexPositionsBuf) {
        this.vertexPositionsBuf = vertexPositionsBuf;
    }

    public BufferGpu getVertexNormalsBuf() {
        return vertexNormalsBuf;
    }

    public void setVertexNormalsBuf(BufferGpu vertexNormalsBuf) {
        this.vertexNormalsBuf = vertexNormalsBuf;
    }
}
