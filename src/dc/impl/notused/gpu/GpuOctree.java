package dc.impl.notused.gpu;

import dc.impl.notused.gpu.opencl.BufferGpu;

public class GpuOctree {
    public int numNodes = 0;
    private BufferGpu nodeCodesBuf, nodeMaterialsBuf;
    private BufferGpu vertexPositionsBuf, vertexNormalsBuf;

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
