package dc.impl;

import dc.impl.opencl.CuckooHashOpenCLService;

public class GpuOctree {
    private int     numNodes = 0;
    private long nodeCodesBuf, nodeMaterialsBuf;
    private long vertexPositionsBuf, vertexNormalsBuf;
    private CuckooHashOpenCLService hashTable;

    public int getNumNodes() {
        return numNodes;
    }

    public void setNumNodes(int numNodes) {
        this.numNodes = numNodes;
    }

    public long getNodeCodesBuf() {
        return nodeCodesBuf;
    }

    public void setNodeCodesBuf(long nodeCodesBuf) {
        this.nodeCodesBuf = nodeCodesBuf;
    }

    public long getNodeMaterialsBuf() {
        return nodeMaterialsBuf;
    }

    public void setNodeMaterialsBuf(long nodeMaterialsBuf) {
        this.nodeMaterialsBuf = nodeMaterialsBuf;
    }

    public long getVertexPositionsBuf() {
        return vertexPositionsBuf;
    }

    public void setVertexPositionsBuf(long vertexPositionsBuf) {
        this.vertexPositionsBuf = vertexPositionsBuf;
    }

    public long getVertexNormalsBuf() {
        return vertexNormalsBuf;
    }

    public void setVertexNormalsBuf(long vertexNormalsBuf) {
        this.vertexNormalsBuf = vertexNormalsBuf;
    }

    public CuckooHashOpenCLService getHashTable() {
        return hashTable;
    }

    public void setHashTable(CuckooHashOpenCLService hashTable) {
        this.hashTable = hashTable;
    }
}
