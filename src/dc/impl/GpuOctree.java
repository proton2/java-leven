package dc.impl;

import dc.impl.opencl.CuckooHashOpenCLService;

public class GpuOctree {
    private int     numNodes = 0;
    private long    d_nodeCodesBuffer, d_nodeMaterialsBuffer;
    private long    d_vertexPositionsBuffer, d_vertexNormalsBuffer;
    private CuckooHashOpenCLService d_hashTable;

    public int getNumNodes() {
        return numNodes;
    }

    public void setNumNodes(int numNodes) {
        this.numNodes = numNodes;
    }

    public long getD_nodeCodesBuffer() {
        return d_nodeCodesBuffer;
    }

    public void setD_nodeCodesBuffer(long d_nodeCodesBuffer) {
        this.d_nodeCodesBuffer = d_nodeCodesBuffer;
    }

    public long getD_nodeMaterialsBuffer() {
        return d_nodeMaterialsBuffer;
    }

    public void setD_nodeMaterialsBuffer(long d_nodeMaterialsBuffer) {
        this.d_nodeMaterialsBuffer = d_nodeMaterialsBuffer;
    }

    public long getD_vertexPositionsBuffer() {
        return d_vertexPositionsBuffer;
    }

    public void setD_vertexPositionsBuffer(long d_vertexPositionsBuffer) {
        this.d_vertexPositionsBuffer = d_vertexPositionsBuffer;
    }

    public long getD_vertexNormalsBuffer() {
        return d_vertexNormalsBuffer;
    }

    public void setD_vertexNormalsBuffer(long d_vertexNormalsBuffer) {
        this.d_vertexNormalsBuffer = d_vertexNormalsBuffer;
    }

    public CuckooHashOpenCLService getD_hashTable() {
        return d_hashTable;
    }

    public void setD_hashTable(CuckooHashOpenCLService d_hashTable) {
        this.d_hashTable = d_hashTable;
    }
}
