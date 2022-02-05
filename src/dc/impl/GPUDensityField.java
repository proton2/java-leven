package dc.impl;

import core.math.Vec3i;
import dc.impl.opencl.BufferGpu;

public class GPUDensityField {
    private BufferGpu materials;
    private BufferGpu edgeIndices;
    private BufferGpu normals;
    public Vec3i min;
    public int size;
    private int numEdges;

    public BufferGpu getNormals() {
        return normals;
    }

    public void setNormals(BufferGpu normals) {
        this.normals = normals;
    }

    public int getNumEdges() {
        return numEdges;
    }

    public void setNumEdges(int numEdges) {
        this.numEdges = numEdges;
    }

    public BufferGpu getMaterials() {
        return materials;
    }

    public void setMaterials(BufferGpu materials) {
        this.materials = materials;
    }

    public BufferGpu getEdgeIndices() {
        return edgeIndices;
    }

    public void setEdgeIndices(BufferGpu edgeIndices) {
        this.edgeIndices = edgeIndices;
    }

    public Vec3i getMin() {
        return min;
    }

    public void setMin(Vec3i min) {
        this.min = min;
    }

    public int getSize() {
        return size;
    }

    public void setSize(int size) {
        this.size = size;
    }
}
