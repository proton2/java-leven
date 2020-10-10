package dc.impl;

import core.math.Vec3i;

public class GPUDensityField {
    private long materials;
    private long edgeIndices;
    private long normals;
    private Vec3i min;
    private int size;
    private int numEdges;

    public long getNormals() {
        return normals;
    }

    public void setNormals(long normals) {
        this.normals = normals;
    }

    public int getNumEdges() {
        return numEdges;
    }

    public void setNumEdges(int numEdges) {
        this.numEdges = numEdges;
    }

    public long getMaterials() {
        return materials;
    }

    public void setMaterials(long materials) {
        this.materials = materials;
    }

    public long getEdgeIndices() {
        return edgeIndices;
    }

    public void setEdgeIndices(long edgeIndices) {
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
