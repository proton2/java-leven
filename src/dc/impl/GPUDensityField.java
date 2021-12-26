package dc.impl;

import core.math.Vec3i;
import core.math.Vec4f;
import dc.impl.opencl.BufferGpu;

import java.util.Map;
import java.util.concurrent.ConcurrentSkipListMap;

public class GPUDensityField {
    private BufferGpu materials;
    private BufferGpu edgeIndices;
    private BufferGpu normals;
    public Vec3i min;
    public int size;
    public int lastCSGOperation = 0;
    private int numEdges;

    public int[] materialsCpu;
    public Map<Integer, Vec4f> hermiteEdgesMap = new ConcurrentSkipListMap<>();

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
