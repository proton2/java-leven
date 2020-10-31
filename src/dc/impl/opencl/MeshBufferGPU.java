package dc.impl.opencl;

public class MeshBufferGPU {
    private long vertices, triangles;
    private int	countVertices, countTriangles;

    public long getVertices() {
        return vertices;
    }

    public void setVertices(long vertices) {
        this.vertices = vertices;
    }

    public long getTriangles() {
        return triangles;
    }

    public void setTriangles(long triangles) {
        this.triangles = triangles;
    }

    public int getCountVertices() {
        return countVertices;
    }

    public void setCountVertices(int countVertices) {
        this.countVertices = countVertices;
    }

    public int getCountTriangles() {
        return countTriangles;
    }

    public void setCountTriangles(int countTriangles) {
        this.countTriangles = countTriangles;
    }
}