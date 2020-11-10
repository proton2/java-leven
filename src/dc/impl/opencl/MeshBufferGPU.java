package dc.impl.opencl;

public class MeshBufferGPU {
    private BufferGpu vertices, triangles;
    private int	countVertices, countTriangles;

    public BufferGpu getVertices() {
        return vertices;
    }

    public void setVertices(BufferGpu vertices) {
        this.vertices = vertices;
    }

    public BufferGpu getTriangles() {
        return triangles;
    }

    public void setTriangles(BufferGpu triangles) {
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