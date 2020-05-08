package dc.entities;

import core.math.Vec4f;

public class DebugDrawBuffer {
    private Vec4f[] vertexBuffer;
    private Vec4f[] colourBuffer;
    private int[] indexBuffer;
    private int	numVertices = 0;
    private int	numIndices = 0;

    public Vec4f[] getVertexBuffer() {
        return vertexBuffer;
    }

    public void setVertexBuffer(Vec4f[] vertexBuffer) {
        this.vertexBuffer = vertexBuffer;
    }

    public Vec4f[] getColourBuffer() {
        return colourBuffer;
    }

    public void setColourBuffer(Vec4f[] colourBuffer) {
        this.colourBuffer = colourBuffer;
    }

    public int[] getIndexBuffer() {
        return indexBuffer;
    }

    public void setIndexBuffer(int[] indexBuffer) {
        this.indexBuffer = indexBuffer;
    }

    public int getNumVertices() {
        return numVertices;
    }

    public void setNumVertices(int numVertices) {
        this.numVertices = numVertices;
    }

    public int getNumIndices() {
        return numIndices;
    }

    public void setNumIndices(int numIndices) {
        this.numIndices = numIndices;
    }
}
