package dc.entities;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;

public class MeshBuffer {
    FloatBuffer vertices;
    IntBuffer indicates;
    int numVertices;
    int numIndicates;

    public MeshBuffer(){}

    public MeshBuffer(FloatBuffer vertices, IntBuffer indicates, int numVertices, int numIndicates) {
        this.vertices = vertices;
        this.indicates = indicates;
        this.numVertices = numVertices;
        this.numIndicates = numIndicates;
    }

    public FloatBuffer getVertices() {
        return vertices;
    }

    public void setVertices(FloatBuffer vertices) {
        this.vertices = vertices;
    }

    public IntBuffer getIndicates() {
        return indicates;
    }

    public void setIndicates(IntBuffer indicates) {
        this.indicates = indicates;
    }

    public int getNumVertices() {
        return numVertices;
    }

    public void setNumVertices(int numVertices) {
        this.numVertices = numVertices;
    }

    public int getNumIndicates() {
        return numIndicates;
    }

    public void setNumIndicates(int numIndicates) {
        this.numIndicates = numIndicates;
    }
}
