package dc.impl.simplifier;

public class MeshTriangle {
    int[]			indices = new int[3];

    public MeshTriangle() {
        indices[0] = indices[1] = indices[2] = 0;
    }

    public MeshTriangle(int i0, int i1, int i2) {
        indices[0] = i0;
        indices[1] = i1;
        indices[2] = i2;
    }
}