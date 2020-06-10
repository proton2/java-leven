package core.math;

public class Vec4i extends Vec3i {
    public int w;

    public Vec4i(int r) {
        super(r);
        this.w = r;
    }

    public Vec4i(int x, int y, int z, int w) {
        super(x, y, z);
        this.w = w;
    }

    public void setVector3i(Vec3i from){
        this.x = from.x;
        this.y = from.y;
        this.z = from.z;
    }
}
