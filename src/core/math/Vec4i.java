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

    public Vec4i() {}

    public void setVector3i(Vec3i from){
        this.x = from.x;
        this.y = from.y;
        this.z = from.z;
    }

    public Vec4i add(Vec4i r) {
        return new Vec4i(this.x + r.x, this.y + r.y, this.z + r.z, this.w + r.w);
    }
}
