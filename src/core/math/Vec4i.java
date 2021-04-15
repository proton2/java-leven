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

    public Vec4i(Vec3i min, int size) {
        this.x = min.x;
        this.y = min.y;
        this.z = min.z;
        this.w = size;
    }

    public Vec4i add(Vec4i r) {
        return new Vec4i(this.x + r.x, this.y + r.y, this.z + r.z, this.w + r.w);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        if (!super.equals(o)) return false;

        Vec4i vec4i = (Vec4i) o;

        return w == vec4i.w;
    }

    @Override
    public int hashCode() {
        int result = super.hashCode();
        result = 31 * result + w;
        return result;
    }
}
