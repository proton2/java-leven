package dc.utils;

import core.math.Vec3i;

public class Aabb {
    public static final float FLT_EPSILON = 1.1920928955078125E-7f;
    private Vec3i min;
    private Vec3i max;

    public Aabb(){
        min = new Vec3i(0);
        max = new Vec3i(0);
    }

    public Aabb(Vec3i min, int size){
        this.min = min;
        this.max = min.add(size);
    }

    public Aabb(Vec3i min, Vec3i max){
        this.min = min;
        this.max = max;
    }

    boolean overlaps(Aabb other) {
        return !
                (max.x < other.min.x ||
                        max.y < other.min.y ||
                        max.z < other.min.z ||
                        min.x > other.max.x ||
                        min.y > other.max.y ||
                        min.z > other.max.z);
    }

    public boolean pointIsInside(Vec3i point) {
        return (point.x >= min.x && point.x < max.x) &&
                (point.y >= min.y && point.y < max.y) &&
                (point.z >= min.z && point.z < max.z);
    }

    Vec3i getOrigin() {
		Vec3i dim = max.sub(min);
        return min.add(dim.div(2));
    }
}
