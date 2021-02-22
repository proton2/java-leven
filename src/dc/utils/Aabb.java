package dc.utils;

import core.math.Vec3f;
import core.math.Vec3i;

public class Aabb {
    public static final float FLT_EPSILON = 1.1920928955078125E-7f;
    public Vec3i min;
    public Vec3i max;

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

    public boolean overlaps(Aabb other) {
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

    public boolean pointIsInside(Vec3f point) {
        return (point.X >= min.x && point.X < max.x) &&
                (point.Y >= min.y && point.Y < max.y) &&
                (point.Z >= min.z && point.Z < max.z);
    }

    Vec3i getOrigin() {
		Vec3i dim = max.sub(min);
        return min.add(dim.div(2));
    }

    // from Real-time Collision Detection
    boolean intersect(Vec3f rayOrigin, Vec3f rayDir, Vec3f point) {
        float tmin = 0.f;
        float tmax = Float.MAX_VALUE;
        float[] rayOrigin1d = rayOrigin.to1dArray();
        float[] rayDir1d = rayDir.to1dArray();

		int[] fmin = min.to1dArray();
		int[] fmax = max.to1dArray();

        for (int i = 0; i < 3; i++) {
            if (Math.abs(rayDir1d[i]) < FLT_EPSILON) {
                if (rayOrigin1d[i] < fmin[i] || rayOrigin1d[i] >= fmax[i])
                    return false;
            }
			else {
				float ood = 1.f / rayDir1d[i];
				float t1 = (fmin[i] - rayOrigin1d[i]) * ood;
				float t2 = (fmax[i] - rayOrigin1d[i]) * ood;

                tmin = Math.max(tmin, Math.min(t1, t2));
                tmax = Math.min(tmax, Math.max(t1, t2));

                if (tmin > tmax) {
                    return false;
                }
            }
        }

        float distance = tmin;
        if (point!=null) {
            point.set(rayOrigin.add(rayDir.mul(tmin)));
        }
        return true;
    }
}
