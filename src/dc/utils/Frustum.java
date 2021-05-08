package dc.utils;

import core.math.Matrix4f;
import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.utils.Util;

public class Frustum {
    private Vec4f[] frustumPlanes = new Vec4f[6];
    private Vec3f[] frustumCorners = new Vec3f[8];
    private static Frustum frustum = new Frustum();

    public static Frustum getFrustum() {
        if(frustum == null) {
            frustum = new Frustum();
        }
        return frustum;
    }

    public boolean pointInFrustum(float x, float y, float z) {
        for (int i = 0; i < 6; i++) {
            if (frustumPlanes[i].x * x + frustumPlanes[i].y * y + frustumPlanes[i].z * z + frustumPlanes[i].w <= 0.0F) {
                return false;
            }
        }
        return true;
    }

    public boolean sphereInFrustum(float x, float y, float z, float radius) {
        for (int i = 0; i < 6; i++) {
            if (frustumPlanes[i].x * x + frustumPlanes[i].y * y + frustumPlanes[i].z * z + frustumPlanes[i].w <= -radius) {
                return false;
            }
        }
        return true;
    }

    public boolean cubeFullyInFrustum(float x1, float y1, float z1, float x2, float y2, float z2) {
        for (int i = 0; i < 6; i++) {
            if (frustumPlanes[i].x * x1 + frustumPlanes[i].y * y1 + frustumPlanes[i].z * z1 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x2 + frustumPlanes[i].y * y1 + frustumPlanes[i].z * z1 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x1 + frustumPlanes[i].y * y2 + frustumPlanes[i].z * z1 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x2 + frustumPlanes[i].y * y2 + frustumPlanes[i].z * z1 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x1 + frustumPlanes[i].y * y1 + frustumPlanes[i].z * z2 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x2 + frustumPlanes[i].y * y1 + frustumPlanes[i].z * z2 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x1 + frustumPlanes[i].y * y2 + frustumPlanes[i].z * z2 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x2 + frustumPlanes[i].y * y2 + frustumPlanes[i].z * z2 + frustumPlanes[i].w <= 0.0F)
                return false;
        }
        return true;
    }

    public boolean cubeInFrustum(int x, int y, int z, int size) {
        for(int i = 0; i < 6; i++ ) {
            if(frustumPlanes[i].x * (x-size) + frustumPlanes[i].y * (y-size) + frustumPlanes[i].z * (z-size) + frustumPlanes[i].w > 0)
                continue;
            if(frustumPlanes[i].x * (x+size) + frustumPlanes[i].y * (y-size) + frustumPlanes[i].z * (z-size) + frustumPlanes[i].w > 0)
                continue;
            if(frustumPlanes[i].x * (x-size) + frustumPlanes[i].y * (y+size) + frustumPlanes[i].z * (z-size) + frustumPlanes[i].w > 0)
                continue;
            if(frustumPlanes[i].x * (x+size) + frustumPlanes[i].y * (y+size) + frustumPlanes[i].z * (z-size) + frustumPlanes[i].w > 0)
                continue;
            if(frustumPlanes[i].x * (x-size) + frustumPlanes[i].y * (y-size) + frustumPlanes[i].z * (z+size) + frustumPlanes[i].w > 0)
                continue;
            if(frustumPlanes[i].x * (x+size) + frustumPlanes[i].y * (y-size) + frustumPlanes[i].z * (z+size) + frustumPlanes[i].w > 0)
                continue;
            if(frustumPlanes[i].x * (x-size) + frustumPlanes[i].y * (y+size) + frustumPlanes[i].z * (z+size) + frustumPlanes[i].w > 0)
                continue;
            if(frustumPlanes[i].x * (x+size) + frustumPlanes[i].y * (y+size) + frustumPlanes[i].z * (z+size) + frustumPlanes[i].w > 0)
                continue;

            return false;
        }
        return true;
    }

    public boolean nodeFullyInFrustum(float x1, float y1, float z1, float x2, float y2, float z2) {
        for (int i = 0; i < 6; i++) {
            if (frustumPlanes[i].x * x1 + frustumPlanes[i].y * y1 + frustumPlanes[i].z * z1 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x2 + frustumPlanes[i].y * y1 + frustumPlanes[i].z * z1 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x1 + frustumPlanes[i].y * y2 + frustumPlanes[i].z * z1 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x2 + frustumPlanes[i].y * y2 + frustumPlanes[i].z * z1 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x1 + frustumPlanes[i].y * y1 + frustumPlanes[i].z * z2 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x2 + frustumPlanes[i].y * y1 + frustumPlanes[i].z * z2 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x1 + frustumPlanes[i].y * y2 + frustumPlanes[i].z * z2 + frustumPlanes[i].w <= 0.0F)
                return false;
            if (frustumPlanes[i].x * x2 + frustumPlanes[i].y * y2 + frustumPlanes[i].z * z2 + frustumPlanes[i].w <= 0.0F)
                return false;
        }
        return true;
    }

    public boolean cubeIntoFrustum(float x1, float y1, float z1, float x2, float y2, float z2) {
        for (int i = 0; i < 6; i++) {
            if (    (frustumPlanes[i].x * x1 + frustumPlanes[i].y * y1 + frustumPlanes[i].z * z1 + frustumPlanes[i].w <= 0.0F) &&
                    (frustumPlanes[i].x * x2 + frustumPlanes[i].y * y1 + frustumPlanes[i].z * z1 + frustumPlanes[i].w <= 0.0F) &&
                    (frustumPlanes[i].x * x1 + frustumPlanes[i].y * y2 + frustumPlanes[i].z * z1 + frustumPlanes[i].w <= 0.0F) &&
                    (frustumPlanes[i].x * x2 + frustumPlanes[i].y * y2 + frustumPlanes[i].z * z1 + frustumPlanes[i].w <= 0.0F) &&
                    (frustumPlanes[i].x * x1 + frustumPlanes[i].y * y1 + frustumPlanes[i].z * z2 + frustumPlanes[i].w <= 0.0F) &&
                    (frustumPlanes[i].x * x2 + frustumPlanes[i].y * y1 + frustumPlanes[i].z * z2 + frustumPlanes[i].w <= 0.0F) &&
                    (frustumPlanes[i].x * x1 + frustumPlanes[i].y * y2 + frustumPlanes[i].z * z2 + frustumPlanes[i].w <= 0.0F) &&
                    (frustumPlanes[i].x * x2 + frustumPlanes[i].y * y2 + frustumPlanes[i].z * z2 + frustumPlanes[i].w <= 0.0F)) {
                return false;
            }
        }
        return true;
    }

    public boolean cubeIntoFrustum(Vec3i min, int size){
        return cubeInFrustum(min.x, min.y, min.z, size);
    }



    public void calculateFrustum(Matrix4f mvp) {
        // ax * bx * cx +  d = 0; store a,b,c,d

        //left plane
        Vec4f leftPlane = new Vec4f(
                mvp.get(3, 0) + mvp.get(0, 0),
                mvp.get(3, 1) + mvp.get(0, 1),
                mvp.get(3, 2) + mvp.get(0, 2),
                mvp.get(3, 3) + mvp.get(0, 3));

        this.frustumPlanes[0] = Util.normalizePlane(leftPlane);

        //right plane
        Vec4f rightPlane = new Vec4f(
                mvp.get(3, 0) - mvp.get(0, 0),
                mvp.get(3, 1) - mvp.get(0, 1),
                mvp.get(3, 2) - mvp.get(0, 2),
                mvp.get(3, 3) - mvp.get(0, 3));

        this.frustumPlanes[1] = Util.normalizePlane(rightPlane);

        //bot plane
        Vec4f botPlane = new Vec4f(
                mvp.get(3, 0) + mvp.get(1, 0),
                mvp.get(3, 1) + mvp.get(1, 1),
                mvp.get(3, 2) + mvp.get(1, 2),
                mvp.get(3, 3) + mvp.get(1, 3));

        this.frustumPlanes[2] = Util.normalizePlane(botPlane);

        //top plane
        Vec4f topPlane = new Vec4f(
                mvp.get(3, 0) - mvp.get(1, 0),
                mvp.get(3, 1) - mvp.get(1, 1),
                mvp.get(3, 2) - mvp.get(1, 2),
                mvp.get(3, 3) - mvp.get(1, 3));

        this.frustumPlanes[3] = Util.normalizePlane(topPlane);

        //near plane
        Vec4f nearPlane = new Vec4f(
                mvp.get(3, 0) + mvp.get(2, 0),
                mvp.get(3, 1) + mvp.get(2, 1),
                mvp.get(3, 2) + mvp.get(2, 2),
                mvp.get(3, 3) + mvp.get(2, 3));

        this.frustumPlanes[4] = Util.normalizePlane(nearPlane);

        //far plane
        Vec4f farPlane = new Vec4f(
                mvp.get(3, 0) - mvp.get(2, 0),
                mvp.get(3, 1) - mvp.get(2, 1),
                mvp.get(3, 2) - mvp.get(2, 2),
                mvp.get(3, 3) - mvp.get(2, 3));

        this.frustumPlanes[5] = Util.normalizePlane(farPlane);

        extractFrustumCorners();
    }

    public boolean AABBInsideFrustum(Aabb aabb) {
        for (int i = 0; i < 6; i++) {
            Vec4f p = frustumPlanes[i];
            int outside = 0;
            outside += p.dot(new Vec4f(aabb.min.x, aabb.min.y, aabb.min.z, 1.f)) < 0.f ? 1 : 0;
            outside += p.dot(new Vec4f(aabb.max.x, aabb.min.y, aabb.min.z, 1.f)) < 0.f ? 1 : 0;
            outside += p.dot(new Vec4f(aabb.min.x, aabb.max.y, aabb.min.z, 1.f)) < 0.f ? 1 : 0;
            outside += p.dot(new Vec4f(aabb.max.x, aabb.max.y, aabb.min.z, 1.f)) < 0.f ? 1 : 0;
            outside += p.dot(new Vec4f(aabb.min.x, aabb.min.y, aabb.max.z, 1.f)) < 0.f ? 1 : 0;
            outside += p.dot(new Vec4f(aabb.max.x, aabb.min.y, aabb.max.z, 1.f)) < 0.f ? 1 : 0;
            outside += p.dot(new Vec4f(aabb.min.x, aabb.max.y, aabb.max.z, 1.f)) < 0.f ? 1 : 0;
            outside += p.dot(new Vec4f(aabb.max.x, aabb.max.y, aabb.max.z, 1.f)) < 0.f ? 1 : 0;

            if (outside == 8) {// all points outside the frustum
                return false;
            }
        }
        return true;
    }

    private static Vec3f IntersectionPoint(Vec4f a, Vec4f b, Vec4f c) {
        // Formula used
        //                d1 ( N2 * N3 ) + d2 ( N3 * N1 ) + d3 ( N1 * N2 )
        //P =   -------------------------------------------------------------------------
        //                             N1 . ( N2 * N3 )
        //
        // Note: N refers to the normal, d refers to the displacement. '.' means dot product. '*' means cross product

        Vec3f v1, v2, v3;
        Vec3f cross = b.getVec3f().cross(c.getVec3f());

        float f = a.getVec3f().dot(cross);
        f *= -1.0f;

        cross = b.getVec3f().cross(c.getVec3f());
        v1 = cross.mul(a.w);
        //v1 = (a.D * (Vector3.Cross(b.Normal, c.Normal)));

        cross = c.getVec3f().cross(a.getVec3f());
        v2 = cross.mul(b.w);
        //v2 = (b.D * (Vector3.Cross(c.Normal, a.Normal)));

        cross = a.getVec3f().cross(b.getVec3f());
        v3 = cross.mul(c.w);
        //v3 = (c.D * (Vector3.Cross(a.Normal, b.Normal)));

        Vec3f result = new Vec3f();
        result.X = (v1.X + v2.X + v3.X) / f;
        result.Y = (v1.Y + v2.Y + v3.Y) / f;
        result.Z = (v1.Z + v2.Z + v3.Z) / f;
        return result;
    }

    private Vec3f intersectionPoint(Vec4f a, Vec4f b, Vec4f c) {
        // Formula used
        //                d1 ( N2 * N3 ) + d2 ( N3 * N1 ) + d3 ( N1 * N2 )
        //P =   ---------------------------------------------------------------------
        //                             N1 . ( N2 * N3 )
        //
        // Note: N refers to the normal, d refers to the displacement. '.' means dot product. '*' means cross product

        Vec3f v1, v2, v3;
        float f = -a.getVec3f().dot(b.getVec3f().cross(c.getVec3f()));

        v1 = b.getVec3f().cross(c.getVec3f()).mul(a.w);
        v2 = c.getVec3f().cross(a.getVec3f()).mul(b.w);
        v3 = a.getVec3f().cross(b.getVec3f()).mul(c.w);

        Vec3f vec = new Vec3f(v1.X + v2.X + v3.X, v1.Y + v2.Y + v3.Y, v1.Z + v2.Z + v3.Z);
        return vec.div(f);
    }

    private void extractFrustumCorners() {
        frustumCorners[0] = intersectionPoint(frustumPlanes[0], frustumPlanes[2], frustumPlanes[4]);
        frustumCorners[1] = intersectionPoint(frustumPlanes[0], frustumPlanes[3], frustumPlanes[4]);
        frustumCorners[2] = intersectionPoint(frustumPlanes[0], frustumPlanes[3], frustumPlanes[5]);
        frustumCorners[3] = intersectionPoint(frustumPlanes[0], frustumPlanes[2], frustumPlanes[5]);
        frustumCorners[4] = intersectionPoint(frustumPlanes[1], frustumPlanes[2], frustumPlanes[4]);
        frustumCorners[5] = intersectionPoint(frustumPlanes[1], frustumPlanes[3], frustumPlanes[4]);
        frustumCorners[6] = intersectionPoint(frustumPlanes[1], frustumPlanes[3], frustumPlanes[5]);
        frustumCorners[7] = intersectionPoint(frustumPlanes[1], frustumPlanes[2], frustumPlanes[5]);
    }

    public Vec3f[] getFrustumCorners() {
        return frustumCorners;
    }
}