package dc.utils;

import core.math.Matrix4f;
import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.utils.Util;

public class Frustum {
    private float[][] plane = new float[6][4];
    private static final int RIGHT = 0;
    private static final int LEFT = 1;
    private static final int BOTTOM = 2;
    private static final int TOP = 3;
    private static final int BACK = 4;
    private static final int FRONT = 5;
    private static final int A = 0;
    private static final int B = 1;
    private static final int C = 2;
    private static final int D = 3;

    private Vec4f[] frustumPlanes = new Vec4f[6];
    private Vec3f[] frustumCorners = new Vec3f[8];
    private static Frustum frustum = new Frustum();

    public static Frustum getFrustum() {
        if(frustum == null)
        {
            frustum = new Frustum();
        }
        return frustum;
    }

    private void normalizePlane(float[][] frustum, int side) {
        float magnitude = (float) Math.sqrt(frustum[side][A] * frustum[side][A] + frustum[side][B] * frustum[side][B] + frustum[side][C] * frustum[side][C]);
        frustum[side][A] /= magnitude;
        frustum[side][B] /= magnitude;
        frustum[side][C] /= magnitude;
        frustum[side][D] /= magnitude;
    }

    public void calculateFrustum(Matrix4f m, Matrix4f p) {

        float[] clip = new float[16];
        float[] proj = p.contTo1dFloat();
        float[] modl = m.contTo1dFloat();

        clip[ 0] = modl[ 0] * proj[ 0] + modl[ 1] * proj[ 4] + modl[ 2] * proj[ 8] + modl[ 3] * proj[12];
        clip[ 1] = modl[ 0] * proj[ 1] + modl[ 1] * proj[ 5] + modl[ 2] * proj[ 9] + modl[ 3] * proj[13];
        clip[ 2] = modl[ 0] * proj[ 2] + modl[ 1] * proj[ 6] + modl[ 2] * proj[10] + modl[ 3] * proj[14];
        clip[ 3] = modl[ 0] * proj[ 3] + modl[ 1] * proj[ 7] + modl[ 2] * proj[11] + modl[ 3] * proj[15];

        clip[ 4] = modl[ 4] * proj[ 0] + modl[ 5] * proj[ 4] + modl[ 6] * proj[ 8] + modl[ 7] * proj[12];
        clip[ 5] = modl[ 4] * proj[ 1] + modl[ 5] * proj[ 5] + modl[ 6] * proj[ 9] + modl[ 7] * proj[13];
        clip[ 6] = modl[ 4] * proj[ 2] + modl[ 5] * proj[ 6] + modl[ 6] * proj[10] + modl[ 7] * proj[14];
        clip[ 7] = modl[ 4] * proj[ 3] + modl[ 5] * proj[ 7] + modl[ 6] * proj[11] + modl[ 7] * proj[15];

        clip[ 8] = modl[ 8] * proj[ 0] + modl[ 9] * proj[ 4] + modl[10] * proj[ 8] + modl[11] * proj[12];
        clip[ 9] = modl[ 8] * proj[ 1] + modl[ 9] * proj[ 5] + modl[10] * proj[ 9] + modl[11] * proj[13];
        clip[10] = modl[ 8] * proj[ 2] + modl[ 9] * proj[ 6] + modl[10] * proj[10] + modl[11] * proj[14];
        clip[11] = modl[ 8] * proj[ 3] + modl[ 9] * proj[ 7] + modl[10] * proj[11] + modl[11] * proj[15];

        clip[12] = modl[12] * proj[ 0] + modl[13] * proj[ 4] + modl[14] * proj[ 8] + modl[15] * proj[12];
        clip[13] = modl[12] * proj[ 1] + modl[13] * proj[ 5] + modl[14] * proj[ 9] + modl[15] * proj[13];
        clip[14] = modl[12] * proj[ 2] + modl[13] * proj[ 6] + modl[14] * proj[10] + modl[15] * proj[14];
        clip[15] = modl[12] * proj[ 3] + modl[13] * proj[ 7] + modl[14] * proj[11] + modl[15] * proj[15];

        // This will extract the LEFT side of the frustum
        plane[LEFT][A] = clip[ 3] + clip[ 0];
        plane[LEFT][B] = clip[ 7] + clip[ 4];
        plane[LEFT][C] = clip[11] + clip[ 8];
        plane[LEFT][D] = clip[15] + clip[12];
        normalizePlane(plane, LEFT);

        // This will extract the RIGHT side of the frustum
        plane[RIGHT][A] = clip[ 3] - clip[ 0];
        plane[RIGHT][B] = clip[ 7] - clip[ 4];
        plane[RIGHT][C] = clip[11] - clip[ 8];
        plane[RIGHT][D] = clip[15] - clip[12];
        normalizePlane(plane, RIGHT);

        // This will extract the BOTTOM side of the frustum
        plane[BOTTOM][A] = clip[ 3] + clip[ 1];
        plane[BOTTOM][B] = clip[ 7] + clip[ 5];
        plane[BOTTOM][C] = clip[11] + clip[ 9];
        plane[BOTTOM][D] = clip[15] + clip[13];
        normalizePlane(plane, BOTTOM);

        // This will extract the TOP side of the frustum
        plane[TOP][A] = clip[ 3] - clip[ 1];
        plane[TOP][B] = clip[ 7] - clip[ 5];
        plane[TOP][C] = clip[11] - clip[ 9];
        plane[TOP][D] = clip[15] - clip[13];
        normalizePlane(plane, TOP);

        // This will extract the FRONT side of the frustum
        plane[FRONT][A] = clip[ 3] + clip[ 2];
        plane[FRONT][B] = clip[ 7] + clip[ 6];
        plane[FRONT][C] = clip[11] + clip[10];
        plane[FRONT][D] = clip[15] + clip[14];
        normalizePlane(plane, FRONT);

        // This will extract the BACK side of the frustum
        plane[BACK][A] = clip[ 3] - clip[ 2];
        plane[BACK][B] = clip[ 7] - clip[ 6];
        plane[BACK][C] = clip[11] - clip[10];
        plane[BACK][D] = clip[15] - clip[14];
        normalizePlane(plane, BACK);
    }

    public boolean pointInFrustum(float x, float y, float z) {
        for (int i = 0; i < 6; i++) {
            if (plane[i][0] * x + plane[i][1] * y + plane[i][2] * z + plane[i][3] <= 0.0F) {
                return false;
            }
        }
        return true;
    }

    public boolean sphereInFrustum(float x, float y, float z, float radius) {
        for (int i = 0; i < 6; i++) {
            if (plane[i][0] * x + plane[i][1] * y + plane[i][2] * z + plane[i][3] <= -radius) {
                return false;
            }
        }
        return true;
    }

    public boolean cubeFullyInFrustum(float x1, float y1, float z1, float x2, float y2, float z2) {
        for (int i = 0; i < 6; i++) {
            if (plane[i][0] * x1 + plane[i][1] * y1 + plane[i][2] * z1 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x2 + plane[i][1] * y1 + plane[i][2] * z1 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x1 + plane[i][1] * y2 + plane[i][2] * z1 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x2 + plane[i][1] * y2 + plane[i][2] * z1 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x1 + plane[i][1] * y1 + plane[i][2] * z2 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x2 + plane[i][1] * y1 + plane[i][2] * z2 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x1 + plane[i][1] * y2 + plane[i][2] * z2 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x2 + plane[i][1] * y2 + plane[i][2] * z2 + plane[i][3] <= 0.0F)
                return false;
        }
        return true;
    }

    public boolean cubeInFrustum(int x, int y, int z, int size) {
        for(int i = 0; i < 6; i++ ) {
            if(plane[i][0] * (x-size) + plane[i][1] * (y-size) + plane[i][2] * (z-size) + plane[i][3] > 0)
                continue;
            if(plane[i][0] * (x+size) + plane[i][1] * (y-size) + plane[i][2] * (z-size) + plane[i][3] > 0)
                continue;
            if(plane[i][0] * (x-size) + plane[i][1] * (y+size) + plane[i][2] * (z-size) + plane[i][3] > 0)
                continue;
            if(plane[i][0] * (x+size) + plane[i][1] * (y+size) + plane[i][2] * (z-size) + plane[i][3] > 0)
                continue;
            if(plane[i][0] * (x-size) + plane[i][1] * (y-size) + plane[i][2] * (z+size) + plane[i][3] > 0)
                continue;
            if(plane[i][0] * (x+size) + plane[i][1] * (y-size) + plane[i][2] * (z+size) + plane[i][3] > 0)
                continue;
            if(plane[i][0] * (x-size) + plane[i][1] * (y+size) + plane[i][2] * (z+size) + plane[i][3] > 0)
                continue;
            if(plane[i][0] * (x+size) + plane[i][1] * (y+size) + plane[i][2] * (z+size) + plane[i][3] > 0)
                continue;

            return false;
        }
        return true;
    }

    public boolean nodeFullyInFrustum(float x1, float y1, float z1, float x2, float y2, float z2) {
        for (int i = 0; i < 6; i++) {
            if (plane[i][0] * x1 + plane[i][1] * y1 + plane[i][2] * z1 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x2 + plane[i][1] * y1 + plane[i][2] * z1 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x1 + plane[i][1] * y2 + plane[i][2] * z1 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x2 + plane[i][1] * y2 + plane[i][2] * z1 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x1 + plane[i][1] * y1 + plane[i][2] * z2 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x2 + plane[i][1] * y1 + plane[i][2] * z2 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x1 + plane[i][1] * y2 + plane[i][2] * z2 + plane[i][3] <= 0.0F)
                return false;
            if (plane[i][0] * x2 + plane[i][1] * y2 + plane[i][2] * z2 + plane[i][3] <= 0.0F)
                return false;
        }
        return true;
    }

    public boolean cubeIntoFrustum(float x1, float y1, float z1, float x2, float y2, float z2) {
        for (int i = 0; i < 6; i++) {
            if (    (plane[i][0] * x1 + plane[i][1] * y1 + plane[i][2] * z1 + plane[i][3] <= 0.0F) &&
                    (plane[i][0] * x2 + plane[i][1] * y1 + plane[i][2] * z1 + plane[i][3] <= 0.0F) &&
                    (plane[i][0] * x1 + plane[i][1] * y2 + plane[i][2] * z1 + plane[i][3] <= 0.0F) &&
                    (plane[i][0] * x2 + plane[i][1] * y2 + plane[i][2] * z1 + plane[i][3] <= 0.0F) &&
                    (plane[i][0] * x1 + plane[i][1] * y1 + plane[i][2] * z2 + plane[i][3] <= 0.0F) &&
                    (plane[i][0] * x2 + plane[i][1] * y1 + plane[i][2] * z2 + plane[i][3] <= 0.0F) &&
                    (plane[i][0] * x1 + plane[i][1] * y2 + plane[i][2] * z2 + plane[i][3] <= 0.0F) &&
                    (plane[i][0] * x2 + plane[i][1] * y2 + plane[i][2] * z2 + plane[i][3] <= 0.0F)) {
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

    private void extractFrustumCorners(Vec3f[] corners) {
        corners[0] = intersectionPoint(frustumPlanes[0], frustumPlanes[2], frustumPlanes[4]);
        corners[1] = intersectionPoint(frustumPlanes[0], frustumPlanes[3], frustumPlanes[4]);
        corners[2] = intersectionPoint(frustumPlanes[0], frustumPlanes[3], frustumPlanes[5]);
        corners[3] = intersectionPoint(frustumPlanes[0], frustumPlanes[2], frustumPlanes[5]);
        corners[4] = intersectionPoint(frustumPlanes[1], frustumPlanes[2], frustumPlanes[4]);
        corners[5] = intersectionPoint(frustumPlanes[1], frustumPlanes[3], frustumPlanes[4]);
        corners[6] = intersectionPoint(frustumPlanes[1], frustumPlanes[3], frustumPlanes[5]);
        corners[7] = intersectionPoint(frustumPlanes[1], frustumPlanes[2], frustumPlanes[5]);
    }

    public Vec3f[] getFrustumCorners() {
        return frustumCorners;
    }
}