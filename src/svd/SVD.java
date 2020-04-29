package svd;

import core.math.Vec3f;

public class SVD {
    public static float calcError(SMat3 origA, Vec3f x, Vec3f b) {
        Mat3 A = new Mat3();
        A.setSymmetric(origA);
        Vec3f vtmp = A.Vmul(x);
        vtmp = b.sub(vtmp);
        return vtmp.dot(vtmp);
    }

    public static void rotate01(SMat3 vtav, Mat3 v) {
        if (vtav.m01 == 0)
            return;

        FloatHolder f = vtav.rot01();
        //f.set(0,0);
        v.rot01_post(f.c, f.s);
    }

    public static void rotate02(SMat3 vtav, Mat3 v) {
        if (vtav.m02 == 0)
            return;

        FloatHolder f = vtav.rot02();
        //f.set(0,0);
        v.rot02_post(f.c, f.s);
    }

    public static void rotate12(SMat3 vtav, Mat3 v) {
        if (vtav.m12 == 0)
            return;

        FloatHolder f = vtav.rot12();
        //f.set(0,0);
        v.rot12_post(f.c, f.s);
    }

    public static void getSymmetricSvd(final SMat3 a, SMat3 vtav, Mat3 v, final float tol, final int max_sweeps) {
        vtav.setSymmetric(a);
        v.set(1, 0, 0, 0, 1, 0, 0, 0, 1);
        float delta = tol * vtav.Fnorm();

        for (int i = 0; i < max_sweeps
                && vtav.Off() > delta; ++i)
        {
            rotate01(vtav, v);
            rotate02(vtav, v);
            rotate12(vtav, v);
        }
    }

    static float pinv(final float x, final float tol) {
        return (Math.abs(x) < tol || Math.abs(1 / x) < tol) ? 0 : (1 / x);
    }

    public static Mat3 psuedoinverse(SMat3 d, Mat3 v, float tol) {
        float   d0 = pinv(d.m00, tol),
                d1 = pinv(d.m11, tol),
                d2 = pinv(d.m22, tol);
        Mat3 mn = new Mat3();
        mn.set( v.m00 * d0 * v.m00 + v.m01 * d1 * v.m01 + v.m02 * d2 * v.m02,
                v.m00 * d0 * v.m10 + v.m01 * d1 * v.m11 + v.m02 * d2 * v.m12,
                v.m00 * d0 * v.m20 + v.m01 * d1 * v.m21 + v.m02 * d2 * v.m22,
                v.m10 * d0 * v.m00 + v.m11 * d1 * v.m01 + v.m12 * d2 * v.m02,
                v.m10 * d0 * v.m10 + v.m11 * d1 * v.m11 + v.m12 * d2 * v.m12,
                v.m10 * d0 * v.m20 + v.m11 * d1 * v.m21 + v.m12 * d2 * v.m22,
                v.m20 * d0 * v.m00 + v.m21 * d1 * v.m01 + v.m22 * d2 * v.m02,
                v.m20 * d0 * v.m10 + v.m21 * d1 * v.m11 + v.m22 * d2 * v.m12,
                v.m20 * d0 * v.m20 + v.m21 * d1 * v.m21 + v.m22 * d2 * v.m22);
        return mn;
    }

    public static float solveSymmetric(SMat3 A, Vec3f b, Vec3f x,
                                       float svd_tol, int svd_sweeps, float pinv_tol)
    {
        Mat3 V = new Mat3();
        SMat3 VTAV = new SMat3();
        getSymmetricSvd(A, VTAV, V, svd_tol, svd_sweeps);
        Mat3 pinv = psuedoinverse(VTAV, V, pinv_tol);

        x.set(pinv.Vmul(b));
        return calcError(A, x, b);
    }

    public static void normalize(Vec3f v) {
        float len2 = v.X * v.X + v.Y * v.Y + v.Z * v.Z;
        scale(v, (float) (1 / Math.sqrt(len2)));
    }

    private static void scale(Vec3f v, float s) {
        v.X *= s;
        v.Y *= s;
        v.Z *= s;
    }

    public static Vec3f Normalize(Vec3f v) {
        float len2 = v.X * v.X + v.Y * v.Y + v.Z * v.Z;
        scale(v, (float) (1 / Math.sqrt(len2)));
        return v;
    }
}

