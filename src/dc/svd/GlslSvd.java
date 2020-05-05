package dc.svd;

import core.math.Vec2f;
import core.math.Vec3f;

/**
 * Created by proton2 on 01.02.2020.
 */
public class GlslSvd {
    public static class Vec4f extends Vec3f{
        public float w;
        public Vec4f(){}
        public Vec4f (Vec3f v, float w){
            X = v.X; Y = v.Y; Z = v.Z; this.w = w;
        }

        public Vec4f(float x, float y, float z, float w) {
            X = x; Y = y; Z = z; this.w = w;
        }

        public void set (Vec4f v){
            X = v.X; Y = v.Y; Z = v.Z; this.w = v.w;
        }

        public void add(Vec3f r, float w) {
            this.X += r.getX(); this.Y += r.getY(); this.Z += r.getZ(); this.w += w;
        }
    }

    private final int SVD_NUM_SWEEPS = 5;
    private final float Tiny_Number = (float) 1.e-20;
    //private final float Tiny_Number = 1e-6f;

    private float rsqrt(float x) {
        return (float) (1.0 / Math.sqrt(x));
    }

    private FloatHolder givens_coeffs_sym(float a_pp, float a_pq, float a_qq) {
        FloatHolder f = new FloatHolder();
        if (a_pq == 0.0) {
            f.c = 1;
            f.s = 0;
            return f;
        }
        float tau = (a_qq - a_pp) / (2 * a_pq);
        float stt = (float) Math.sqrt(1.0 + tau * tau);
        float tan = 1.0f / ((tau >= 0) ? (tau + stt) : (tau - stt));
        f.c = rsqrt(1 + tan * tan);
        f.s = tan * f.c;
        return f;
    }

    private Vec2f svd_rotate_xy(float x, float y, FloatHolder f) {
        float u = x;
        float v = y;
        Vec2f a = new Vec2f();
        a.X = f.c * u - f.s * v;
        a.Y = f.s * u + f.c * v;
        return a;
    }

    private Vec2f svd_rotateq_xy(float x, float y, float a, FloatHolder f) {
        float cc = f.c * f.c;
        float ss = f.s * f.s;
        float mx = 2 * f.c * f.s * a;
        float u = x; float v = y;
        Vec2f val = new Vec2f();
        val.X = cc * u - mx + ss * v;
        val.Y = ss * u + mx + cc * v;
        return val;
    }

    private void svd_rotate01(SMat3 vtav, Mat3 v) {
        if (vtav.m01 == 0.0) return;

        FloatHolder f = givens_coeffs_sym(vtav.m00, vtav.m01, vtav.m11);
        Vec2f val = svd_rotateq_xy(vtav.m00, vtav.m11, vtav.m01, f);
        vtav.m00 = val.X; vtav.m11 = val.Y;

        val = svd_rotate_xy(vtav.m02, vtav.m12, f);
        vtav.m02 = val.X; vtav.m12 = val.Y;

        vtav.m01 = 0;

        val = svd_rotate_xy(v.m00, v.m01, f);
        v.m00 = val.X; v.m01 = val.Y;
        val = svd_rotate_xy(v.m10, v.m11, f);
        v.m10 = val.X; v.m11 = val.Y;
        val = svd_rotate_xy(v.m20, v.m21, f);
        v.m20 = val.X; v.m21 = val.Y;
    }

    private void svd_rotate02(SMat3 vtav, Mat3 v) {
        if (vtav.m02 == 0.0) return;

        FloatHolder f = givens_coeffs_sym(vtav.m00, vtav.m02, vtav.m22);
        Vec2f val = svd_rotateq_xy(vtav.m00, vtav.m22, vtav.m02, f);
        vtav.m00 = val.X; vtav.m22 = val.Y;

        val = svd_rotate_xy(vtav.m01, vtav.m12, f);
        vtav.m01 = val.X; vtav.m12 = val.Y;
        vtav.m02 = 0;

        val = svd_rotate_xy(v.m00, v.m02, f);
        v.m00 = val.X; v.m02 = val.Y;
        val = svd_rotate_xy(v.m10, v.m12, f);
        v.m10 = val.X; v.m12 = val.Y;
        val = svd_rotate_xy(v.m20, v.m22, f);
        v.m20 = val.X; v.m22 = val.Y;
    }

    private void svd_rotate12(SMat3 vtav, Mat3 v) {
        if (vtav.m12 == 0.0) return;

        FloatHolder f = givens_coeffs_sym(vtav.m11, vtav.m12, vtav.m22);
        Vec2f val = svd_rotateq_xy(vtav.m11,vtav.m22,vtav.m12,f);
        vtav.m11 = val.X; vtav.m22 = val.Y;

        val = svd_rotate_xy(vtav.m01, vtav.m02, f);
        vtav.m01 = val.X; vtav.m02 = val.Y;
        vtav.m12 = 0;

        val = svd_rotate_xy(v.m01, v.m02, f);
        v.m01 = val.X; v.m02 = val.Y;
        val = svd_rotate_xy(v.m11, v.m12, f);
        v.m11 = val.X; v.m12 = val.Y;
        val = svd_rotate_xy(v.m21, v.m22, f);
        v.m21 = val.X; v.m22 = val.Y;
    }

    private void svd_rotate(float[][] vtav, float[][]v, int a, int b) {
        if (vtav[a][b] == 0.0) return;

        FloatHolder f = givens_coeffs_sym(vtav[a][a], vtav[a][b], vtav[b][b]);
        Vec2f val = svd_rotateq_xy(vtav[a][a], vtav[b][b], vtav[a][b], f);
        vtav[a][a] = val.X; vtav[b][b] = val.Y;
        val = svd_rotate_xy(vtav[0][3-b], vtav[1-a][2], f);
        vtav[0][3-b] = val.X; vtav[1-a][2] = val.Y;
        vtav[a][b] = 0;

        val = svd_rotate_xy(v[0][a], v[0][b], f);
        v[0][a] = val.X; v[0][b] = val.Y;
        val = svd_rotate_xy(v[1][a], v[1][b], f);
        v[1][a] = val.X; v[1][b] = val.Y;
        val = svd_rotate_xy(v[2][a], v[2][b], f);
        v[2][a] = val.X; v[2][b] = val.Y;
    }

    private Vec3f solveSymmetric(SMat3 vtav, Mat3 v) {
        for(int i = 0; i < SVD_NUM_SWEEPS; ++i) {
            // Rotate the upper right (lower left) triagonal.
            svd_rotate01(vtav, v);
            svd_rotate02(vtav, v);
            svd_rotate12(vtav, v);
        }
        return new Vec3f(vtav.m00, vtav.m11,vtav.m22);
    }

    float[][] SMatToMat(float[] a){
        float[][] mat = new float[3][3];
        mat[0][0] = a[0]; mat[0][1] = a[1]; mat[0][2] = a[2];
        mat[1][0] = 0.f;  mat[1][1] = a[3]; mat[1][2] = a[4];
        mat[2][0] = 0.f;  mat[2][1] = 0.f;  mat[2][2] = a[5];
        return mat;
    }

    private float[] svd_solve_sym(float[][] a, float[][] v) {
        // assuming that A is symmetric: can optimize all operations for the upper right triagonal
        // assuming V is identity: you can also pass a matrix the rotations should be applied to U is not computed
        float[][] vtav = new float[3][3];
        vtav[0][0] = a[0][0];   vtav[0][1] = a[0][1];   vtav[0][2] = a[0][2];
        vtav[1][0] = 0.f;       vtav[1][1] = a[1][1];   vtav[1][2] = a[1][2];
        vtav[2][0] = 0.f;       vtav[2][1] = 0.f;       vtav[2][2] = a[2][2];

        for (int i = 0; i < SVD_NUM_SWEEPS; ++i) {
            svd_rotate(vtav, v, 0, 1);
            svd_rotate(vtav, v, 0, 2);
            svd_rotate(vtav, v, 1, 2);
        }
        return new float[] {vtav[0][0],vtav[1][1],vtav[2][2]};
    }

    private float svd_invdet(float x, float tol) {
        return (float) ((Math.abs(x) < tol || Math.abs(1.0 / x) < tol) ? 0.0 : (1.0 / x));
    }

    private float[][] svd_pseudoinverse(float[] sigma, float[][] v) {
        float d0 = svd_invdet(sigma[0], Tiny_Number);
        float d1 = svd_invdet(sigma[1], Tiny_Number);
        float d2 = svd_invdet(sigma[2], Tiny_Number);
        float o[][] = new float[3][3];
        o[0][0] = v[0][0] * d0 * v[0][0] + v[0][1] * d1 * v[0][1] + v[0][2] * d2 * v[0][2];
        o[0][1] = v[0][0] * d0 * v[1][0] + v[0][1] * d1 * v[1][1] + v[0][2] * d2 * v[1][2];
        o[0][2] = v[0][0] * d0 * v[2][0] + v[0][1] * d1 * v[2][1] + v[0][2] * d2 * v[2][2];
        o[1][0] = v[1][0] * d0 * v[0][0] + v[1][1] * d1 * v[0][1] + v[1][2] * d2 * v[0][2];
        o[1][1] = v[1][0] * d0 * v[1][0] + v[1][1] * d1 * v[1][1] + v[1][2] * d2 * v[1][2];
        o[1][2] = v[1][0] * d0 * v[2][0] + v[1][1] * d1 * v[2][1] + v[1][2] * d2 * v[2][2];
        o[2][0] = v[2][0] * d0 * v[0][0] + v[2][1] * d1 * v[0][1] + v[2][2] * d2 * v[0][2];
        o[2][1] = v[2][0] * d0 * v[1][0] + v[2][1] * d1 * v[1][1] + v[2][2] * d2 * v[1][2];
        o[2][2] = v[2][0] * d0 * v[2][0] + v[2][1] * d1 * v[2][1] + v[2][2] * d2 * v[2][2];
        return o;
    }

    float unExponent(float number, int w){
        // float number = t;//12.43543542f;
        // int aux = (int)(number*100);//1243
        // float result = aux/100f;//12.43
        int aux = (int)(number*w);
        return aux/(float)w;
    }

    //roundAvoid(260.775d, 2)); // OUTPUTS: 260.77 instead of expected 260.78
    public static float roundAvoid(float value, int nDigits) {
        float scale = (float) Math.pow(10, nDigits);
        return Math.round(value * scale) / scale;
    }

    void svd_solve_ATA_ATb(float[][] ATA, Vec3f ATb, Vec3f x) {
        float[][] V = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } }; // mat3 V = mat3(1.0);
        float[] sigma = svd_solve_sym(ATA, V);
        float[][] vInv = svd_pseudoinverse(sigma, V);            // A = UEV^T; U = A / (E*V^T)
        x.set(vmulSym(vInv, ATb));                               // x = vInv * ATb;
    }

    Vec3f solve(SMat3 ata, Vec3f atb) {
        Mat3 V = new Mat3().identity();
		Vec3f sigma = solveSymmetric(ata.copy(), V);
        float[][] invV = svd_pseudoinverse(sigma.convTo1dFloat(), V.convTo2dFloat());
        return vmulSym(invV, atb); //x.copy(atb).applyMatrix3(invV);

    }

    private Vec3f vmulSym(float[][] a, Vec3f v) {
        return new Vec3f(
                (a[0][0] * v.X) + (a[0][1] * v.Y) + (a[0][2] * v.Z),
                (a[0][1] * v.X) + (a[1][1] * v.Y) + (a[1][2] * v.Z),
                (a[0][2] * v.X) + (a[1][2] * v.Y) + (a[2][2] * v.Z));
    }

    private SMat3 svd_mul_ata_sym(Mat3 a){
        SMat3 o = new SMat3();
        o.m00 = a.m00 * a.m00 + a.m10 * a.m10 + a.m20 * a.m20;
        o.m01 = a.m00 * a.m01 + a.m10 * a.m11 + a.m20 * a.m21;
        o.m02 = a.m00 * a.m02 + a.m10 * a.m12 + a.m20 * a.m22;
        o.m11 = a.m01 * a.m01 + a.m11 * a.m11 + a.m21 * a.m21;
        o.m12 = a.m01 * a.m02 + a.m11 * a.m12 + a.m21 * a.m22;
        o.m22 = a.m02 * a.m02 + a.m12 * a.m12 + a.m22 * a.m22;
        return o;
    }

    void svd_solve_Ax_b(Mat3 a, Vec3f b, SMat3 ATA, Vec3f ATb, Vec3f x) {
        ATA.set(svd_mul_ata_sym(a));
        ATb.set(a.Vmul(b)); // transpose(a) * b;
        svd_solve_ATA_ATb(ATA.convTo2dFloat(), ATb, x);
    }

    private void qef_add(Vec3f n, Vec3f p, float[][] ATA, Vec3f ATb, Vec4f pointaccum) {
        ATA[0][0] += n.X * n.X;
        ATA[0][1] += n.X * n.Y;
        ATA[0][2] += n.X * n.Z;
        ATA[1][1] += n.Y * n.Y;
        ATA[1][2] += n.Y * n.Z;
        ATA[2][2] += n.Z * n.Z;
        float dot = p.dot(n);
        ATb.X += dot * n.X;
        ATb.Y += dot * n.Y;
        ATb.Z += dot * n.Z;
        pointaccum.add(p,1.0f);
    }

    float qef_calc_error(float[][] A, Vec3f x, Vec3f b) {
//        Vec3f atax = this.ata.Vmul(pos);
//        return pos.dot(atax) - 2 * pos.dot(atb) + data.btb;

        Vec3f vtmp = b.sub(vmulSym(A, x));
        return vtmp.dot(vtmp);
    }

    private float qef_solve(float[][] ATA, Vec3f ATb, Vec4f pointaccum, Vec3f x) {
        Vec3f masspoint = pointaccum.div(pointaccum.w);
        Vec3f tmpv = vmulSym(ATA, masspoint);
        ATb = ATb.sub(tmpv);

        svd_solve_ATA_ATb(ATA, ATb, x);
        float result = qef_calc_error(ATA, x, ATb);
        x.set(x.add(masspoint));
        return result;
    }

    public static void main(String[] args) {
        Vec4f pointaccum = new Vec4f(0,0,0,0);
        float[][] ATA = new float[3][3];
        Vec3f ATb = new Vec3f();

        final int count = 5;
        Vec3f[] normals = {
                new Vec3f( 1.0f,1.0f,0.0f).normalize(),
                new Vec3f( 1.0f,1.0f,0.0f).normalize(),
                new Vec3f(-1.0f,1.0f,0.0f).normalize(),
                new Vec3f(-1.0f,2.0f,1.0f).normalize(),
                new Vec3f(-1.0f,1.0f,0.0f).normalize(),
        };
        Vec3f[] points = {
                new Vec3f(  1.0f, 0.0f, 0.3f),
                new Vec3f(  0.9f, 0.1f, -0.5f),
                new Vec3f( -0.8f, 0.2f, 0.6f),
                new Vec3f( -1.0f, 0.0f, 0.01f),
                new Vec3f( -1.1f, -0.1f, -0.5f),
        };

        GlslSvd solver = new GlslSvd();

        for (int i= 0; i < count; ++i) {
            solver.qef_add(normals[i], points[i], ATA, ATb, pointaccum);
        }
        Vec3f com = pointaccum.div(pointaccum.w);

        Vec3f x = new Vec3f();
        float error = solver.qef_solve(ATA, ATb, pointaccum, x);

        System.out.println(String.format("masspoint = (%.5f %.5f %.5f)\n", com.X, com.Y, com.Z));
        System.out.println(String.format("point = (%.5f %.5f %.5f)\n", x.X, x.Y, x.Z));
        System.out.println(String.format("error = %.5f\n", error));
    }
}
