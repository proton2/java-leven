package dc.svd;

import core.math.Vec2f;
import core.math.Vec3f;
import core.math.Vec4f;

/**
 * Created by proton2 on 01.02.2020.
 */
public class GlslSvd {

    private final int SVD_NUM_SWEEPS = 5;
    private final float Tiny_Number = (float) 1.e-20;
    //private final float Tiny_Number = 1e-6f;

    private float rsqrt(float x) {
        return (float) (1.0 / Math.sqrt(x));
    }

    private Vec2f givens_coeffs_sym(float a_pp, float a_pq, float a_qq) {
        Vec2f f = new Vec2f();
        if (a_pq == 0.0) {
            f.X = 1;
            f.Y = 0;
            return f;
        }
        float tau = (a_qq - a_pp) / (2 * a_pq);
        float stt = (float) Math.sqrt(1.0 + tau * tau);
        float tan = 1.0f / ((tau >= 0) ? (tau + stt) : (tau - stt));
        f.X = rsqrt(1 + tan * tan);
        f.Y = tan * f.X;
        return f;
    }

    private Vec2f svd_rotate_xy(float x, float y, float c, float s) {
        float u = x;
        float v = y;
        Vec2f a = new Vec2f();
        a.X = c * u - s * v;
        a.Y = s * u + c * v;
        return a;
    }

    private Vec2f svd_rotateq_xy(float x, float y, float a, float c, float s) {
        float cc = c * c;
        float ss = s * s;
        float mx = 2 * c * s * a;
        float u = x; float v = y;
        Vec2f val = new Vec2f();
        val.X = cc * u - mx + ss * v;
        val.Y = ss * u + mx + cc * v;
        return val;
    }

    private void svd_rotate(float[][] vtav, float[][]v, int a, int b) {
        if (vtav[a][b] == 0.0) return;

        Vec2f f = givens_coeffs_sym(vtav[a][a], vtav[a][b], vtav[b][b]);
        Vec2f val = svd_rotateq_xy(vtav[a][a], vtav[b][b], vtav[a][b], f.X, f.Y);
        vtav[a][a] = val.X; vtav[b][b] = val.Y;
        val = svd_rotate_xy(vtav[0][3-b], vtav[1-a][2], f.X, f.Y);
        vtav[0][3-b] = val.X; vtav[1-a][2] = val.Y;
        vtav[a][b] = 0;

        val = svd_rotate_xy(v[0][a], v[0][b], f.X, f.Y);
        v[0][a] = val.X; v[0][b] = val.Y;
        val = svd_rotate_xy(v[1][a], v[1][b], f.X, f.Y);
        v[1][a] = val.X; v[1][b] = val.Y;
        val = svd_rotate_xy(v[2][a], v[2][b], f.X, f.Y);
        v[2][a] = val.X; v[2][b] = val.Y;
    }

    float[][] SMatToMat(float[] a){
        float[][] mat = new float[3][3];
        mat[0][0] = a[0]; mat[0][1] = a[1]; mat[0][2] = a[2];
        mat[1][0] = 0.f;  mat[1][1] = a[3]; mat[1][2] = a[4];
        mat[2][0] = 0.f;  mat[2][1] = 0.f;  mat[2][2] = a[5];
        return mat;
    }

    private float[] svd_solve_sym(float[] a, float[][] v) {
        // assuming that A is symmetric: can optimize all operations for the upper right triagonal
        // assuming V is identity: you can also pass a matrix the rotations should be applied to U is not computed
        float[][] vtav = new float[3][3];
        vtav[0][0] = a[0];   vtav[0][1] = a[1];   vtav[0][2] = a[2];
        vtav[1][0] = 0.f;    vtav[1][1] = a[3];   vtav[1][2] = a[4];
        vtav[2][0] = 0.f;    vtav[2][1] = 0.f;    vtav[2][2] = a[5];

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
        float[][] o = new float[3][3];
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

    Vec3f svd_solve_ATA_ATb(float[] ATA, Vec3f ATb) {
        float[][] V = { { 1, 0, 0 }, { 0, 1, 0 }, { 0, 0, 1 } }; // mat3 V = mat3(1.0);
        float[] sigma = svd_solve_sym(ATA, V);
        float[][] vInv = svd_pseudoinverse(sigma, V);            // A = UEV^T; U = A / (E*V^T)
        return vmulSym(vInv, ATb); // x = vInv * ATb;
    }

    public Vec3f vmulSym(float[] a, Vec3f v) {
        return new Vec3f(
                (a[0] * v.X) + (a[1] * v.Y) + (a[2] * v.Z),
                (a[1] * v.X) + (a[3] * v.Y) + (a[4] * v.Z),
                (a[2] * v.X) + (a[4] * v.Y) + (a[5] * v.Z));
    }

    private Vec3f vmulSym(float[][] a, Vec3f v) {
        return new Vec3f(
                (a[0][0] * v.X) + (a[0][1] * v.Y) + (a[0][2] * v.Z),
                (a[0][1] * v.X) + (a[1][1] * v.Y) + (a[1][2] * v.Z),
                (a[0][2] * v.X) + (a[1][2] * v.Y) + (a[2][2] * v.Z));
    }

    private void qef_add(Vec3f n, Vec3f p, float[] ATA, Vec3f ATb, Vec4f pointaccum) {
        ATA[0] += n.X * n.X;
        ATA[1] += n.X * n.Y;
        ATA[2] += n.X * n.Z;
        ATA[3] += n.Y * n.Y;
        ATA[4] += n.Y * n.Z;
        ATA[5] += n.Z * n.Z;
        float dot = p.dot(n);
        ATb.X += dot * n.X;
        ATb.Y += dot * n.Y;
        ATb.Z += dot * n.Z;
        pointaccum.set(pointaccum.add(p,1.0f));
    }

    float qef_calc_error(float[] A, Vec3f x, Vec3f b) {
//        Vec3f atax = this.ata.Vmul(pos);
//        return pos.dot(atax) - 2 * pos.dot(atb) + data.btb;

        Vec3f vtmp = b.sub(vmulSym(A, x));
        return vtmp.dot(vtmp);
    }

    private float qef_solve(float[] ATA, Vec3f ATb, Vec4f pointaccum, Vec3f x) {
        Vec3f masspoint = pointaccum.div(pointaccum.w).getVec3f();
        Vec3f tmpv = vmulSym(ATA, masspoint);
        ATb = ATb.sub(tmpv);

        x.set(svd_solve_ATA_ATb(ATA, ATb));
        float result = qef_calc_error(ATA, x, ATb);
        x.set(x.add(masspoint));
        return result;
    }

    public static void main(String[] args) {
        Vec4f pointaccum = new Vec4f(0,0,0,0);
        float[] ATA = new float[6];
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
        Vec3f com = pointaccum.div(pointaccum.w).getVec3f();

        Vec3f x = new Vec3f();
        float error = solver.qef_solve(ATA, ATb, pointaccum, x);

        System.out.println(String.format("masspoint = (%.5f %.5f %.5f)\n", com.X, com.Y, com.Z));
        System.out.println(String.format("point = (%.5f %.5f %.5f)\n", x.X, x.Y, x.Z));
        System.out.println(String.format("error = %.5f\n", error));
    }
}
