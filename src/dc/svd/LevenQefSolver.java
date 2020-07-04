package dc.svd;

import core.math.Vec2f;
import core.math.Vec3f;
import core.math.Vec4f;

public class LevenQefSolver {
    private float[]   mat3x3_tri_ATA = new float[6];
    private Vec4f     ATb;
    private Vec4f	  masspoint;

    int SVD_NUM_SWEEPS = 10;
    float PSUEDO_INVERSE_THRESHOLD = 0.1f;

    private float rsqrt(float a) {
        return (float) Math.pow(a, -0.5);
    }

    private void svd_mul_matrix_vec(Vec4f result, float[][] mat3x3_a, Vec4f b) {
        result.x = b.dot(new Vec4f(mat3x3_a[0][0], mat3x3_a[0][1], mat3x3_a[0][2], 0.f));
        result.y = b.dot(new Vec4f(mat3x3_a[1][0], mat3x3_a[1][1], mat3x3_a[1][2], 0.f));
        result.z = b.dot(new Vec4f(mat3x3_a[2][0], mat3x3_a[2][1], mat3x3_a[2][2], 0.f));
        result.w = 0.f;
    }

    private void givens_coeffs_sym(float a_pp, float a_pq, float a_qq, Vec2f cs) {
        if (a_pq == 0.f) {
		    cs.X = 1.f;
		    cs.Y = 0.f;
            return;
        }
        float tau = (a_qq - a_pp) / (2.f * a_pq);
        float stt = (float) Math.sqrt(1.f + tau * tau);
        float tan = 1.f / ((tau >= 0.f) ? (tau + stt) : (tau - stt));
	    cs.X = rsqrt(1.f + tan * tan);
	    cs.Y = tan * (cs.X);
    }

    private void svd_rotate_xy(Vec2f xy, float c, float s) {
        float u = xy.X;
        float v = xy.Y;
	    xy.X = c * u - s * v;
	    xy.Y = s * u + c * v;
    }

    private void svd_rotateq_xy(Vec3f xya, float c, float s) {
        float cc = c * c;
        float ss = s * s;
        float mx = 2 * c * s * (xya.getZ());
        float u = xya.X;
        float v = xya.Y;
        xya.X = cc * u - mx + ss * v;
        xya.Y = ss * u + mx + cc * v;
    }

    private void svd_rotate(float[][] mat3x3_vtav, float[][] mat3x3_v, int a, int b) {
        if (mat3x3_vtav[a][b] == 0.0) return;

        Vec2f cs = new Vec2f();
        givens_coeffs_sym(mat3x3_vtav[a][a], mat3x3_vtav[a][b], mat3x3_vtav[b][b], cs);

        Vec3f xyz = new Vec3f(mat3x3_vtav[a][a], mat3x3_vtav[b][b], mat3x3_vtav[a][b]);
        svd_rotateq_xy(xyz, cs.X,cs.Y);
        mat3x3_vtav[a][a] = xyz.X; mat3x3_vtav[b][b] = xyz.Y; mat3x3_vtav[a][b] = xyz.Z;

        Vec2f xy = new Vec2f(mat3x3_vtav[0][3-b], mat3x3_vtav[1-a][2]);
        svd_rotate_xy(xy, cs.X, cs.Y);
        mat3x3_vtav[0][3-b] = xy.X; mat3x3_vtav[1-a][2] = xy.Y;

        mat3x3_vtav[a][b] = 0f;

        xy.X = mat3x3_v[0][a]; xy.Y = mat3x3_v[0][b];
        svd_rotate_xy(xy, cs.X, cs.Y);
        mat3x3_v[0][a] = xy.X; mat3x3_v[0][b] = xy.Y;

        xy.X = mat3x3_v[1][a]; xy.Y = mat3x3_v[1][b];
        svd_rotate_xy(xy, cs.X, cs.Y);
        mat3x3_v[1][a] = xy.X; mat3x3_v[1][b] = xy.Y;

        xy.X = mat3x3_v[2][a]; xy.Y = mat3x3_v[2][b];
        svd_rotate_xy(xy, cs.X, cs.Y);
        mat3x3_v[2][a] = xy.X; mat3x3_v[2][b] = xy.Y;
    }

    private void svd_solve_sym(float[] mat3x3_tri_a, Vec4f sigma, float[][] mat3x3_v) {
        // assuming that A is symmetric: can optimize all operations for the upper right triagonal
        float[][] mat3x3_vtav = new float[3][3];
        mat3x3_vtav[0][0] = mat3x3_tri_a[0];    mat3x3_vtav[0][1] = mat3x3_tri_a[1];    mat3x3_vtav[0][2] = mat3x3_tri_a[2];
        mat3x3_vtav[1][0] = 0.f;                mat3x3_vtav[1][1] = mat3x3_tri_a[3];    mat3x3_vtav[1][2] = mat3x3_tri_a[4];
        mat3x3_vtav[2][0] = 0.f;                mat3x3_vtav[2][1] = 0.f;                mat3x3_vtav[2][2] = mat3x3_tri_a[5];

        // assuming V is identity: you can also pass a matrix the rotations should be applied to. (U is not computed)
        for (int i = 0; i < SVD_NUM_SWEEPS; ++i) {
            svd_rotate(mat3x3_vtav, mat3x3_v, 0, 1);
            svd_rotate(mat3x3_vtav, mat3x3_v, 0, 2);
            svd_rotate(mat3x3_vtav, mat3x3_v, 1, 2);
        }

        sigma.setX(mat3x3_vtav[0][0]);
        sigma.setY(mat3x3_vtav[1][1]);
        sigma.setZ(mat3x3_vtav[2][2]);
        sigma.setW(0.f);
    }

    private float svd_invdet(float x, float tol) {
        return (float) ((Math.abs(x) < tol || Math.abs(1.0 / x) < tol) ? 0.0 : (1.0 / x));
    }

    private void svd_pseudoinverse(float[][] mat3x3_o, Vec4f sigma, float[][] mat3x3_v) {
        float d0 = svd_invdet(sigma.x, PSUEDO_INVERSE_THRESHOLD);
        float d1 = svd_invdet(sigma.y, PSUEDO_INVERSE_THRESHOLD);
        float d2 = svd_invdet(sigma.z, PSUEDO_INVERSE_THRESHOLD);

        mat3x3_o[0][0] = mat3x3_v[0][0] * d0 * mat3x3_v[0][0] + mat3x3_v[0][1] * d1 * mat3x3_v[0][1] + mat3x3_v[0][2] * d2 * mat3x3_v[0][2];
        mat3x3_o[0][1] = mat3x3_v[0][0] * d0 * mat3x3_v[1][0] + mat3x3_v[0][1] * d1 * mat3x3_v[1][1] + mat3x3_v[0][2] * d2 * mat3x3_v[1][2];
        mat3x3_o[0][2] = mat3x3_v[0][0] * d0 * mat3x3_v[2][0] + mat3x3_v[0][1] * d1 * mat3x3_v[2][1] + mat3x3_v[0][2] * d2 * mat3x3_v[2][2];
        mat3x3_o[1][0] = mat3x3_v[1][0] * d0 * mat3x3_v[0][0] + mat3x3_v[1][1] * d1 * mat3x3_v[0][1] + mat3x3_v[1][2] * d2 * mat3x3_v[0][2];
        mat3x3_o[1][1] = mat3x3_v[1][0] * d0 * mat3x3_v[1][0] + mat3x3_v[1][1] * d1 * mat3x3_v[1][1] + mat3x3_v[1][2] * d2 * mat3x3_v[1][2];
        mat3x3_o[1][2] = mat3x3_v[1][0] * d0 * mat3x3_v[2][0] + mat3x3_v[1][1] * d1 * mat3x3_v[2][1] + mat3x3_v[1][2] * d2 * mat3x3_v[2][2];
        mat3x3_o[2][0] = mat3x3_v[2][0] * d0 * mat3x3_v[0][0] + mat3x3_v[2][1] * d1 * mat3x3_v[0][1] + mat3x3_v[2][2] * d2 * mat3x3_v[0][2];
        mat3x3_o[2][1] = mat3x3_v[2][0] * d0 * mat3x3_v[1][0] + mat3x3_v[2][1] * d1 * mat3x3_v[1][1] + mat3x3_v[2][2] * d2 * mat3x3_v[1][2];
        mat3x3_o[2][2] = mat3x3_v[2][0] * d0 * mat3x3_v[2][0] + mat3x3_v[2][1] * d1 * mat3x3_v[2][1] + mat3x3_v[2][2] * d2 * mat3x3_v[2][2];
    }

    private void svd_solve_ATA_ATb(
            float[] mat3x3_tri_ATA,
            Vec4f ATb,
            Vec4f x)
    {
        float[][] mat3x3_V = new float[3][3];
        mat3x3_V[0][0] = 1.f; mat3x3_V[0][1] = 0.f; mat3x3_V[0][2] = 0.f;
        mat3x3_V[1][0] = 0.f; mat3x3_V[1][1] = 1.f; mat3x3_V[1][2] = 0.f;
        mat3x3_V[2][0] = 0.f; mat3x3_V[2][1] = 0.f; mat3x3_V[2][2] = 1.f;

        Vec4f sigma = new Vec4f();
        svd_solve_sym(mat3x3_tri_ATA, sigma, mat3x3_V);

        // A = UEV^T; U = A / (E*V^T)
        float[][] mat3x3_Vinv = new float[3][3];
        svd_pseudoinverse(mat3x3_Vinv, sigma, mat3x3_V);
        svd_mul_matrix_vec(x, mat3x3_Vinv, ATb);
    }

    private void svd_vmul_sym(Vec4f result, float[] mat3x3_tri_A, Vec4f v) {
        Vec4f A_row_x = new Vec4f(mat3x3_tri_A[0], mat3x3_tri_A[1], mat3x3_tri_A[2], 0.f);

        result.x = v.dot(A_row_x);
        result.y = mat3x3_tri_A[1] * v.x + mat3x3_tri_A[3] * v.y + mat3x3_tri_A[4] * v.z;
        result.z = mat3x3_tri_A[2] * v.x + mat3x3_tri_A[4] * v.y + mat3x3_tri_A[5] * v.z;
    }

    private void qef_add_point(
            Vec4f n,
            Vec4f p)
    {
        mat3x3_tri_ATA[0] += n.x * n.x;
        mat3x3_tri_ATA[1] += n.x * n.y;
        mat3x3_tri_ATA[2] += n.x * n.z;
        mat3x3_tri_ATA[3] += n.y * n.y;
        mat3x3_tri_ATA[4] += n.y * n.z;
        mat3x3_tri_ATA[5] += n.z * n.z;

        float b = n.dot(p);
        ATb.x += n.x * b;
        ATb.y += n.y * b;
        ATb.z += n.z * b;

        masspoint.x += p.x;
        masspoint.y += p.y;
        masspoint.z += p.z;
        masspoint.w += 1.f;
    }

    private float qef_calc_error(float[] mat3x3_tri_A, Vec4f x, Vec4f b) {
        Vec4f tmp = new Vec4f();
        svd_vmul_sym(tmp, mat3x3_tri_A, x);
        tmp = b.sub(tmp);

        return tmp.dot(tmp);
    }

    public float qef_solve(Vec4f solvedPosition)
    {
        // prevent a div-by-zero exception
        masspoint = masspoint.div(Math.max(masspoint.w, 1.f));

        Vec4f A_mp = new Vec4f();
        svd_vmul_sym(A_mp, mat3x3_tri_ATA, masspoint);
        A_mp = ATb.sub(A_mp);

        svd_solve_ATA_ATb(mat3x3_tri_ATA, A_mp, solvedPosition);

        float error = qef_calc_error(mat3x3_tri_ATA, solvedPosition, ATb);
        solvedPosition.set(solvedPosition.add(masspoint));

        return error;
    }

    public Vec4f qef_solve() {
        Vec4f solvedPosition = new Vec4f();
        float error = qef_solve(solvedPosition);
        return solvedPosition;
    }

    public void qef_create_from_points(
            Vec4f[] positions,
            Vec4f[] normals,
            int count)
    {
        mat3x3_tri_ATA[0] = 0.f;
        mat3x3_tri_ATA[1] = 0.f;
        mat3x3_tri_ATA[2] = 0.f;
        mat3x3_tri_ATA[3] = 0.f;
        mat3x3_tri_ATA[4] = 0.f;
        mat3x3_tri_ATA[5] = 0.f;
        ATb = new Vec4f(0.f, 0.f, 0.f, 0.f);
        masspoint = new Vec4f(0.f, 0.f, 0.f, 0.f);

        for (int i= 0; i < count; ++i) {
            qef_add_point(normals[i], positions[i]);
        }

        masspoint = masspoint.div(masspoint.w);
    }
}
