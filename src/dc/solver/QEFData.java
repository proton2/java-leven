package dc.solver;

import core.math.Vec3f;
import core.math.Vec4f;

public class QEFData {
    public float[] mat3x3_tri_ATA;
    public Vec4f atb;
    public Vec4f massPoint;
    public Vec4f x;
    private SvdSolver solver;
    private float btb;

    public Vec4f getMasspoint() {
        return massPoint;
    }

    public void setSolvedPos(Vec4f pos) {
        x = pos;
    }

    public Vec4f getSolvedPos() {
        return x;
    }

    public QEFData(SvdSolver solver) {
        mat3x3_tri_ATA = new float[6];
        atb = new Vec4f();
        massPoint = new Vec4f();
        this.solver = solver;
    }

    public void add(QEFData rhs){
        mat3x3_tri_ATA[0] += rhs.mat3x3_tri_ATA[0];
        mat3x3_tri_ATA[1] += rhs.mat3x3_tri_ATA[1];
        mat3x3_tri_ATA[2] += rhs.mat3x3_tri_ATA[2];
        mat3x3_tri_ATA[3] += rhs.mat3x3_tri_ATA[3];
        mat3x3_tri_ATA[4] += rhs.mat3x3_tri_ATA[4];
        mat3x3_tri_ATA[5] += rhs.mat3x3_tri_ATA[5];
        atb.x += rhs.atb.x;
        atb.y += rhs.atb.y;
        atb.z += rhs.atb.z;
        btb += rhs.btb;
        massPoint.x += rhs.massPoint.x;
        massPoint.y += rhs.massPoint.y;
        massPoint.z += rhs.massPoint.z;
        massPoint.w += rhs.massPoint.w;
    }

    private void qef_add_point(Vec4f p, Vec4f n) {
        mat3x3_tri_ATA[0] += n.x * n.x;
        mat3x3_tri_ATA[1] += n.x * n.y;
        mat3x3_tri_ATA[2] += n.x * n.z;
        mat3x3_tri_ATA[3] += n.y * n.y;
        mat3x3_tri_ATA[4] += n.y * n.z;
        mat3x3_tri_ATA[5] += n.z * n.z;
        float dot = n.x * p.x + n.y * p.y + n.z * p.z;
        atb.x += dot * n.x;
        atb.y += dot * n.y;
        atb.z += dot * n.z;
        btb += dot * dot;
        massPoint.x += p.x;
        massPoint.y += p.y;
        massPoint.z += p.z;
        ++massPoint.w;
    }

    public void qef_add_point(Vec3f p, Vec3f n) {
        mat3x3_tri_ATA[0] += n.X * n.X;
        mat3x3_tri_ATA[1] += n.X * n.Y;
        mat3x3_tri_ATA[2] += n.X * n.Z;
        mat3x3_tri_ATA[3] += n.Y * n.Y;
        mat3x3_tri_ATA[4] += n.Y * n.Z;
        mat3x3_tri_ATA[5] += n.Z * n.Z;
        float dot = n.X * p.X + n.Y * p.Y + n.Z * p.Z;
        atb.x += dot * n.X;
        atb.y += dot * n.Y;
        atb.z += dot * n.Z;
        btb += dot * dot;
        massPoint.x += p.X;
        massPoint.y += p.Y;
        massPoint.z += p.Z;
        ++massPoint.w;
    }

    public void qef_create_from_points(Vec4f[] positions, Vec4f[] normals, int count) {
        for (int i= 0; i < count; ++i) {
            qef_add_point(positions[i], normals[i]);
        }
        massPoint = massPoint.div(massPoint.w);
    }

    public Vec4f solveSimple() {
        if (massPoint.w == 0) {
            throw new IllegalArgumentException("...");
        }
        massPoint = massPoint.mul(1.0f / massPoint.w);
        x = x.add(massPoint);
        return x;
    }

    public Vec4f solve() {
        x = solver.solve(mat3x3_tri_ATA, atb, massPoint);
        return x;
    }

    public float getError() {
        return getError(x);
        //return solver.qef_calc_error(mat3x3_tri_ATA, x, atb);
    }

    private float getError(Vec4f pos) {
        Vec4f atax = pos.vmul(mat3x3_tri_ATA);
        double result = pos.dot(atax) - 2 * pos.dot(atb) + btb;
        return (float) Math.max(result, 0);
    }

    public static void main(String[] args) {
        Vec4f pointaccum = new Vec4f(0,0,0,0);
        QEFData solver = new QEFData(new GlslSvd());

        final int count = 5;
        Vec4f[] normals = {
                new Vec4f( 1.0f,1.0f,0.0f).normalize(),
                new Vec4f( 1.0f,1.0f,0.0f).normalize(),
                new Vec4f(-1.0f,1.0f,0.0f).normalize(),
                new Vec4f(-1.0f,2.0f,1.0f).normalize(),
                new Vec4f(-1.0f,1.0f,0.0f).normalize(),
        };
        Vec4f[] points = {
                new Vec4f(  1.0f, 0.0f, 0.3f),
                new Vec4f(  0.9f, 0.1f, -0.5f),
                new Vec4f( -0.8f, 0.2f, 0.6f),
                new Vec4f( -1.0f, 0.0f, 0.01f),
                new Vec4f( -1.1f, -0.1f, -0.5f),
        };

        solver.qef_create_from_points(points, normals, count);

        Vec3f com = pointaccum.div(pointaccum.w).getVec3f();

        Vec4f x = solver.solve();

        System.out.println(String.format("masspoint = (%.5f %.5f %.5f)\n", com.X, com.Y, com.Z));
        System.out.println(String.format("point = (%.5f %.5f %.5f %.5f)\n", x.x, x.y, x.z, x.w));
    }
}