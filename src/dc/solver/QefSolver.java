package dc.solver;

import core.math.Vec3f;
import core.math.Vec4f;

public class QefSolver{
    private float[] mat3x3_tri_ATA;
    private Vec4f atb;
    private Vec4f massPoint;
    private Vec4f x;
    private SvdSolver solver;

    public Vec4f getMasspoint() {
        return massPoint;
    }

    public void setSolvedPos(Vec4f pos) {
        x = pos;
    }

    public Vec4f getSolvedPos() {
        return x;
    }

    public QefSolver(SvdSolver solver) {
        mat3x3_tri_ATA = new float[6];
        atb = new Vec4f();
        massPoint = new Vec4f();
        this.solver = solver;
    }

    private void qef_add_point(Vec4f p, Vec4f n) {
        n.normalize();
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
        massPoint.x += p.x;
        massPoint.y += p.y;
        massPoint.z += p.z;
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
        return solver.solve(mat3x3_tri_ATA, atb, massPoint);
    }

    public static void main(String[] args) {
        Vec4f pointaccum = new Vec4f(0,0,0,0);
        float[] ATA = new float[6];
        Vec4f ATb = new Vec4f();
        QefSolver solver = new QefSolver(new GlslSvd());

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