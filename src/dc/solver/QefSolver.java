package dc.solver;

import core.math.Vec3f;
import core.math.Vec4f;

public class QefSolver implements SvdSolver{
    private float[] ata;
    private Vec3f atb;
    private Vec4f massPoint;
    private float btb;
    private Vec4f x;
    //private int numPoints;

    public Vec4f getMasspoint() {
        return massPoint;
    }

    @Override
    public void setSolvedPos(Vec4f pos) {
        x = pos;
    }

    @Override
    public Vec4f getSolvedPos() {
        return x;
    }

    private GlslSvd glslSvdSolver = new GlslSvd();

    public QefSolver() {
        ata = new float[6];
        atb = new Vec3f();
        massPoint = new Vec4f();
        //x = new Vec4f();
    }

    public void qef_add_point(Vec4f p, Vec4f n) {
        n.normalize();
        ata[0] += n.x * n.x;
        ata[1] += n.x * n.y;
        ata[2] += n.x * n.z;
        ata[3] += n.y * n.y;
        ata[4] += n.y * n.z;
        ata[5] += n.z * n.z;
        float dot = n.x * p.x + n.y * p.y + n.z * p.z;
        atb.X += dot * n.x;
        atb.Y += dot * n.y;
        atb.Z += dot * n.z;
        btb += dot * dot;
        massPoint.x += p.x;
        massPoint.y += p.y;
        massPoint.z += p.z;
        ++massPoint.w;
        //++numPoints;
    }

    public void qef_create_from_points(Vec4f[] positions, Vec4f[] normals, int count) {
        for (int i= 0; i < count; ++i) {
            qef_add_point(positions[i], normals[i]);
        }
    }

    public float getError(Vec3f pos) {
        Vec3f atax = glslSvdSolver.vmulSym(ata, pos);
        return pos.dot(atax) - 2 * pos.dot(atb) + btb;
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
        if (massPoint.w == 0)
            throw new IllegalArgumentException("...");
        massPoint = massPoint.mul(1.0f / massPoint.w);

        Vec3f tmpv = glslSvdSolver.vmulSym(ata, massPoint.getVec3f());
        this.atb = this.atb.sub(tmpv);
        this.x = glslSvdSolver.svd_solve_ATA_ATb(this.ata, this.atb);
        //float result = glslSvdSolver.qef_calc_error(this.ata.convTo2dFloat(), x, this.atb);
        float result = getError(x.getVec3f());
        this.x = this.x.add(massPoint);

        return x;
    }
}