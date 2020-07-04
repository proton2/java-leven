package dc.svd;

import core.math.Vec3f;
import core.math.Vec4f;

public class QefSolver implements SvdSolver{
    private QefData data;
    private SMat3 ata;
    private Vec3f atb;
    private Vec3f massPoint;

    public Vec4f getMasspoint() {
        return new Vec4f(massPoint);
    }

    public void setX(Vec3f x) {
        this.x = x;
    }

    public Vec3f getX() {
        return x;
    }

    private Vec3f x;

    public boolean isHasSolution() {
        return hasSolution;
    }

    public void setHasSolution(boolean hasSolution) {
        this.hasSolution = hasSolution;
    }

    private boolean hasSolution;
    private GlslSvd glslSvdSolver = new GlslSvd();

    public QefSolver() {
        data = new QefData();
        ata = new SMat3();
        atb = new Vec3f();
        massPoint = new Vec3f();
        x = new Vec3f();
        hasSolution = false;
    }

    public Vec3f getMassPoint() {
        return massPoint;
    }

    public void qef_add_point(Vec4f p, Vec4f n) {
        hasSolution = false;
        n.normalize();
        data.ata_00 += n.x * n.x;
        data.ata_01 += n.x * n.y;
        data.ata_02 += n.x * n.z;
        data.ata_11 += n.y * n.y;
        data.ata_12 += n.y * n.z;
        data.ata_22 += n.z * n.z;
        float dot = n.x * p.x + n.y * p.y + n.z * p.z;
        data.atb_x += dot * n.x;
        data.atb_y += dot * n.y;
        data.atb_z += dot * n.z;
        data.btb += dot * dot;
        data.massPoint_x += p.x;
        data.massPoint_y += p.y;
        data.massPoint_z += p.z;
        ++data.numPoints;
    }

    public void qef_create_from_points(Vec4f[] positions, Vec4f[] normals, int count) {
        for (int i= 0; i < count; ++i) {
            qef_add_point(positions[i], normals[i]);
        }
    }

    public void add(QefData rhs) {
        hasSolution = false;
        data.add(rhs);
    }

    public QefData getData() {
        return data;
    }

    public float getError() {
        if (!hasSolution) {
            throw new IllegalArgumentException("Qef Solver does not have a solution!");
        }
        return getError(x);
    }

    public float getError(Vec3f pos) {
        if (!hasSolution) {
            setAta();
            setAtb();
        }
        Vec3f atax = this.ata.Vmul(pos);
        return pos.dot(atax) - 2 * pos.dot(atb) + data.btb;
    }

    public void reset() {
        hasSolution = false;
        data.clear();
    }

    public float solveSimple(Vec3f outx) {
        if (data.numPoints == 0) {
            throw new IllegalArgumentException("...");
        }
        this.massPoint.set(this.data.massPoint_x, this.data.massPoint_y, this.data.massPoint_z);
        this.massPoint = this.massPoint.mul(1.0f / this.data.numPoints);
        setAta();
        setAtb();
        this.x.set(0);
        this.x = this.x.add(massPoint);
        setAtb();
        outx.set(x);
        hasSolution = true;
        return 0;
    }

    public float solveNew(Vec3f outx) {
        if (data.numPoints == 0)
            throw new IllegalArgumentException("...");
        this.massPoint.set(this.data.massPoint_x, this.data.massPoint_y, this.data.massPoint_z);
        this.massPoint = this.massPoint.mul(1.0f / this.data.numPoints);
        setAta();
        setAtb();
        Vec3f tmpv = ata.Vmul(this.massPoint);
        this.atb = this.atb.sub(tmpv);
        //glslSvdSolver.svd_solve_ATA_ATb(this.ata.convTo2dFloat(), this.atb, this.x);
        this.x = glslSvdSolver.solve(this.ata, this.atb);
        //float result = glslSvdSolver.qef_calc_error(this.ata.convTo2dFloat(), x, this.atb);
        float result = getError(x);
        this.x = this.x.add(massPoint);
        setAtb();
        outx.set(x);
        hasSolution = true;
        return result;
    }

    public Vec4f solve() {
        Vec3f outx = new Vec3f();
        float error = solveNew(outx);
        return new Vec4f(outx);
    }

    private void setAta() {
        ata.setSymmetric(data.ata_00, data.ata_01, data.ata_02, data.ata_11, data.ata_12, data.ata_22);
    }

    private void setAtb() {
        atb.setX(data.atb_x);
        atb.setY(data.atb_y);
        atb.setZ(data.atb_z);
    }
}
