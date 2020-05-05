package dc.svd;

import core.math.Vec3f;

public class QefSolver {
    private QefData data;
    private SMat3 ata;
    private Vec3f atb, massPoint, x;
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

    public void add(Vec3f p, Vec3f n) {
        hasSolution = false;
        n.normalize();
        data.ata_00 += n.X * n.X;
        data.ata_01 += n.X * n.Y;
        data.ata_02 += n.X * n.Z;
        data.ata_11 += n.Y * n.Y;
        data.ata_12 += n.Y * n.Z;
        data.ata_22 += n.Z * n.Z;
        float dot = n.X * p.X + n.Y * p.Y + n.Z * p.Z;
        data.atb_x += dot * n.X;
        data.atb_y += dot * n.Y;
        data.atb_z += dot * n.Z;
        data.btb += dot * dot;
        data.massPoint_x += p.X;
        data.massPoint_y += p.Y;
        data.massPoint_z += p.Z;
        ++data.numPoints;
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

    public float solve(Vec3f outx, float svd_tol, int svd_sweeps, float pinv_tol) {
        //return solveSimple(outx);
        return solveNew(outx, svd_tol, svd_sweeps, pinv_tol);
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

    public float solveNew(Vec3f outx, float svd_tol, int svd_sweeps, float pinv_tol) {
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

    private void setAta() {
        ata.setSymmetric(data.ata_00, data.ata_01, data.ata_02, data.ata_11, data.ata_12, data.ata_22);
    }

    private void setAtb() {
        atb.setX(data.atb_x);
        atb.setY(data.atb_y);
        atb.setZ(data.atb_z);
    }
}
