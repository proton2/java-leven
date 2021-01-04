package dc.solver;

import core.math.Vec4f;

public interface SvdSolver {
    Vec4f solve(float[] ATA, Vec4f ATb, Vec4f pointaccum);
    float qef_calc_error(float[] mat3x3_tri_A, Vec4f x, Vec4f atb);
}
