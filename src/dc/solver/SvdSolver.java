package dc.solver;

import core.math.Vec4f;

public interface SvdSolver {
    Vec4f solve(float[] ATA, Vec4f ATb, Vec4f pointaccum);
    float solve(float[] ATA, Vec4f ATb, Vec4f pointaccum, Vec4f pos);
}
