package dc.svd;

import core.math.Vec4f;

public interface SvdSolver {
    Vec4f solve();
    void qef_create_from_points(Vec4f[] positions, Vec4f[] normals, int count);
    //void qef_add_point(Vec4f p, Vec4f n);
    Vec4f getMasspoint();
}
