package dc.impl;

import core.math.Vec3f;
import dc.solver.QEFData;

public class MdcVertex {
    public MdcVertex parent;
    public int index;
    public boolean collapsible;
    public QEFData qef;
    public Vec3f pos;
    public Vec3f normal;
    public int surface_index;
    public float error;
    public int euler;
    public int[] eis;
    public int in_cell;
    public boolean face_prop2;
    public boolean debugFlag = false;

    public MdcVertex()
    {
        parent = null;
        index = -1;
        collapsible = true;
        qef = null;
        normal = new Vec3f(0);
        surface_index = -1;
        eis = null;
    }
}
