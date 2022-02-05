package dc.impl;

import core.math.Vec3i;
import core.math.Vec4f;

import java.util.Map;
import java.util.concurrent.ConcurrentSkipListMap;

public class CPUDensityField {
    public Vec3i min;
    public int size;
    public int lastCSGOperation = 0;
    public int[] materialsCpu;
    public Map<Integer, Vec4f> hermiteEdgesMap = new ConcurrentSkipListMap<>();
}
