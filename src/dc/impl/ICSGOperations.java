package dc.impl;

import core.math.Vec3i;
import dc.entities.CSGOperationInfo;

import java.util.List;

public interface ICSGOperations {
    void ApplyCSGOperations(MeshGenerationContext meshGen, List<CSGOperationInfo> opInfo, Vec3i clipmapNodeMin,
                            int clipmapNodeSize, GPUDensityField field);
}
