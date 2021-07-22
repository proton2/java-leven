package dc.impl;

import core.math.Vec3i;
import dc.entities.CSGOperationInfo;

import java.util.Collection;

public interface ICSGOperations {
    void ApplyCSGOperations(MeshGenerationContext meshGen, Collection<CSGOperationInfo> opInfo, Vec3i clipmapNodeMin,
                            int clipmapNodeSize, GPUDensityField field);
}
