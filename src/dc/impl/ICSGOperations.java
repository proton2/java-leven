package dc.impl;

import core.math.Vec4i;
import dc.ChunkNode;
import dc.entities.CSGOperationInfo;

import java.util.Collection;
import java.util.Map;

public interface ICSGOperations {
    boolean ApplyCSGOperations(MeshGenerationContext meshGen, Collection<CSGOperationInfo> opInfo, ChunkNode node, CPUDensityField field);
    void ApplyReduceOperations(ChunkNode node, CPUDensityField field, Map<Vec4i, CPUDensityField> densityFieldCache);
}
