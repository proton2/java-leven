package dc.impl;

import core.math.Vec4i;
import dc.ChunkNode;
import dc.entities.CSGOperationInfo;

import java.util.Collection;
import java.util.Map;

public interface ICSGOperations {
    void ApplyCSGOperations(MeshGenerationContext meshGen, Collection<CSGOperationInfo> opInfo, ChunkNode node, GPUDensityField field);
    boolean isReduceChunk();
    void ApplyReduceOperations(ChunkNode node, GPUDensityField field, Map<Vec4i, GPUDensityField> densityFieldCache);
}
