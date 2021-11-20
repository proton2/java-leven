package dc.impl;

import dc.ChunkNode;
import dc.entities.CSGOperationInfo;

import java.util.Collection;

public interface ICSGOperations {
    void ApplyCSGOperations(MeshGenerationContext meshGen, Collection<CSGOperationInfo> opInfo, ChunkNode node, GPUDensityField field);
    boolean isReduceChunk();
}
