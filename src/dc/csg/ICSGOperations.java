package dc.csg;

import core.math.Vec4i;
import dc.ChunkNode;
import dc.entities.CSGOperationInfo;
import dc.impl.CPUDensityField;
import dc.impl.MeshGenerationContext;

import java.util.Map;

public interface ICSGOperations {
    boolean ApplyCSGOperations(MeshGenerationContext meshGen, CSGOperationInfo opInfo, ChunkNode node, CPUDensityField field);
    void ApplyReduceOperations(ChunkNode node, CPUDensityField field, Map<Vec4i, CPUDensityField> densityFieldCache);
}
