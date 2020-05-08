package dc;

import core.math.Vec3i;
import core.math.Vec4f;
import dc.entities.MeshBuffer;
import dc.entities.VoxelTypes;

import java.util.EnumMap;
import java.util.List;

public interface VoxelOctree {
    int[][] edgevmap = {
            {0,4},{1,5},{2,6},{3,7},	// x-axis
            {0,2},{1,3},{4,6},{5,7},	// y-axis
            {0,1},{2,3},{4,5},{6,7}		// z-axis
    };
    Vec3i[] CHILD_MIN_OFFSETS = {
            // needs to match the vertMap from Dual Contouring impl
            new Vec3i( 0, 0, 0 ),
            new Vec3i( 0, 0, 1 ),
            new Vec3i( 0, 1, 0 ),
            new Vec3i( 0, 1, 1 ),
            new Vec3i( 1, 0, 0 ),
            new Vec3i( 1, 0, 1 ),
            new Vec3i( 1, 1, 0 ),
            new Vec3i( 1, 1, 1 ),
    };
    int MATERIAL_AIR = 0;
    int MATERIAL_SOLID = 1;

    float QEF_ERROR = 1e-6f;
    int QEF_SWEEPS = 4;

    EnumMap<VoxelTypes, List<OctreeNode>> createLeafVoxelNodes(ChunkNode chunk, Vec4f[] frustumPlanes, int voxelsPerChunk,
                                                               int clipmapLeafSize, int leafSizeScale);
    MeshBuffer GenerateMeshFromOctree(OctreeNode node, boolean isSeam);

    OctreeNode constructTreeUpwards(List<OctreeNode> inputNodes, Vec3i rootMin, int rootNodeSize);
}
