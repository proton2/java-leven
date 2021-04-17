package dc;

import core.math.Vec3i;
import dc.entities.CSGOperationInfo;
import dc.entities.MeshBuffer;

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

    float QEF_ERROR = 1e-6f;
    int QEF_SWEEPS = 4;

    boolean createLeafVoxelNodes(int chunkSize, Vec3i chunkMin, List<OctreeNode> seamNodes, MeshBuffer meshBuffer);
    void processNodesToMesh(List<OctreeNode> seamNodes, Vec3i min, int rootNodeSize, boolean isSeam, MeshBuffer meshBuffer);
    void applyCSGOperations(List<CSGOperationInfo> operations, ChunkNode clipmapNode);
}
