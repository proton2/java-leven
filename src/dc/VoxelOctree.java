package dc;

import core.math.Vec3i;
import dc.entities.CSGOperationInfo;
import dc.entities.MeshBuffer;
import dc.impl.CPUDensityField;
import dc.impl.ICSGOperations;
import dc.utils.Aabb;

import java.util.Collection;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Future;
import java.util.logging.Level;
import java.util.logging.Logger;

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

    Vec3i[] EDGE_END_OFFSETS = {
            new Vec3i(1,0,0),
            new Vec3i(0,1,0),
            new Vec3i(0,0,1)
    };

    float QEF_ERROR = 1e-6f;
    int QEF_SWEEPS = 4;

    boolean createLeafVoxelNodes(ChunkNode node, List<OctreeNode> seamNodes, MeshBuffer meshBuffer);
    void processNodesToMesh(List<OctreeNode> seamNodes, Vec3i min, int rootNodeSize, boolean isSeam, MeshBuffer meshBuffer);
    CPUDensityField computeApplyCSGOperations(Collection<CSGOperationInfo> operations, ChunkNode node);
    void computeStoreCSGOperation(CSGOperationInfo opInfo, Aabb aabb);
    void computeClearCSGOperations();
    void computeFreeChunkOctree(Vec3i min, int clipmapNodeSize);
    ICSGOperations getCsgOperationsProcessor();

    static int performIntCallableTask(List<Callable<Integer>> tasks, ExecutorService service, Logger logger){
        int size = 0;
        try {
            List<Future<Integer>> futures = service.invokeAll(tasks);
            for (Future<Integer> future : futures) {
                size += future.get();
            }
        } catch (Exception e) {
            logger.log(Level.SEVERE, e.toString());
        }
        return size;
    }

    static void performBoolCallableTask(List<Callable<Boolean>> tasks, ExecutorService service, Logger logger){
        try {
            service.invokeAll(tasks);
        } catch (Exception e) {
            logger.log(Level.SEVERE, e.toString());
        }
    }
}
