package dc.utils;

import core.math.Vec3f;
import core.math.Vec3i;
import dc.ChunkNode;
import dc.OctreeNode;
import dc.OctreeNodeType;

import static dc.ChunkOctree.LEAF_SIZE_SCALE;
import static dc.ChunkOctree.VOXELS_PER_CHUNK;

public class VoxelHelperUtils {
    static public int countLeafNodes(OctreeNode node){
        if (node.Type== OctreeNodeType.Node_Leaf){
            return 1;
        }
        int result = 0;
        for (OctreeNode n : node.children){
            if(n!=null) result += countLeafNodes(n);
        }
        return result;
    }

    static private float max(float x, float y, float z){
        return Math.max(Math.max(x, y), z);
    }

    public static float DistanceToNode(ChunkNode node, Vec3f cameraPos) {
        // from http://stackoverflow.com/questions/5254838/calculating-distance-between-a-point-and-a-rectangular-box-nearest-point
        Vec3i min = node.min;
        Vec3i max = node.min.add(node.size);

        float dx = max(min.x - cameraPos.X, 0f, cameraPos.X - max.x);
        float dy = max(min.y - cameraPos.Y, 0f, cameraPos.Y - max.y);
        float dz = max(min.z - cameraPos.Z, 0f, cameraPos.Z - max.z);
        return (float) Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    float Distance3DToNode(ChunkNode node, Vec3f cameraPos) {
        Vec3f center = node.min.add(new Vec3i(node.size/2, node.size/2, node.size/2)).toVec3f();
        Vec3f p = center.sub(cameraPos);
        return (float) Math.sqrt(p.X * p.X + p.Y * p.Y + p.Z * p.Z);
    }

    private int getOctreeSizeByChunkSize(int chunkSize){
        int chunkScaleSize = chunkSize / (VOXELS_PER_CHUNK * LEAF_SIZE_SCALE);
        return chunkScaleSize * LEAF_SIZE_SCALE;
    }
}