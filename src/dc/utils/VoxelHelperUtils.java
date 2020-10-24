package dc.utils;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import dc.ChunkNode;
import dc.OctreeNodeType;
import dc.PointerBasedOctreeNode;

public class VoxelHelperUtils {
    static public int countLeafNodes(PointerBasedOctreeNode node){
        if (node.Type== OctreeNodeType.Node_Leaf){
            return 1;
        }
        int result = 0;
        for (PointerBasedOctreeNode n : node.children){
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

    public static Vec3f mix(Vec3f p0, Vec3f p1, float t) {
        return p0.add((p1.sub(p0)).mul(t)); // p0 + ((p1 - p0) * t);
    }

    public static Vec4f mix(Vec4f p0, Vec4f p1, float t) {
        return p0.add((p1.sub(p0)).mul(t)); // p0 + ((p1 - p0) * t);
    }

    public static boolean isOutFromBounds(Vec3f p, Vec3f min, int size) {
        return  p.X < min.X || p.X > (min.X + size) ||
                p.Y < min.Y || p.Y > (min.Y + size) ||
                p.Z < min.Z || p.Z > (min.Z + size);
    }

    public static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }

    public static int log2(int N) {
        return (int) (Math.log(N) / Math.log(2));
    }
}