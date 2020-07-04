package dc.utils;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import dc.ChunkNode;
import dc.OctreeNodeType;
import dc.PointerBasedOctreeNode;

import static dc.ChunkOctree.LEAF_SIZE_SCALE;
import static dc.ChunkOctree.VOXELS_PER_CHUNK;

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

    private int getOctreeSizeByChunkSize(int chunkSize){
        int chunkScaleSize = chunkSize / (VOXELS_PER_CHUNK * LEAF_SIZE_SCALE);
        return chunkScaleSize * LEAF_SIZE_SCALE;
    }

    public static Vec4f ApproximateZeroCrossingPosition(Vec3f p0, Vec3f p1, float[] densityField) {
        // approximate the zero crossing by finding the min value along the edge
        float minValue = 100000.f;
        float t = 0.f;
        float currentT = 0.f;
        int steps = 8;
        float increment = 1.f / (float)steps;
        while (currentT <= 1.f) {
            Vec3f p = mix(p0, p1, currentT);
            float density = Math.abs(Density.getNoise(p, densityField));
            if (density < minValue) {
                minValue = density;
                t = currentT;
            }
            currentT += increment;
        }
        return new Vec4f(mix(p0, p1, t), t);
    }

    public static Vec4f ApproximateLevenCrossingPosition(Vec3f p0, Vec3f p1, float[] densityField) {
        float FIND_EDGE_INFO_INCREMENT = 1.f / 16.f;
        int FIND_EDGE_INFO_STEPS = 16;
        float minValue = 100000.f;;
        float currentT = 0.f;
        float t = 0.f;
        for (int i = 0; i <= FIND_EDGE_INFO_STEPS; i++) {
            Vec3f p = mix(p0, p1, currentT);
            float d = Math.abs(Density.getNoise(p, densityField));
            if (d < minValue) {
                t = currentT;
                minValue = d;
            }
            currentT += FIND_EDGE_INFO_INCREMENT;
        }
        return new Vec4f(mix(p0, p1, t), t);
    }

    public static Vec3f mix(Vec3f p0, Vec3f p1, float t) {
        return p0.add((p1.sub(p0)).mul(t)); // p0 + ((p1 - p0) * t);
    }

    public static Vec4f mix(Vec4f p0, Vec4f p1, float t) {
        return p0.add((p1.sub(p0)).mul(t)); // p0 + ((p1 - p0) * t);
    }

    public static Vec4f CalculateSurfaceNormal(Vec4f p, float[] densityField) {
//	    float H = 0.001f;
//	    float dx = Density.Density_Func(p.add(new Vec3f(H, 0.f, 0.f)), densityField) - Density.Density_Func(p.sub(new Vec3f(H, 0.f, 0.f)), densityField);
//	    float dy = Density.Density_Func(p.add(new Vec3f(0.f, H, 0.f)), densityField) - Density.Density_Func(p.sub(new Vec3f(0.f, H, 0.f)), densityField);
//	    float dz = Density.Density_Func(p.add(new Vec3f(0.f, 0.f, H)), densityField) - Density.Density_Func(p.sub(new Vec3f(0.f, 0.f, H)), densityField);

        float H = 1f;
        Vec4f xOffcet = new Vec4f(H, 0.f, 0.f, 0.f);
        Vec4f yOffcet = new Vec4f(0.f, H, 0.f, 0.f);
        Vec4f zOffcet = new Vec4f(0.f, 0.f, H, 0.f);
        float dx = Density.getNoise(p.add(xOffcet), densityField) - Density.getNoise(p.sub(xOffcet), densityField);
        float dy = Density.getNoise(p.add(yOffcet), densityField) - Density.getNoise(p.sub(yOffcet), densityField);
        float dz = Density.getNoise(p.add(zOffcet), densityField) - Density.getNoise(p.sub(zOffcet), densityField);

        Vec4f v = new Vec4f(dx, dy, dz);
        v.normalize();
        return v;
    }

    public static boolean isOutFromBounds(Vec3f p, Vec3f min, int size) {
        return  p.X < min.X || p.X > (min.X + size) ||
                p.Y < min.Y || p.Y > (min.Y + size) ||
                p.Z < min.Z || p.Z > (min.Z + size);
    }
}