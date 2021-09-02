package dc.utils;

import core.math.Vec2f;
import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import dc.ChunkNode;
import dc.OctreeNode;
import dc.OctreeNodeType;
import dc.impl.MeshGenerationContext;

import java.io.*;
import java.nio.file.Paths;
import java.util.Scanner;

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

    public static StringBuilder createMainBuildOptions(MeshGenerationContext meshGenCtx) {
        StringBuilder buildOptions = new StringBuilder();
        buildOptions.append("-cl-denorms-are-zero ");
        buildOptions.append("-cl-finite-math-only ");
        buildOptions.append("-cl-no-signed-zeros ");
        buildOptions.append("-cl-fast-relaxed-math ");
        buildOptions.append("-g "); // option nesessary for debug
        buildOptions.append("-Werror ");

        buildOptions.append("-DFIELD_DIM=").append(meshGenCtx.getFieldSize()).append(" ");
        buildOptions.append("-DFIND_EDGE_INFO_STEPS=" + 16 + " ");
        buildOptions.append("-DFIND_EDGE_INFO_INCREMENT=" + (1.f/16.f) + " ");
        buildOptions.append("-DVOXELS_PER_CHUNK=").append(meshGenCtx.getVoxelsPerChunk()).append(" ");
        buildOptions.append("-DMAX_OCTREE_DEPTH=").append(meshGenCtx.MAX_OCTREE_DEPTH).append(" ");
        buildOptions.append("-DVOXEL_INDEX_SHIFT=").append(meshGenCtx.getIndexShift()).append(" ");
        buildOptions.append("-DVOXEL_INDEX_MASK=").append(meshGenCtx.getIndexMask()).append(" ");
        buildOptions.append("-DHERMITE_INDEX_SIZE=").append(meshGenCtx.getHermiteIndexSize()).append(" ");
        buildOptions.append("-DLEAF_SIZE_SCALE=").append(meshGenCtx.leafSizeScale).append(" ");
        buildOptions.append("-DCUCKOO_STASH_HASH_INDEX=").append(meshGenCtx.CUCKOO_STASH_HASH_INDEX).append(" ");
        buildOptions.append("-DCUCKOO_EMPTY_VALUE=").append(meshGenCtx.CUCKOO_EMPTY_VALUE).append(" ");
        buildOptions.append("-DCUCKOO_STASH_SIZE=").append(meshGenCtx.CUCKOO_STASH_SIZE).append(" ");
        buildOptions.append("-DCUCKOO_MAX_ITERATIONS=").append(meshGenCtx.CUCKOO_MAX_ITERATIONS).append(" ");
        buildOptions.append("-DMATERIAL_AIR=").append(201).append(" ");
        buildOptions.append("-DMATERIAL_NONE=").append(200).append(" ");
        File file = new File(Paths.get("res/opencl/scan.cl").toUri());
        if(file.exists()){
            buildOptions.append("-I ").append(file.getParent());
        }
        return buildOptions;
    }

    public static StringBuilder getCuckooBuildOptions(MeshGenerationContext meshGen) {
        StringBuilder buildOptions = new StringBuilder();
        buildOptions.append("-DCUCKOO_EMPTY_VALUE=").append(meshGen.CUCKOO_EMPTY_VALUE).append(" ");
        buildOptions.append("-DCUCKOO_STASH_HASH_INDEX=").append(meshGen.CUCKOO_STASH_HASH_INDEX).append(" ");
        buildOptions.append("-DCUCKOO_HASH_FN_COUNT=").append(meshGen.CUCKOO_HASH_FN_COUNT).append(" ");
        buildOptions.append("-DCUCKOO_STASH_SIZE=").append(meshGen.CUCKOO_STASH_SIZE).append(" ");
        buildOptions.append("-DCUCKOO_MAX_ITERATIONS=").append(meshGen.CUCKOO_MAX_ITERATIONS).append(" ");
        File file = new File(Paths.get("res/opencl/cuckoo.cl").toUri());
        if(file.exists()){
            buildOptions.append("-I ").append(file.getParent());
        }
        return buildOptions;
    }

    public static Vec3f ColourForMinLeafSize(int minLeafSize) {
        switch (minLeafSize) {
            case 1:
                return new Vec3f(0.3f, 0.1f, 0.f);
            case 2:
                return new Vec3f(0, 0.f, 0.5f);
            case 4:
                return new Vec3f(0, 0.5f, 0.5f);
            case 8:
                return new Vec3f(0.5f, 0.f, 0.5f);
            case 16:
                return new Vec3f(0.0f, 0.5f, 0.f);
            default:
                return new Vec3f(0.5f, 0.0f, 0.f);
        }
    }

    public static boolean intersectRayAab(Vec3f origin, Vec3f dir, Vec3f min, Vec3f max, Vec2f result) {
        return intersectRayAab(origin.X, origin.Y, origin.Z, dir.X, dir.Y, dir.Z, min.X, min.Y, min.Z, max.X, max.Y, max.Z, result);
    }

    public static boolean intersectRayAab(float originX, float originY, float originZ, float dirX, float dirY, float dirZ,
                                          float minX, float minY, float minZ, float maxX, float maxY, float maxZ, Vec2f result) {
        float invDirX = 1.0f / dirX, invDirY = 1.0f / dirY, invDirZ = 1.0f / dirZ;
        float tNear, tFar, tymin, tymax, tzmin, tzmax;
        if (invDirX >= 0.0f) {
            tNear = (minX - originX) * invDirX;
            tFar = (maxX - originX) * invDirX;
        } else {
            tNear = (maxX - originX) * invDirX;
            tFar = (minX - originX) * invDirX;
        }
        if (invDirY >= 0.0f) {
            tymin = (minY - originY) * invDirY;
            tymax = (maxY - originY) * invDirY;
        } else {
            tymin = (maxY - originY) * invDirY;
            tymax = (minY - originY) * invDirY;
        }
        if (tNear > tymax || tymin > tFar)
            return false;
        if (invDirZ >= 0.0f) {
            tzmin = (minZ - originZ) * invDirZ;
            tzmax = (maxZ - originZ) * invDirZ;
        } else {
            tzmin = (maxZ - originZ) * invDirZ;
            tzmax = (minZ - originZ) * invDirZ;
        }
        if (tNear > tzmax || tzmin > tFar)
            return false;
        tNear = tymin > tNear || Float.isNaN(tNear) ? tymin : tNear;
        tFar = tymax < tFar || Float.isNaN(tFar) ? tymax : tFar;
        tNear = tzmin > tNear ? tzmin : tNear;
        tFar = tzmax < tFar ? tzmax : tFar;
        if (tNear < tFar && tFar >= 0.0f) {
            result.X = tNear;
            result.Y = tFar;
            return true;
        }
        return false;
    }

    public static void saveToFile(int[] arr, String fileName){
        try {
            Writer wr = new FileWriter(fileName);
            for (int j : arr) {
                wr.write(j + "\t" + "\n");
            }
            wr.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void saveToFile(Vec4f[] arr, String fileName){
        try {
            Writer wr = new FileWriter(fileName);
            for (Vec4f j : arr) {
                wr.write(j.x + "\t" + "");
                wr.write(j.y + "\t" + "");
                wr.write(j.z + "\t" + "");
                wr.write(j.w + "\t" + "\n");
            }
            wr.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void readFromFile(int[] arr, String fileName){
        try {
            Scanner scanner = new Scanner(new File(fileName));
            int i = 0;
            while(scanner.hasNextInt()){
                arr[i++] = scanner.nextInt();
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    public static boolean isSorted(int[] a) {
        // Our strategy will be to compare every element to its successor.
        // The array is considered unsorted
        // if a successor has a greater value than its predecessor.
        // If we reach the end of the loop without finding that the array is unsorted,
        // then it must be sorted instead.

        // Note that we are always comparing an element to its successor.
        // Because of this, we can end the loop after comparing
        // the second-last element to the last one.
        // This means the loop iterator will end as an index of the second-last
        // element of the array instead of the last one.
        for (int i = 0; i < a.length - 1; i++) {
            if (a[i] > a[i + 1]) {
                return false; // It is proven that the array is not sorted.
            }
        }
        return true; // If this part has been reached, the array must be sorted.
    }
}