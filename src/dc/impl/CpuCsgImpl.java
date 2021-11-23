package dc.impl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.math.Vec4i;
import dc.ChunkNode;
import dc.VoxelOctree;
import dc.entities.CSGOperationInfo;
import dc.utils.SimplexNoise;
import dc.utils.VoxelHelperUtils;

import java.util.*;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Logger;
import java.util.stream.Collectors;

import static java.lang.Math.max;

public class CpuCsgImpl implements ICSGOperations{
    final public static Logger logger = Logger.getLogger(CpuCsgImpl.class.getName());
    private MeshGenerationContext meshGen;
    private final ExecutorService service;
    int availableProcessors;

    @Override
    public boolean isReduceChunk() {
        return reduceChunk;
    }

    final private boolean reduceChunk;

    public CpuCsgImpl(boolean reduce) {
        this.reduceChunk = reduce;
        availableProcessors = max(1, Runtime.getRuntime().availableProcessors() / 2);
        service = Executors.newFixedThreadPool(availableProcessors, new ThreadFactory() {
            private final AtomicInteger count = new AtomicInteger();
            @Override
            public Thread newThread(Runnable r) {
                Thread thread = new Thread(r);
                thread.setName("CpuCsgImpl " + count.getAndIncrement());
                return thread;
            }
        });
    }

    @Override
    public void ApplyCSGOperations(MeshGenerationContext meshGen, Collection<CSGOperationInfo> opInfo, ChunkNode node, GPUDensityField field){
        this.meshGen = meshGen;
        if (opInfo.isEmpty()) {
            return;
        }
        processCSG(meshGen, opInfo, node, field);
    }

    private void processCSG(MeshGenerationContext meshGen, Collection<CSGOperationInfo> opInfo, ChunkNode node, GPUDensityField field) {
        Vec4i fieldOffset = LeafScaleVec(node.min);
        int sampleScale = node.size / (meshGen.leafSizeScale * meshGen.getVoxelsPerChunk());
        int fieldBufferSize = meshGen.fieldSize * meshGen.fieldSize * meshGen.fieldSize;
        int[] d_updatedIndices = new int[fieldBufferSize];
        Vec3i[] d_updatedPoints = new Vec3i[fieldBufferSize];

        int numUpdatedPoints = CSG_HermiteIndicesMultiThread(fieldOffset, opInfo, sampleScale, field.materialsCpu,
                    d_updatedIndices, d_updatedPoints);
        if (numUpdatedPoints <= 0) {    // < 0 will be an error code
            return;
        }
        node.chunkIsEdited = true;
        node.parentIsDirty = true;

        Vec3i[] d_compactUpdatedPoints = new Vec3i[numUpdatedPoints];
        compactElements(d_updatedIndices, d_updatedPoints, d_compactUpdatedPoints);

        int[] d_generatedEdgeIndices = new int [numUpdatedPoints * 6];
        int numCompactEdgeIndices = FindUpdatedEdgesMultiThread(d_compactUpdatedPoints, d_generatedEdgeIndices);

        Set<Integer> d_invalidatedEdges = CompactIndexArray(d_generatedEdgeIndices, numCompactEdgeIndices);
        Map<Integer, Vec4f> createdEdges = FilterValidEdges(fieldOffset, opInfo, sampleScale, d_invalidatedEdges, field.materialsCpu);

        if (d_invalidatedEdges.size() > 0 && field.numEdges > 0) {
            field.hermiteEdgesMap.keySet().removeAll(d_invalidatedEdges);
            field.numEdges = field.hermiteEdgesMap.size();
        }

        if (createdEdges.size() > 0) {
            if (field.numEdges > 0) {
                field.hermiteEdgesMap.putAll(createdEdges);
                field.numEdges = field.hermiteEdgesMap.size();
            } else {
                field.numEdges = createdEdges.size();
                field.hermiteEdgesMap = createdEdges;
            }
        }
    }

    private int CSG_HermiteIndicesMultiThread(Vec4i worldspaceOffset, Collection<CSGOperationInfo> operations, int sampleScale, int[] field_materials,
                                   int[] updated_indices, Vec3i[] updated_positions){
        int bound = meshGen.getFieldSize();
        int threadBound = (bound * bound * bound) / availableProcessors;
        List<Callable<Integer>> tasks = new ArrayList<>();
        for (int i = 0; i < availableProcessors; i++) {
            int finalI = i;
            tasks.add(() -> {
                int from = finalI * threadBound;
                int to = from + threadBound;
                int size = 0;
                for (int it = from; it < to; it++) {
                    int x = it % bound;
                    int y = (it / bound) % bound;
                    int z = (it / bound / bound);
                    size = processMaterials(worldspaceOffset, operations, sampleScale, field_materials, updated_indices, updated_positions, size, z, y, x);
                }
                return size;
            });
        }
        return VoxelOctree.performIntCallableTask(tasks, service, logger);
    }

    private int processMaterials(Vec4i worldspaceOffset, Collection<CSGOperationInfo> operations, int sampleScale, int[] field_materials, int[] updated_indices, Vec3i[] updated_positions, int size, int z, int y, int x) {
        Vec3i local_pos = new Vec3i(x, y, z);
        int sx = sampleScale * x;
        int sy = sampleScale * y;
        int sz = sampleScale * z;

        int index = meshGen.getMaterialIndex(local_pos);
        int oldMaterial = field_materials[index];
        int material = field_materials[index];

        Vec3f world_pos = new Vec3f(worldspaceOffset.x + sx, worldspaceOffset.y + sy, worldspaceOffset.z + sz);
        material = BrushMaterial(world_pos, operations, material);

        int updated = material != oldMaterial ? 1 : 0;
        if(updated==1){
            field_materials[index] = material;
            size++;
        }
        updated_indices[index] = updated;
        updated_positions[index] = local_pos;
        return size;
    }

    private void compactElements(int[] edgeValid,  Vec3i[] edgeNormals,
                                 Vec3i[] compactNormals) {
        int cid = 0;
        for (int index = 0; index<edgeValid.length; index++) {
            if (edgeValid[index]==1) {
                compactNormals[cid++] = edgeNormals[index];
            }
        }
    }

    private int FindUpdatedEdgesMultiThread(Vec3i[] updatedHermiteIndices,
                                 int[] updatedHermiteEdgeIndices) {
        int bound = updatedHermiteIndices.length;
        final int threadBound = bound / availableProcessors;
        List<Callable<Integer>> tasks = new ArrayList<>();

        for (int i = 0; i < availableProcessors; i++) {
            int from = i * threadBound;
            int to = from + threadBound;
            boolean last = (i == availableProcessors - 1 && to <= bound - 1);
            tasks.add(() -> FindUpdatedEdges(from, last ? bound : to, updatedHermiteIndices, updatedHermiteEdgeIndices));
        }
        return VoxelOctree.performIntCallableTask(tasks, service, logger);
    }

    private int FindUpdatedEdges(int from, int to, Vec3i[] updatedHermiteIndices,
            int[] updatedHermiteEdgeIndices)
    {
        int size=0;
        for(int id=from; id<to; id++){
            int edgeIndex = id * 6;
            Vec3i pos = updatedHermiteIndices[id];
            int posIndex = (pos.x | (pos.y << meshGen.getIndexShift()) | (pos.z << (meshGen.getIndexShift() * 2))) << 2;

            updatedHermiteEdgeIndices[edgeIndex + 0] = posIndex | 0;
            updatedHermiteEdgeIndices[edgeIndex + 1] = posIndex | 1;
            updatedHermiteEdgeIndices[edgeIndex + 2] = posIndex | 2;
            size+=3;

            if (pos.x > 0) {
		        Vec3i xPos = pos.sub(new Vec3i(1, 0, 0));
		        int xPosIndex = (xPos.x | (xPos.y << meshGen.getIndexShift()) | (xPos.z << (meshGen.getIndexShift() * 2))) << 2;
                updatedHermiteEdgeIndices[edgeIndex + 3] = xPosIndex | 0;
                size++;
            }
            else {
                updatedHermiteEdgeIndices[edgeIndex + 3] = -1;
            }

            if (pos.y > 0) {
                Vec3i yPos = pos.sub(new Vec3i(0, 1, 0));
		        int yPosIndex = (yPos.x | (yPos.y << meshGen.getIndexShift()) | (yPos.z << (meshGen.getIndexShift() * 2))) << 2;
                updatedHermiteEdgeIndices[edgeIndex + 4] = yPosIndex | 1;
                size++;
            }
            else {
                updatedHermiteEdgeIndices[edgeIndex + 4] = -1;
            }

            if (pos.z > 0) {
                Vec3i zPos = pos.sub(new Vec3i(0, 0, 1));
		        int zPosIndex = (zPos.x | (zPos.y << meshGen.getIndexShift()) | (zPos.z << (meshGen.getIndexShift() * 2))) << 2;
                updatedHermiteEdgeIndices[edgeIndex + 5] = zPosIndex | 2;
                size++;
            }
            else {
                updatedHermiteEdgeIndices[edgeIndex + 5] = -1;
            }
        }
        return size;
    }

    private Set<Integer> CompactIndexArray(int[] edgeIndices, int numCompactEdgeIndices) {
        int[] compactIndices = new int[numCompactEdgeIndices];
        int current = 0;
        for (int edgeIndex : edgeIndices) {
            if (edgeIndex != -1) {
                compactIndices[current++] = edgeIndex;
            }
        }
        return Arrays.stream(compactIndices).boxed().collect(Collectors.toSet());
    }

    private Map<Integer, Vec4f> FilterValidEdges(Vec4i offset, Collection<CSGOperationInfo> operations, int sampleScale,
                                  Set<Integer> generatedHermiteEdgeIndices, int[] materials) {
        Map<Integer, Vec4f> normals = new HashMap<>();
        int FIELD_BUFFER_SIZE = meshGen.getFieldSize() * meshGen.getFieldSize() * meshGen.getFieldSize();
        for (int generatedEdgeIndex : generatedHermiteEdgeIndices) {
            int edgeNumber = generatedEdgeIndex & 3;
            int edgeIndex = generatedEdgeIndex >> 2;
            Vec3i position = new Vec4i(
                    (edgeIndex >> (meshGen.getIndexShift() * 0)) & meshGen.getIndexMask(),
                    (edgeIndex >> (meshGen.getIndexShift() * 1)) & meshGen.getIndexMask(),
                    (edgeIndex >> (meshGen.getIndexShift() * 2)) & meshGen.getIndexMask(),
                    0);

            int materialIndex0 = meshGen.getMaterialIndex(position);
            int materialIndex1 = meshGen.getMaterialIndex(position.add(VoxelOctree.EDGE_END_OFFSETS[edgeNumber]));

            // There should be no need to check these indices, the previous call to
            // RemoveInvalidIndices should have validated the generatedEdgeIndices array
            int material0 = materialIndex0 < FIELD_BUFFER_SIZE ? materials[materialIndex0] : meshGen.MATERIAL_AIR;
	        int material1 = materialIndex1 < FIELD_BUFFER_SIZE ? materials[materialIndex1] : meshGen.MATERIAL_AIR;

            int signChange = (material0 == meshGen.MATERIAL_AIR && material1 != meshGen.MATERIAL_AIR) ||
                    (material1 == meshGen.MATERIAL_AIR && material0 != meshGen.MATERIAL_AIR) ? 1 : 0;

            int edgeValid = signChange==1 && generatedEdgeIndex != -1 ? 1 : 0;
            if (edgeValid == 1) {
                normals.put(generatedEdgeIndex, calculateNorm(operations, offset, sampleScale, edgeNumber * 4, position));
            }
        }
        return normals;
    }

    private Vec4f calculateNorm(Collection<CSGOperationInfo> operations, Vec4i offset, int sampleScale, int edgeIndex, Vec3i local_pos){
        int e0 = VoxelOctree.edgevmap[edgeIndex][0];
        int e1 = VoxelOctree.edgevmap[edgeIndex][1];

        Vec3i world_pos = (local_pos.mul(sampleScale)).add(offset);
        Vec4f p0 = world_pos.add(VoxelOctree.CHILD_MIN_OFFSETS[e0]).toVec4f();
        Vec4f p1 = world_pos.add(VoxelOctree.CHILD_MIN_OFFSETS[e1].mul(sampleScale)).toVec4f();

        float t = BrushZeroCrossing(p0, p1, operations);
        Vec3f p = VoxelHelperUtils.mix(p0, p1, t).getVec3f();

        Vec3f n = BrushNormal(p, operations);
        return new Vec4f(n, t);
    }

    private Vec4i LeafScaleVec(Vec3i v) {
        Vec4i s = new Vec4i();
        s.x = v.x / meshGen.leafSizeScale;
        s.y = v.y / meshGen.leafSizeScale;
        s.z = v.z / meshGen.leafSizeScale;
        s.w = 0;
        return s;
    }

    float BrushDensity(Vec3f worldspaceOffset, CSGOperationInfo op) {
        float [] brushDensity = {Float.MIN_VALUE, Float.MAX_VALUE};
        //brushDensity[0] = Density_Cuboid(worldspaceOffset, op.getOrigin(), op.getDimensions(), op.getRotateY());
        brushDensity[0] = SimplexNoise.Density_Cuboid(worldspaceOffset, op.getOrigin().getVec3f(), op.getDimensions().getVec3f());
        brushDensity[1] = SimplexNoise.Density_Sphere(worldspaceOffset, op.getOrigin().getVec3f(), op.getDimensions().x);
        return brushDensity[op.getBrushShape().ordinal()];
    }

    // "concatenate" the brush operations to get the final density for the brush in isolation
    private float BrushZeroCrossing(Vec4f p0, Vec4f p1, Collection<CSGOperationInfo> operations) {
        float minDensity = Float.MAX_VALUE;
        float crossing = 0.f;
        for (float t = 0.f; t <= 1.f; t += (1.f/16.f)) {
		    Vec3f p = VoxelHelperUtils.mix(p0, p1, t).getVec3f();
            for (CSGOperationInfo op : operations) {
			    float d = Math.abs(BrushDensity(p, op));
                if (d < minDensity) {
                    crossing = t;
                    minDensity = d;
                }
            }
        }
        return crossing;
    }

    private int BrushMaterial(Vec3f world_pos, Collection<CSGOperationInfo> operations, int material) {
        int m = material;
        for (CSGOperationInfo op : operations) {
		    int[] operationMaterial = {op.getMaterial(), meshGen.MATERIAL_AIR};
		    float d = BrushDensity(world_pos, op);
            if (d <= 0.f) {
                m = operationMaterial[op.getType()];
            }
        }
        return m;
    }

    private Vec3f BrushNormal(Vec3f p, Collection<CSGOperationInfo> operations) {
        Vec3f normal = new Vec3f(0);
        for (CSGOperationInfo op : operations) {
		    //float d = BrushDensity(p, op);
            //if (d > 0.f) continue;

		    float h = 0.001f;
            Vec3f xOffcet = new Vec3f(h, 0.f, 0.f);
            Vec3f yOffcet = new Vec3f(0.f, h, 0.f);
            Vec3f zOffcet = new Vec3f(0.f, 0.f, h);
		    float dx0 = BrushDensity(p.add(xOffcet), op);
		    float dx1 = BrushDensity(p.sub(xOffcet), op);

		    float dy0 = BrushDensity(p.add(yOffcet), op);
		    float dy1 = BrushDensity(p.sub(yOffcet), op);

		    float dz0 = BrushDensity(p.add(zOffcet), op);
		    float dz1 = BrushDensity(p.sub(zOffcet), op);

            float flip = op.getType() == 0 ? 1.f : -1.f;
            normal = new Vec3f(dx0 - dx1, dy0 - dy1, dz0 - dz1).normalize().mul(flip);
        }
        return normal;
    }

//    private Vec4f[] FindEdgeIntersectionInfoMultiThread(Vec4i offset, Collection<CSGOperationInfo> operations, int sampleScale, int[] compactEdges) {
//        List<Callable<Boolean>> tasks = new ArrayList<>();
//        Vec4f[] normals = new Vec4f[compactEdges.length];
//        int bound = compactEdges.length;
//        final int threadBound = bound / availableProcessors;
//
//        for (int i = 0; i < availableProcessors; i++) {
//            int from = i * threadBound;
//            int to = from + threadBound;
//            boolean last = (i == availableProcessors - 1 && to <= bound - 1);
//            Callable<Boolean> task = () -> {
//                FindEdgeIntersectionInfo(from, last ? bound : to, offset, operations, sampleScale, compactEdges, normals);
//                return true;
//            };
//            tasks.add(task);
//        }
//        try {
//            service.invokeAll(tasks);
//        } catch (Exception e) {
//            logger.log(Level.SEVERE, e.toString());
//        }
//        return normals;
//    }
//
//    private void FindEdgeIntersectionInfo(int from, int to, Vec4i offset, Collection<CSGOperationInfo> operations, int sampleScale, int[] compactEdges, Vec4f[] normals) {
//        for (int index = from; index < to; index++) {
//            int globalEdgeIndex = compactEdges[index];
//            int edgeIndex = 4 * (globalEdgeIndex & 3);
//            int voxelIndex = globalEdgeIndex >> 2;
//
//            Vec4i local_pos = new Vec4i(
//                    (voxelIndex >> (meshGen.getIndexShift() * 0)) & meshGen.getIndexMask(),
//                    (voxelIndex >> (meshGen.getIndexShift() * 1)) & meshGen.getIndexMask(),
//                    (voxelIndex >> (meshGen.getIndexShift() * 2)) & meshGen.getIndexMask(),
//                    0);
//
//            normals[index] = calculateNorm(operations, offset, sampleScale, edgeIndex, local_pos);
//        }
//    }
//    private int CSG_HermiteIndices(Vec4i worldspaceOffset, Collection<CSGOperationInfo> operations, int sampleScale, int[] field_materials,
//                                   int[] updated_indices, Vec3i[] updated_positions) {
//        int size = 0;
//        for (int z = 0; z < meshGen.getFieldSize(); z++) {
//            for (int y = 0; y < meshGen.getFieldSize(); y++) {
//                for (int x = 0; x < meshGen.getFieldSize(); x++) {
//                    size = processMaterials(worldspaceOffset, operations, sampleScale, field_materials, updated_indices, updated_positions, size, z, y, x);
//                }
//            }
//        }
//        return size;
//    }

    //обрезать (дерево или куст), срезая мертвые или разросшиеся ветви или стебли, особенно для увеличения урожайности и роста.
//    private int PruneFieldEdges(int[] fieldEdgeIndices, Set<Integer> invalidatedEdgeIndices,
//                                int[] fieldEdgeValidity)
//    {
//        int count = 0;
//        for (int id = 0; id < fieldEdgeIndices.length; id++) {
//            if (invalidatedEdgeIndices.contains(fieldEdgeIndices[id])) {
//                fieldEdgeValidity[id] = 0;
//            } else {
//                fieldEdgeValidity[id] = 1;
//                count++;
//            }
//        }
//        return count;
//    }

//    private int PruneFieldEdges(Map<Integer, Vec4f> edgeIndices, Set<Integer> invalidatedEdgeIndices,
//                                int[] fieldEdgeValidity)
//    {
//        int count = 0;
//        int id = 0;
//        for (Integer edgeIndicate : edgeIndices.keySet()) {
//            if (invalidatedEdgeIndices.contains(edgeIndicate)) {
//                fieldEdgeValidity[id] = 0;
//            } else {
//                fieldEdgeValidity[id] = 1;
//                count++;
//            }
//            id++;
//        }
//        return count;
//    }

//    private void compactElements(int[] edgeValid, int[] edgeIndices, Vec4f[] edgeNormals,
//                                 int[] compactIndices, Vec4f[] compactNormals) {
//        int cid = 0;
//        for (int index = 0; index<edgeValid.length; index++) {
//            if (edgeValid[index]==1) {
//                compactIndices[cid] = edgeIndices[index];
//                compactNormals[cid] = edgeNormals[index];
//                cid++;
//            }
//        }
//    }
}