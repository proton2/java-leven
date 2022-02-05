package dc.impl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.math.Vec4i;
import core.utils.BufferUtil;
import dc.*;
import dc.entities.CSGOperationInfo;
import dc.entities.MeshBuffer;
import dc.entities.MeshVertex;
import dc.solver.LevenQefSolver;
import dc.solver.QEFData;
import dc.utils.Aabb;
import dc.utils.VoxelHelperUtils;

import java.util.*;
import java.util.concurrent.Callable;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Logger;

import static dc.utils.SimplexNoise.getNoise;
import static java.lang.Math.max;

public class LevenLinearCPUOctreeImpl extends AbstractDualContouring implements VoxelOctree {
    final public static Logger logger = Logger.getLogger(LevenLinearCPUOctreeImpl.class.getName());
    private final ExecutorService service;
    private final int availableProcessors;
    private final boolean enableQefClamping = true;
    private final ExecutorService childsService;
    private Map<Vec4i, CPUDensityField> densityFieldCache;
    protected Map<Vec4i, CpuOctree> octreeCache;

    public LevenLinearCPUOctreeImpl(MeshGenerationContext meshGenerationContext, ICSGOperations csgOperations,
                                    Map<Vec4i, CPUDensityField> densityFieldCache, Map<Vec4i, CpuOctree> octreeCache,
                                    Map<Long, ChunkNode> chunks) {
        super(meshGenerationContext, csgOperations, chunks);

        availableProcessors = max(1, Runtime.getRuntime().availableProcessors() / 2);
        service = Executors.newFixedThreadPool(availableProcessors, new ThreadFactory() {
            private final AtomicInteger count = new AtomicInteger();
            @Override
            public Thread newThread(Runnable r) {
                Thread thread = new Thread(r);
                thread.setName("LevenLinearCPUOctreeImpl " + count.getAndIncrement());
                return thread;
            }
        });
        childsService = Executors.newFixedThreadPool(8);
        this.densityFieldCache = densityFieldCache;
        this.octreeCache = octreeCache;
    }

    @Override
    public void computeFreeChunkOctree(Vec3i min, int clipmapNodeSize) {
        Vec4i key = new Vec4i(min, clipmapNodeSize);
        octreeCache.remove(key);
    }

    @Override
    public boolean createLeafVoxelNodes(ChunkNode node, List<OctreeNode> seamNodes, MeshBuffer buffer) {
        CpuOctree octree = LoadOctree(node);
        if(octree==null){
            return false;
        }

        if (octree.numNodes > 0) {
            int indexBufferSize = octree.numNodes * 6 * 3;
            int[] d_indexBuffer = new int[indexBufferSize];
            int trianglesValidSize = octree.numNodes * 3;
            int[] d_trianglesValid = new int[trianglesValidSize];
            int trianglesValidCount = generateMeshMultiThread(octree.nodeCodes.length, octree.octreeNodes, octree.nodeCodes, octree.nodeMaterials,
                    d_indexBuffer, d_trianglesValid);

            int numTriangles = trianglesValidCount * 2;
            int[] d_compactIndexBuffer = new int[numTriangles * 3];
            //////////////////////////////
            compactMeshTriangles(d_trianglesValid, d_indexBuffer, d_compactIndexBuffer);

            MeshVertex[] d_vertexBuffer = new MeshVertex[octree.numNodes];
            //////////////////////////////
            GenerateMeshVertexBuffer(octree.vertexPositions, octree.vertexNormals, octree.nodeMaterials,
                    VoxelHelperUtils.ColourForMinLeafSize(node.size / meshGen.clipmapLeafSize), d_vertexBuffer);
            buffer.setVertices(BufferUtil.createDcFlippedBufferAOS(d_vertexBuffer));
            buffer.setNumVertices(octree.numNodes);
            buffer.setIndicates(BufferUtil.createFlippedBuffer(d_compactIndexBuffer));
            buffer.setNumIndicates(d_compactIndexBuffer.length);

            int[] isSeamNode = new int[octree.numNodes];
            // ToDo return seamNodes which size have seamSize from method
            int seamSize = findSeamNodes(octree.nodeCodes, isSeamNode, 0, octree.numNodes);
            Set<Integer> seamNodeCodes = new HashSet<>(seamSize);
            extractNodeInfo(isSeamNode, VoxelHelperUtils.ColourForMinLeafSize(node.size / meshGen.getVoxelsPerChunk()),//Constants.Yellow,
                    node.size / meshGen.getVoxelsPerChunk(), node.min, 0, octree.numNodes,
                    octree.nodeCodes, octree.nodeMaterials, octree.vertexPositions, octree.vertexNormals, seamNodes, seamNodeCodes);

            List<OctreeNode> addedNodes = findAndCreateBorderNodes(seamNodeCodes, node.min, node.size / meshGen.getVoxelsPerChunk());
            seamNodes.addAll(addedNodes);
        }
        return true;
    }

    @Override
    public CPUDensityField computeApplyCSGOperations(Collection<CSGOperationInfo> opInfo, ChunkNode node) {
        CPUDensityField field = LoadDensityField(node);
        if(field==null)
            return null;

        if(node.size == meshGen.clipmapLeafSize) {
            node.chunkIsChanged = getCsgOperationsProcessor().ApplyCSGOperations(meshGen, opInfo, node, field);
            if(node.chunkIsChanged) {
                node.chunkCSGEdited = true;
            }
            field.lastCSGOperation += opInfo.size();
        } else {
            node.reduceStatus = ReduceStateEnum.CSG_TOUCHED;
            getCsgOperationsProcessor().ApplyReduceOperations(node, field, densityFieldCache);
        }
        if(node.chunkIsChanged || node.reduceStatus.equals(ReduceStateEnum.CSG_TOUCHED)) {
            StoreDensityField(field);
        }
        return field;
    }

    private CpuOctree LoadOctree(ChunkNode node){
        Vec4i key = new Vec4i(node.min, node.size);
        CpuOctree octree = octreeCache.get(key);
        if (octree!=null){
            return octree;
        }
        CPUDensityField field = LoadDensityField(node);
        if(field==null){
            return null;
        }

        if(node.size > meshGen.clipmapLeafSize && node.reduceStatus.equals(ReduceStateEnum.NEED_TO_REDUCE)) {
            getCsgOperationsProcessor().ApplyReduceOperations(node, field, densityFieldCache);
            if (node.chunkCSGEdited) {
                StoreDensityField(field);
                node.reduceStatus = ReduceStateEnum.INITIAL;
            }
        }
        if(field.hermiteEdges.size()>0){
            octree = ConstructOctreeFromField(node.min, node.size, field);
            octreeCache.put(key, octree);
        }
        return octree;
    }

    private void StoreDensityField(CPUDensityField field) {
	    Vec4i key = new Vec4i(field.min, field.size);
        densityFieldCache.put(key, field);
    }

    private CPUDensityField LoadDensityField(ChunkNode node){
        Vec4i key = new Vec4i(node.min, node.size);
        CPUDensityField field = densityFieldCache.get(key);
        if(field==null) {
            field = new CPUDensityField();
            field.min = node.min;
            field.size = node.size;
            if(GenerateDefaultDensityField(field)==0){
                return null;
            }
            FindFieldEdgesPerChild(field, node);
        }

        if(!getCsgOperationsProcessor().isReduceChunk()) {
            Aabb fieldBB = new Aabb(field.min, field.size);
            Set<CSGOperationInfo> csgOperations = new HashSet<>();
            for (int i = field.lastCSGOperation; i < storedOps.size(); i++) {
                if (fieldBB.overlaps(storedOpAABBs.get(i))) {
                    csgOperations.add(storedOps.get(i));
                }
            }
            field.lastCSGOperation = storedOps.size();
            if (!csgOperations.isEmpty()) {
                getCsgOperationsProcessor().ApplyCSGOperations(meshGen, csgOperations, node, field);
                StoreDensityField(field);
            }
        }

        return field;
    }

    private int GenerateDefaultDensityField(CPUDensityField field){
        field.materials = new int[meshGen.getFieldSize() * meshGen.getFieldSize() * meshGen.getFieldSize()];
        int materialSize = GenerateDefaultFieldMultiThread(field.min, field.size / meshGen.getVoxelsPerChunk(), meshGen.MATERIAL_SOLID,
                    field.materials);
        if(materialSize==0){
            field.materials = null;
        }
        return materialSize;
    }

    private CpuOctree ConstructOctreeFromField(Vec3i chunkMin, int chunkSize, CPUDensityField field){
        CpuOctree octree = new CpuOctree();
        int voxelCount = meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk();
        int[] d_leafOccupancy = new int [voxelCount];
        int[] d_leafEdgeInfo = new int [voxelCount];
        int[] d_leafCodes = new int [voxelCount];
        int[] d_leafMaterials = new int [voxelCount];

        octree.numNodes = FindActiveVoxelsMultiThread(field.materials,
                d_leafOccupancy, d_leafEdgeInfo, d_leafCodes, d_leafMaterials);
        if (octree.numNodes<=0){
            return null;
        }

        octree.nodeCodes = new int[octree.numNodes];
        int[] d_compactLeafEdgeInfo = new int[octree.numNodes];
        octree.nodeMaterials = new int[octree.numNodes];
        //////////////////////////////
        octree.octreeNodes = compactVoxels(d_leafOccupancy, d_leafEdgeInfo, d_leafCodes, d_leafMaterials,
                octree.nodeCodes, d_compactLeafEdgeInfo, octree.nodeMaterials, octree.numNodes);

        QEFData[] qefs = new QEFData[octree.numNodes];
        octree.vertexNormals = new Vec4f[octree.numNodes];
        createLeafNodesMultiThread(octree.numNodes, octree.nodeCodes, d_compactLeafEdgeInfo, field.hermiteEdges,
                qefs, octree.vertexNormals);

        octree.vertexPositions = new Vec4f[octree.numNodes];
        solveQEFsMultiThread(octree.nodeCodes, chunkSize, meshGen.getVoxelsPerChunk(), chunkMin,
                qefs, octree.vertexPositions, octree.numNodes);
        return octree;
    }

    private int GenerateDefaultFieldMultiThread(Vec3i offset, int sampleScale, int defaultMaterialIndex,
                                                int[] field_materials) {
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
                    size = processMaterial(offset, sampleScale, defaultMaterialIndex, field_materials, size, new Vec3i(x, y, z));
                }
                return size;
            });
        }
        return VoxelOctree.performIntCallableTask(tasks, service, logger);
    }

    private int processMaterial(Vec3i offset, int sampleScale, int defaultMaterialIndex, int[] field_materials, int size, Vec3i local_pos) {
        Vec3i world_pos = local_pos.mul(sampleScale).add(offset);
        float density = getNoise(world_pos);
        int material = density < 0.f ? defaultMaterialIndex : meshGen.MATERIAL_AIR;
        field_materials[meshGen.getMaterialIndex(local_pos)] = material;
        if (material == defaultMaterialIndex) size++;
        return size;
    }

    private int FindFieldEdgesPerChild(CPUDensityField field, ChunkNode node) {

        int childSize = meshGen.getHermiteIndexSize()/2;
        List<Callable<Integer>> tasks = new ArrayList<>();
        for (int i = 0; i < 8; i++) {
            long locCodeChild = (node.chunkCode<<3)|i;
            ChunkNode child = getChunksMap().get(locCodeChild);
            if(child!=null && child.chunkCSGEdited) {
                continue;
            }
            Vec3i childOffset = meshGen.offset(i, childSize);
            tasks.add(() -> {
                int size = 0;
                for (int z = 0; z < (childOffset.z == 0 ? childSize : childSize + 1); z++) {
                    for (int y = 0; y < (childOffset.y == 0 ? childSize : childSize + 1); y++) {
                        for (int x = 0; x < (childOffset.x == 0 ? childSize : childSize + 1); x++) {
                            size = processFindFieldEdges(field.min, field.size / meshGen.getVoxelsPerChunk(), field.materials, size,
                                    z + childOffset.z, y + childOffset.y, x + childOffset.x, field.hermiteEdges);
                        }
                    }
                }
                return size;
            });
        }
        return VoxelOctree.performIntCallableTask(tasks, childsService, logger);
    }

    private void FindFieldEdgesMultiThread(CPUDensityField field) {
        int bound = meshGen.getHermiteIndexSize();
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
                    size = processFindFieldEdges(field.min, field.size / meshGen.getVoxelsPerChunk(),
                            field.materials, size, z, y, x, field.hermiteEdges);
                }
                return size;
            });
        }
        VoxelOctree.performIntCallableTask(tasks, service, logger);
    }

    private int processFindFieldEdges(Vec3i chunkMin, int sampleScale, int[] materials,
                                      int size, int z, int y, int x, Map<Integer, Vec4f> normals) {
        Vec3i pos = new Vec3i(x, y, z);

        int[] CORNER_MATERIALS = {
                materials[meshGen.getMaterialIndex(pos.add(new Vec4i(0, 0, 0, 0)))],
                materials[meshGen.getMaterialIndex(pos.add(new Vec4i(1, 0, 0, 0)))],
                materials[meshGen.getMaterialIndex(pos.add(new Vec4i(0, 1, 0, 0)))],
                materials[meshGen.getMaterialIndex(pos.add(new Vec4i(0, 0, 1, 0)))],
        };

        for (int i = 0; i < 3; i++) {
            int e = 1 + i;
            boolean signChange = (CORNER_MATERIALS[0] != meshGen.MATERIAL_AIR && CORNER_MATERIALS[e] == meshGen.MATERIAL_AIR) ||
                    (CORNER_MATERIALS[0] == meshGen.MATERIAL_AIR && CORNER_MATERIALS[e] != meshGen.MATERIAL_AIR);
            if (signChange) {
                normals.put(meshGen.getEdgeCodeByPos(x,y,z, i), calculateNorm(chunkMin, sampleScale, i, x, y, z));
                ++size;
            }
        }
        return size;
    }

    private Vec4f calculateNorm(Vec3i chunkMin, int sampleScale, int axisIndex, int x, int y, int z){
        Vec3f p0 = new Vec3i(x, y, z).mul(sampleScale).add(chunkMin).toVec3f();
        Vec3f p1 = p0.add(EDGE_END_OFFSETS[axisIndex].mul(sampleScale).toVec3f());

        Vec4f p = ApproximateLevenCrossingPosition(p0, p1);
        Vec4f normal = CalculateSurfaceNormal(p);
        return new Vec4f(normal.getVec3f(), p.w);
    }

    private int FindActiveVoxelsMultiThread(int[] materials,
                                 int[] voxelOccupancy, int[] voxelEdgeInfo, int[] voxelPositions, int[] voxelMaterials) {
        List<Callable<Integer>> tasks = new ArrayList<>();
        int bound = voxelMaterials.length;
        int threadBound = bound / availableProcessors;

        for (int i = 0; i < availableProcessors; i++) {
            int from = i * threadBound;
            int to = from + threadBound;
            boolean last = (i == availableProcessors - 1 && to <= bound - 1);
            tasks.add(() -> FindActiveVoxels(from, last ? bound : to, materials,
                    voxelOccupancy, voxelEdgeInfo, voxelPositions, voxelMaterials));
        }
        return VoxelOctree.performIntCallableTask(tasks, service, logger);
    }

    private int FindActiveVoxels(int from, int to,
                         int[] materials,
                         int[] voxelOccupancy, int[] voxelEdgeInfo, int[] voxelPositions, int[] voxelMaterials) {
        int size = 0;
        for (int k = from; k < to; k++) {
            int indexShift = VoxelHelperUtils.log2(meshGen.getVoxelsPerChunk()); // max octree depth
            int x = (k >> (indexShift * 0)) & meshGen.getVoxelsPerChunk() - 1;
            int y = (k >> (indexShift * 1)) & meshGen.getVoxelsPerChunk() - 1;
            int z = (k >> (indexShift * 2)) & meshGen.getVoxelsPerChunk() - 1;

            int index = x + (y * meshGen.getVoxelsPerChunk()) + (z * meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk());
            Vec3i pos = new Vec3i(x, y, z);

            int[] cornerMaterials = {
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[0]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[1]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[2]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[3]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[4]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[5]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[6]))],
                    materials[meshGen.getMaterialIndex(pos.add(CHILD_MIN_OFFSETS[7]))],
            };

            // record the on/off values at the corner of each voxel
            int cornerValues = 0;
            cornerValues |= (((cornerMaterials[0]) == meshGen.MATERIAL_AIR ? 0 : 1) << 0);
            cornerValues |= (((cornerMaterials[1]) == meshGen.MATERIAL_AIR ? 0 : 1) << 1);
            cornerValues |= (((cornerMaterials[2]) == meshGen.MATERIAL_AIR ? 0 : 1) << 2);
            cornerValues |= (((cornerMaterials[3]) == meshGen.MATERIAL_AIR ? 0 : 1) << 3);
            cornerValues |= (((cornerMaterials[4]) == meshGen.MATERIAL_AIR ? 0 : 1) << 4);
            cornerValues |= (((cornerMaterials[5]) == meshGen.MATERIAL_AIR ? 0 : 1) << 5);
            cornerValues |= (((cornerMaterials[6]) == meshGen.MATERIAL_AIR ? 0 : 1) << 6);
            cornerValues |= (((cornerMaterials[7]) == meshGen.MATERIAL_AIR ? 0 : 1) << 7);

            boolean haveVoxel = cornerValues != 0 && cornerValues != 255;
            if (haveVoxel) {
                ++size;
            }
            // record which of the 12 voxel edges are on/off
            int edgeList = 0;
            for (int i = 0; i < 12; i++) {
                int i0 = edgevmap[i][0];
                int i1 = edgevmap[i][1];
                int edgeStart = (cornerValues >> i0) & 1;
                int edgeEnd = (cornerValues >> i1) & 1;
                int signChange = edgeStart != edgeEnd ? 1 : 0;
                edgeList |= (signChange << i);
            }
            voxelOccupancy[index] = haveVoxel ? 1 : 0;
            voxelPositions[index] = meshGen.codeForPosition(pos);
            voxelEdgeInfo[index] = edgeList;

            // store cornerValues here too as its needed by the CPU side and edgeInfo isn't exported
            int materialIndex = findDominantMaterial(cornerMaterials);
            voxelMaterials[index] = (materialIndex << 8) | cornerValues;
        }
        return size;
    }

    private Map<Integer, Integer> compactVoxels(int[] voxelValid, int[] voxelEdgeInfo, int[] voxelPositions, int[] voxelMaterials,
                                                int[] compactPositions, int[] compactEdgeInfo, int[] compactMaterials, int numVertices){
        int current = 0;
        Map<Integer, Integer> octreeNodes = new HashMap<>(numVertices);
        for (int i = 0; i < voxelPositions.length; i++) {
            if (voxelValid[i]==1) {
                octreeNodes.put(voxelPositions[i], current);
                compactPositions[current] = voxelPositions[i];
                compactEdgeInfo[current] = voxelEdgeInfo[i];
                compactMaterials[current] = voxelMaterials[i];
                ++current;
            }
        }
        return octreeNodes;
    }

    private void createLeafNodesMultiThread(int bound, int[] voxelPositions, int[] voxelEdgeInfo, Map<Integer, Vec4f> nodes,
                                               QEFData[] leafQEFs, Vec4f[] vertexNormals) {
        final int threadBound = bound / availableProcessors;
        List<Callable<Boolean>> tasks = new ArrayList<>();
        for (int i = 0; i < availableProcessors; i++) {
            int from = i * threadBound;
            int to = from + threadBound;
            boolean last = (i == availableProcessors - 1 && to <= bound - 1);
            tasks.add(() -> {
                createLeafNodes(from, last ? bound : to, voxelPositions, voxelEdgeInfo, nodes, leafQEFs, vertexNormals);
                return true;
            });
        }
        VoxelOctree.performBoolCallableTask(tasks, service, logger);
    }

    private void createLeafNodes(int from, int to, int[] voxelPositions, int[] voxelEdgeInfo, Map<Integer, Vec4f> nodes,
                         QEFData[] leafQEFs, Vec4f[] vertexNormals)
    {
        for (int index = from; index < to; index++) {
            int encodedPosition = voxelPositions[index];
            Vec3i position = meshGen.positionForCode(encodedPosition);
            int edgeList = voxelEdgeInfo[index];

            Vec4f[] edgePositions = new Vec4f[12];
            Vec4f[] edgeNormals = new Vec4f[12];
            int edgeCount = 0;

            for (int i = 0; i < 12; i++) {
                int active = (edgeList >> i) & 1;
                if (active==0) {
                    continue;
                }
                int e0 = edgevmap[i][0];
                int e1 = edgevmap[i][1];
                Vec4f p0 = position.add(CHILD_MIN_OFFSETS[e0]).toVec4f();
                Vec4f p1 = position.add(CHILD_MIN_OFFSETS[e1]).toVec4f();

                // this works due to the layout EDGE_VERTEX_MAP, the first 4 elements are the X axis
                // the next 4 are the Y axis and the last 4 are the Z axis
                int axis = i / 4;
                Vec3i hermiteIndexPosition = position.add(CHILD_MIN_OFFSETS[e0]);
                int edgeIndex = (meshGen.encodeVoxelIndex(hermiteIndexPosition) << 2) | axis;

                Vec4f edgeData = nodes.get(edgeIndex);
                if (edgeData!=null) {
                    edgePositions[edgeCount] = VoxelHelperUtils.mix(p0, p1, edgeData.w);//.mul(sampleScale);
                    edgeNormals[edgeCount] = new Vec4f(edgeData.x, edgeData.y, edgeData.z, 0);
                    edgeCount++;
                }
            }
            QEFData qef = new QEFData(new LevenQefSolver());
            qef.qef_create_from_points(edgePositions, edgeNormals, edgeCount);
            leafQEFs[index] = qef;

            Vec4f normal = new Vec4f(0.f, 0.f, 0.f, 0.f);
            for (int i = 0; i < edgeCount; i++) {
                normal = normal.add(edgeNormals[i]);
                normal.w += 1.f;
            }

            Vec4f nor = normal.div(normal.w).normalize();
            normal.w = 0.f;

            vertexNormals[index] = nor;
        }
    }

    private void solveQEFsMultiThread(int[] d_nodeCodes, int chunkSize, int voxelsPerChunk, Vec3i chunkMin,
                                      QEFData[] qefs, Vec4f[] solvedPosition, int bound) {
        final int threadBound = bound / availableProcessors;
        List<Callable<Boolean>> tasks = new ArrayList<>();
        for (int i = 0; i < availableProcessors; i++) {
            int from = i * threadBound;
            int to = from + threadBound;
            boolean last = (i == availableProcessors - 1 && to <= bound - 1);
            tasks.add(() -> {
                solveQEFs(d_nodeCodes, chunkSize, voxelsPerChunk, chunkMin, from, last ? bound : to, qefs, solvedPosition);
                return true;
            });
        }
        VoxelOctree.performBoolCallableTask(tasks, service, logger);
    }

    private void solveQEFs(int[] d_nodeCodes, int chunkSize, int voxelsPerChunk, Vec3i chunkMin, int from, int to,
                           QEFData[] qefs, Vec4f[] solvedPositions){
        for (int index = from; index < to; index++) {
            int leafSize = (chunkSize / voxelsPerChunk);

            Vec4f solvedPos;
            if(leafSize == meshGen.leafSizeScale) {
                solvedPos = qefs[index].solve();       // run solver only for LOD 0
            } else {
                solvedPos = qefs[index].getMasspoint();// for other LOD's get masspoint - to increase performance
            }

            solvedPos = solvedPos.mul(leafSize).add(chunkMin);
            if(enableQefClamping) {
                Vec3i leaf = meshGen.positionForCode(d_nodeCodes[index]).mul(leafSize).add(chunkMin);
                Vec4f massPoint = qefs[index].getMasspoint().mul(leafSize).add(chunkMin);
                solvedPos = VoxelHelperUtils.isOutFromBounds(solvedPos.getVec3f(), leaf.toVec3f(), leafSize) ? massPoint : solvedPos;
            }
            solvedPositions[index] = solvedPos;
        }
    }

    private int findSeamNodes(int[] nodeCodes, int[] isSeamNode, int from, int to) {
        int res = 0;
        for (int index = from; index < to; index++) {
            int code = nodeCodes[index];
            Vec3i position = meshGen.positionForCode(code);
            boolean xSeam = position.x == 0 || position.x == (meshGen.getVoxelsPerChunk() - 1);
            boolean ySeam = position.y == 0 || position.y == (meshGen.getVoxelsPerChunk() - 1);
            boolean zSeam = position.z == 0 || position.z == (meshGen.getVoxelsPerChunk() - 1);
            boolean isSeam = xSeam | ySeam | zSeam;
            if(isSeam) {
                ++res;
            }
            isSeamNode[index] = xSeam | ySeam | zSeam ? 1 : 0;
        }
        return res;
    }

    private final Vec3i[][] EDGE_NODE_OFFSETS = {
            { new Vec3i(0, 0, 0), new Vec3i(0, 0, 1), new Vec3i(0, 1, 0), new Vec3i(0, 1, 1) },
            { new Vec3i(0, 0, 0), new Vec3i(1, 0, 0), new Vec3i(0, 0, 1), new Vec3i(1, 0, 1) },
            { new Vec3i(0, 0, 0), new Vec3i(0, 1, 0), new Vec3i(1, 0, 0), new Vec3i(1, 1, 0) },
    };

    private int generateMeshMultiThread(int bound, Map<Integer, Integer> nodes, int[] octreeNodeCodes, int[] octreeMaterials,
                                         int[] meshIndexBuffer, int[] trianglesValid) {
        List<Callable<Integer>> tasks = new ArrayList<>();
        final int threadBound = bound / availableProcessors;

        for (int i = 0; i < availableProcessors; i++) {
            int from = i * threadBound;
            int to = from + threadBound;
            boolean last = (i == availableProcessors - 1 && to <= bound - 1);
            tasks.add(() -> generateMesh(from, last ? bound : to, nodes, octreeNodeCodes, octreeMaterials,
                    meshIndexBuffer, trianglesValid));
        }
        return VoxelOctree.performIntCallableTask(tasks, service, logger);
    }

    private int generateMesh(int from, int to, Map<Integer, Integer> nodes, int[] octreeNodeCodes, int[] octreeMaterials,
                             int[] meshIndexBuffer, int[] trianglesValid) {
        int size = 0;
        for (int index = from; index < to; index++) {
            int code = octreeNodeCodes[index];
            int triIndex = index * 3;

            Vec3i offset = meshGen.positionForCode(code);
            int[] pos = {offset.x, offset.y, offset.z};
            Integer[] nodeIndices = {null, null, null, null};

            for (int axis = 0; axis < 3; axis++) {
                trianglesValid[triIndex + axis] = 0;

                // need to check that the position generated when the offsets are added won't exceed
                // the chunk bounds -- if this happens rather than failing the octree lookup
                // will actually wrap around to 0 again causing weird polys to be generated

                int a = pos[(axis + 1) % 3];
                int b = pos[(axis + 2) % 3];
                boolean isEdgeVoxel = a == (meshGen.getVoxelsPerChunk() - 1) || b == (meshGen.getVoxelsPerChunk() - 1);
                if (isEdgeVoxel) {
                    continue;
                }

                nodeIndices[0] = index;
                for (int n = 1; n < 4; n++) {
                    Vec3i p = offset.add(EDGE_NODE_OFFSETS[axis][n]);
                    int c = meshGen.codeForPosition(p);
                    nodeIndices[n] = nodes.get(c);
                }

                if (nodeIndices[1] != null && nodeIndices[2] != null && nodeIndices[3] != null) {
                    int bufferOffset = (triIndex * 6) + (axis * 6);
                    int trisEmitted = processEdge(nodeIndices, octreeMaterials[index], axis, meshIndexBuffer, bufferOffset);
                    size += trisEmitted;
                    trianglesValid[triIndex + axis] = trisEmitted;
                }
            }
        }
        return size;
    }

    private int processEdge(Integer[] nodeIndices, int nodeMaterial, int axis, int[] indexBuffer, int bufferOffset) {
        int edge = (axis * 4) + 3;
        int c1 = edgevmap[edge][0];
        int c2 = edgevmap[edge][1];

        int corners = nodeMaterial & 0xff;
        int m1 = (corners >> c1) & 1;
        int m2 = (corners >> c2) & 1;

        boolean signChange = (m1 == meshGen.MATERIAL_AIR && m2 != meshGen.MATERIAL_AIR) || (m1 != meshGen.MATERIAL_AIR && m2 == meshGen.MATERIAL_AIR);
        if (!signChange) {
            return 0;
        }

        // flip the winding depending on which end of the edge is outside the volume
        int flip = m1 != 0 ? 1 : 0;
        int[][] indices = {
                // different winding orders depending on the sign change direction
                {0, 1, 3, 0, 3, 2},
                {0, 3, 1, 0, 2, 3},
        };

        indexBuffer[bufferOffset + 0] = nodeIndices[indices[flip][0]];
        indexBuffer[bufferOffset + 1] = nodeIndices[indices[flip][1]];
        indexBuffer[bufferOffset + 2] = nodeIndices[indices[flip][2]];
        indexBuffer[bufferOffset + 3] = nodeIndices[indices[flip][3]];
        indexBuffer[bufferOffset + 4] = nodeIndices[indices[flip][4]];
        indexBuffer[bufferOffset + 5] = nodeIndices[indices[flip][5]];
        return 1;
    }

    private int compactMeshTriangles(int[] trianglesValid, int[] meshIndexBuffer, int[] compactMeshIndexBuffer) {
        int current = 0;
        for (int index = 0; index < trianglesValid.length; index++) {
            if (trianglesValid[index] == 1) {
                int scanOffset = current * 6;
                int bufferOffset = (index * 6);
                System.arraycopy(meshIndexBuffer, bufferOffset, compactMeshIndexBuffer, scanOffset, 6);
//                for (int i = 0; i < 6; i++) {
//                    compactMeshIndexBuffer[scanOffset + i] = meshIndexBuffer[bufferOffset + i];
//                }
                ++current;
            }
        }
        return current;
    }

    private void GenerateMeshVertexBuffer(Vec4f[] vertexPositions, Vec4f[] vertexNormals, int[] nodeMaterials, Vec3f colour,
                                          MeshVertex[] meshVertexBuffer) {
        for (int index = 0; index < vertexPositions.length; index++) {
            int material = nodeMaterials[index];
            meshVertexBuffer[index] = new MeshVertex();
            meshVertexBuffer[index].setPos(vertexPositions[index].getVec3f());
            meshVertexBuffer[index].setNormal(vertexNormals[index].getVec3f());
            meshVertexBuffer[index].setColor(colour); //colour = new Vec4f(colour.X, colour.Y, colour.Z, (float) (material >> 8));
        }
    }

    private void extractNodeInfo(int[] isSeamNode, Vec3f color,
                                 int leafSize, Vec3i chunkMin, int from, int to,
                                 int[] octreeCodes, int[] octreeMaterials, Vec4f[] octreePositions, Vec4f[] octreeNormals,
                                 List<OctreeNode> seamNodes, Set<Integer> seamNodeCodesForSearch) {
        int i=0;
        for (int index = from; index < to; index++) {
            if (isSeamNode==null || isSeamNode[index]==1) {
                if(leafSize > meshGen.leafSizeScale) {
                    seamNodeCodesForSearch.add(octreeCodes[index]);
                }
                Vec3i min = meshGen.positionForCode(octreeCodes[index]).mul(leafSize).add(chunkMin);
                PointerBasedOctreeNode node = new PointerBasedOctreeNode(min, leafSize, OctreeNodeType.Node_Leaf);
                node.corners = octreeMaterials[index];
                OctreeDrawInfo drawInfo = new OctreeDrawInfo();
                drawInfo.position = octreePositions[index].getVec3f();
                drawInfo.color = color;
                drawInfo.averageNormal = octreeNormals[index].getVec3f();
                node.drawInfo = drawInfo;
                node.nodeNum = meshGen.positionForCode(octreeCodes[index]);
                seamNodes.add(node);
            }
        }
    }

//    private int[] compactEdges(int[] edgeValid, int[] edges, int compactEdgesSize) {
//        int[] compactActiveEdges = new int [compactEdgesSize];
//        int current = 0;
//        for (int index = 0; index < edges.length; index++) {
//            if (edgeValid[index]==1) {
//                compactActiveEdges[current] = edges[index];
//                ++current;
//            }
//        }
//        return compactActiveEdges;
//    }

//    private Vec4f[] FindEdgeIntersectionInfoMultiThread(Vec3i chunkMin, int sampleScale, int[] encodedEdges, int bound) {
//
//        Vec4f[] normals = new Vec4f[bound];
//        final int threadBound = bound / availableProcessors;
//        List<Callable<Boolean>> tasks = new ArrayList<>();
//        for (int i = 0; i < availableProcessors; i++) {
//            int from = i * threadBound;
//            int to = from + threadBound;
//            boolean last = (i == availableProcessors - 1 && to <= bound - 1);
//            tasks.add(() -> {
//                FindEdgeIntersectionInfo(chunkMin, sampleScale, from, last ? bound : to, encodedEdges, normals);
//                return true;
//            });
//        }
//        performBoolCallableTask(tasks, service);
//        return normals;
//    }

    //    private void FindEdgeIntersectionInfo(Vec3i chunkMin, int sampleScale, int from, int to, int[] encodedEdges,
//                                          Vec4f[] normals) {
//        for (int index = from; index < to; index++) {
//            int edge = encodedEdges[index];
//            int axisIndex = edge & 3;
//            int hermiteIndex = edge >> 2;
//
//            int x = (hermiteIndex >> (meshGen.getIndexShift() * 0)) & meshGen.getIndexMask();
//            int y = (hermiteIndex >> (meshGen.getIndexShift() * 1)) & meshGen.getIndexMask();
//            int z = (hermiteIndex >> (meshGen.getIndexShift() * 2)) & meshGen.getIndexMask();
//            normals[index] = calculateNorm(chunkMin, sampleScale, axisIndex, x, y, z);
//        }
//    }

    //    private int GenerateDefaultField(Vec3i offset, int sampleScale, int defaultMaterialIndex,
//                              int[] field_materials)
//    {
//        int size = 0;
//        for (int z = 0; z < meshGen.getFieldSize(); z++) {
//            for (int y = 0; y < meshGen.getFieldSize(); y++) {
//                for (int x = 0; x < meshGen.getFieldSize(); x++) {
//                    size = processMaterial(offset, sampleScale, defaultMaterialIndex, field_materials, size, new Vec3i(x, y, z));
//                }
//            }
//        }
//        return size;
//    }
//
//    private int GenerateDefaultField1d(Vec3i offset, int sampleScale, int defaultMaterialIndex,
//                                     int[] field_materials)
//    {
//        int size = 0;
//        int bound = meshGen.getFieldSize();
//        for (int i = 0; i < bound * bound * bound; i++) {
//            int x = i % bound;
//            int y = (i / bound) % bound;
//            int z = (i / bound / bound);
//            size = processMaterial(offset, sampleScale, defaultMaterialIndex, field_materials, size, new Vec3i(x, y, z));
//        }
//        return size;
//    }
}