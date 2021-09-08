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
import java.util.concurrent.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.logging.Level;
import java.util.logging.Logger;

import static dc.utils.SimplexNoise.getNoise;
import static java.lang.Math.max;

/*
    Nick Gildea Leven OpenCL kernels Dual contouring implementation translated to java CPU
    Some holes in seams is not fixed.
    The first raw version will still improve.
 */

public class LevenLinearCPUOctreeImpl extends AbstractDualContouring implements VoxelOctree {
    final public static Logger logger = Logger.getLogger(LevenLinearCPUOctreeImpl.class.getName());
    private final ExecutorService service;
    int availableProcessors;

    public LevenLinearCPUOctreeImpl(MeshGenerationContext meshGenerationContext, ICSGOperations csgOperations,
                                    Map<Vec4i, GPUDensityField> densityFieldCache, Map<Vec4i, GpuOctree> octreeCache) {
        super(meshGenerationContext, csgOperations, densityFieldCache, octreeCache);

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
    }

    @Override
    public GPUDensityField computeApplyCSGOperations(Collection<CSGOperationInfo> opInfo, Vec3i clipmapNodeMin, int clipmapNodeSize) {
        GPUDensityField field = LoadDensityField(clipmapNodeMin, clipmapNodeSize);
        if(field==null){
            return null;
        }
        csgOperationsProcessor.ApplyCSGOperations(meshGen, opInfo, clipmapNodeMin, clipmapNodeSize, field);
        field.lastCSGOperation += opInfo.size();

        StoreDensityField(field);
        return field;
    }

    @Override
    public boolean createLeafVoxelNodes(int chunkSize, Vec3i chunkMin, List<OctreeNode> seamNodes, MeshBuffer buffer) {
        GpuOctree octree = LoadOctree(chunkMin, chunkSize);
        if(octree==null){
            return false;
        }

        if (octree.numNodes > 0) {
            int indexBufferSize = octree.numNodes * 6 * 3;
            int[] d_indexBuffer = new int[indexBufferSize];
            int trianglesValidSize = octree.numNodes * 3;
            int[] d_trianglesValid = new int[trianglesValidSize];
            //////////////////////////////
            int trianglesValidCount = generateMesh(octree.octreeNodes, octree.d_nodeCodesCpu, octree.d_nodeMaterialsCpu,
                    d_indexBuffer, d_trianglesValid);

            int numTriangles = trianglesValidCount * 2;
            int[] d_compactIndexBuffer = new int[numTriangles * 3];
            //////////////////////////////
            compactMeshTriangles(d_trianglesValid, d_indexBuffer, d_compactIndexBuffer);

            MeshVertex[] d_vertexBuffer = new MeshVertex[octree.numNodes];
            //////////////////////////////
            GenerateMeshVertexBuffer(octree.d_vertexPositionsCpu, octree.d_vertexNormalsCpu, octree.d_nodeMaterialsCpu,
                    VoxelHelperUtils.ColourForMinLeafSize(chunkSize / meshGen.clipmapLeafSize), d_vertexBuffer);
            buffer.setVertices(BufferUtil.createDcFlippedBufferAOS(d_vertexBuffer));
            buffer.setNumVertices(octree.numNodes);
            buffer.setIndicates(BufferUtil.createFlippedBuffer(d_compactIndexBuffer));
            buffer.setNumIndicates(d_compactIndexBuffer.length);

            int[] isSeamNode = new int[octree.numNodes];
            // ToDo return seamNodes which size have seamSize from method
            int seamSize = findSeamNodes(octree.d_nodeCodesCpu, isSeamNode, 0, octree.numNodes);

            extractNodeInfo(isSeamNode, VoxelHelperUtils.ColourForMinLeafSize(chunkSize / meshGen.getVoxelsPerChunk()),//Constants.Yellow,
                    chunkSize / meshGen.getVoxelsPerChunk(), chunkMin, 0, octree.numNodes,
                    octree.d_nodeCodesCpu, octree.d_nodeMaterialsCpu, octree.d_vertexPositionsCpu, octree.d_vertexNormalsCpu, seamNodes);

            Map<Vec3i, OctreeNode> seamNodesMap = new HashMap<>();
            for (OctreeNode seamNode : seamNodes) {
                if (seamNode.size > meshGen.leafSizeScale) {
                    seamNodesMap.put(seamNode.min, seamNode);
                }
            }
            List<OctreeNode> addedNodes = findAndCreateBorderNodes(seamNodesMap);
            seamNodes.addAll(addedNodes);
        }
        return true;
    }

    private GpuOctree LoadOctree(Vec3i chunkMin, int chunkSize){
        Vec4i key = new Vec4i(chunkMin, chunkSize);
        GpuOctree octree = octreeCache.get(key);
        if (octree!=null){
            return octree;
        }
        GPUDensityField field = LoadDensityField(chunkMin, chunkSize);
        if(field==null){
            return null;
        }
        if(field.numEdges>0){
            octree = ConstructOctreeFromField(chunkMin, chunkSize, field);
            octreeCache.put(key, octree);
        }
        return octree;
    }

    private void StoreDensityField(GPUDensityField field) {
	    Vec4i key = new Vec4i(field.min, field.size);
        densityFieldCache.put(key, field);
    }

    private GPUDensityField LoadDensityField(Vec3i chunkMin, int chunkSize){
        Vec4i key = new Vec4i(chunkMin, chunkSize);
        GPUDensityField field = densityFieldCache.get(key);
        if(field==null) {
            field = new GPUDensityField();
            field.min = chunkMin;
            field.size = chunkSize;
            if(GenerateDefaultDensityField(field)==0){
                return null;
            }
            FindDefaultEdges(field);
        }

        Aabb fieldBB = new Aabb(field.min, field.size);
        Set<CSGOperationInfo> csgOperations = new HashSet<>();
        for (int i = field.lastCSGOperation; i < storedOps.size(); i++) {
            if (fieldBB.overlaps(storedOpAABBs.get(i))) {
                csgOperations.add(storedOps.get(i));
            }
        }

        field.lastCSGOperation = storedOps.size();

        if (!csgOperations.isEmpty()) {
            csgOperationsProcessor.ApplyCSGOperations(meshGen, csgOperations, field.min, field.size, field);
            StoreDensityField(field);
        }

        return field;
    }

    private int GenerateDefaultDensityField(GPUDensityField field){
        field.materialsCpu = new int[meshGen.getFieldSize() * meshGen.getFieldSize() * meshGen.getFieldSize()];
        int materialSize = GenerateDefaultFieldMultiThread(field.min, field.size / meshGen.getVoxelsPerChunk(), meshGen.MATERIAL_SOLID,
                    field.materialsCpu);
        if(materialSize==0){
            field.materialsCpu = null;
        }
        return materialSize;
    }

    private void FindDefaultEdges(GPUDensityField field){
        int edgeBufferSize = meshGen.getHermiteIndexSize() * meshGen.getHermiteIndexSize() * meshGen.getHermiteIndexSize() * 3;
        int[] edgeOccupancy = new int[edgeBufferSize];
        int[] edgeIndicesNonCompact = new int[edgeBufferSize];
        //////////////////////////////
        field.numEdges = FindFieldEdges(field.materialsCpu,
                edgeOccupancy, edgeIndicesNonCompact);
        if(field.numEdges==0 || field.numEdges<0){
            return;
        }

        field.edgeIndicesCpu = compactEdges(edgeOccupancy, edgeIndicesNonCompact, field.numEdges);
        field.normalsCpu = FindEdgeIntersectionInfo(field.min, field.size / meshGen.getVoxelsPerChunk(), 0, field.numEdges, field.edgeIndicesCpu);
    }

    private GpuOctree ConstructOctreeFromField(Vec3i chunkMin, int chunkSize, GPUDensityField field){
        GpuOctree octree = new GpuOctree();

        int[] d_leafOccupancy = new int [meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()];
        int[] d_leafEdgeInfo = new int [meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()];
        int[] d_leafCodes = new int [meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()];
        int[] d_leafMaterials = new int [meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()];
        //////////////////////////////
        octree.numNodes = FindActiveVoxels(chunkSize, chunkMin,
                0, meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk()*meshGen.getVoxelsPerChunk(), field.materialsCpu,
                d_leafOccupancy, d_leafEdgeInfo, d_leafCodes, d_leafMaterials);
        if (octree.numNodes<=0){
            return null;
        }

        octree.d_nodeCodesCpu = new int[octree.numNodes];
        int[] d_compactLeafEdgeInfo = new int[octree.numNodes];
        octree.d_nodeMaterialsCpu = new int[octree.numNodes];
        //////////////////////////////
        octree.octreeNodes = compactVoxels(d_leafOccupancy, d_leafEdgeInfo, d_leafCodes, d_leafMaterials,
                octree.d_nodeCodesCpu, d_compactLeafEdgeInfo, octree.d_nodeMaterialsCpu, octree.numNodes);

        QEFData[] qefs = new QEFData[octree.numNodes];
        octree.d_vertexNormalsCpu = new Vec4f[octree.numNodes];
        //////////////////////////////

        Map<Integer, Integer> edgeIndicatesMap = new HashMap<>(field.edgeIndicesCpu.length);
        for(int i =0; i<field.edgeIndicesCpu.length; i++){
            edgeIndicatesMap.put(field.edgeIndicesCpu[i], i);
        }
        createLeafNodes(0, octree.numNodes, octree.d_nodeCodesCpu, d_compactLeafEdgeInfo, field.normalsCpu, edgeIndicatesMap,
                qefs, octree.d_vertexNormalsCpu);

        octree.d_vertexPositionsCpu = new Vec4f[octree.numNodes];
        //////////////////////////////
        solveQEFs(octree.d_nodeCodesCpu, chunkSize, meshGen.getVoxelsPerChunk(), chunkMin, 0, octree.numNodes, qefs,
                octree.d_vertexPositionsCpu);
        return octree;
    }

    private int GenerateDefaultField(Vec3i offset, int sampleScale, int defaultMaterialIndex,
                              int[] field_materials)
    {
        int size = 0;
        for (int z = 0; z < meshGen.getFieldSize(); z++) {
            for (int y = 0; y < meshGen.getFieldSize(); y++) {
                for (int x = 0; x < meshGen.getFieldSize(); x++) {
                    size = processMaterial(offset, sampleScale, defaultMaterialIndex, field_materials, size, new Vec3i(x, y, z));
                }
            }
        }
        return size;
    }

    private int GenerateDefaultField1d(Vec3i offset, int sampleScale, int defaultMaterialIndex,
                                     int[] field_materials)
    {
        int size = 0;
        int bound = meshGen.getFieldSize();
        for (int i = 0; i < bound * bound * bound; i++) {
            int x = i % bound;
            int y = (i / bound) % bound;
            int z = (i / bound / bound);
            size = processMaterial(offset, sampleScale, defaultMaterialIndex, field_materials, size, new Vec3i(x, y, z));
        }
        return size;
    }

    private int GenerateDefaultFieldMultiThread(Vec3i offset, int sampleScale, int defaultMaterialIndex,
                                                int[] field_materials) {
        List<Callable<Integer>> tasks = new ArrayList<>();
        int bound = meshGen.getFieldSize();
        int threadBound = (bound * bound * bound) / availableProcessors;

        for (int i = 0; i < availableProcessors; i++) {
            int finalI = i;
            Callable<Integer> task = () -> {
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
            };
            tasks.add(task);
        }

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

    private int processMaterial(Vec3i offset, int sampleScale, int defaultMaterialIndex, int[] field_materials, int size, Vec3i local_pos) {
        Vec3i world_pos = local_pos.mul(sampleScale).add(offset);
        float density = getNoise(world_pos);
        //float cube = SimplexNoise.Density_Cuboid(world_pos.toVec3f(), new Vec3f(-139.034f,-176.0f,-1923.888f), new Vec3f(4.0f, 4.0f, 4.0f));
        //float cube = SimplexNoise.Density_Sphere(new Vec3f(world_pos), new Vec3f(-139.034f,-176.0f,-1923.888f), 4);
        //float densityCorr = Math.max(-cube, density);
        int material = density < 0.f ? defaultMaterialIndex : meshGen.MATERIAL_AIR;
        field_materials[field_index(local_pos)] = material;
        if (material == defaultMaterialIndex) size++;
        return size;
    }

    private int FindFieldEdges(int[] materials,
                       int[] edgeOccupancy, int[] edgeIndices) {
        int size = 0;
        for (int z = 0; z < meshGen.getHermiteIndexSize(); z++) {
            for (int y = 0; y < meshGen.getHermiteIndexSize(); y++) {
                for (int x = 0; x < meshGen.getHermiteIndexSize(); x++)
                {
                    Vec4i pos = new Vec4i(x, y, z, 0);
                    int index = (x + (y * meshGen.getHermiteIndexSize()) + (z * meshGen.getHermiteIndexSize() * meshGen.getHermiteIndexSize()));
                    int edgeIndex = index * 3;

                    int[] CORNER_MATERIALS = {
                            materials[field_index(pos.add(new Vec4i(0, 0, 0, 0)))],
                            materials[field_index(pos.add(new Vec4i(1, 0, 0, 0)))],
                            materials[field_index(pos.add(new Vec4i(0, 1, 0, 0)))],
                            materials[field_index(pos.add(new Vec4i(0, 0, 1, 0)))],
                    };

                    int voxelIndex = pos.x | (pos.y << meshGen.getIndexShift()) | (pos.z << (meshGen.getIndexShift() * 2));

                    for (int i = 0; i < 3; i++) {
                        int e = 1 + i;
                        boolean signChange =(CORNER_MATERIALS[0] != meshGen.MATERIAL_AIR && CORNER_MATERIALS[e] == meshGen.MATERIAL_AIR) ||
                                            (CORNER_MATERIALS[0] == meshGen.MATERIAL_AIR && CORNER_MATERIALS[e] != meshGen.MATERIAL_AIR);
                        edgeOccupancy[edgeIndex + i] = signChange ? 1 : 0;
                        edgeIndices[edgeIndex + i] = signChange ? ((voxelIndex << 2) | i) : -1;
                        if (signChange)
                            ++size;
                    }
                }
            }
        }
        return size;
    }

    private int[] compactEdges(int[] edgeValid, int[] edges, int compactEdgesSize) {
        int[] compactActiveEdges = new int [compactEdgesSize];
        int current = 0;
        for (int index = 0; index < edges.length; index++) {
            if (edgeValid[index]==1) {
                compactActiveEdges[current] = edges[index];
                ++current;
            }
        }
        return compactActiveEdges;
    }

    Vec3i[] EDGE_END_OFFSETS = {
            new Vec3i(1,0,0),
            new Vec3i(0,1,0),
            new Vec3i(0,0,1)
    };

    private Vec4f[] FindEdgeIntersectionInfo(Vec3i chunkMin, int sampleScale, int from, int to, int[] encodedEdges) {
        Vec4f[] edgeInfo = new Vec4f[encodedEdges.length];
        for (int index = from; index < to; index++) {
            int edge = encodedEdges[index];
            int axisIndex = edge & 3;
            int hermiteIndex = edge >> 2;

            int x = (hermiteIndex >> (meshGen.getIndexShift() * 0)) & meshGen.getIndexMask();
            int y = (hermiteIndex >> (meshGen.getIndexShift() * 1)) & meshGen.getIndexMask();
            int z = (hermiteIndex >> (meshGen.getIndexShift() * 2)) & meshGen.getIndexMask();

            Vec3f p0 = new Vec3i(x, y, z).mul(sampleScale).add(chunkMin).toVec3f();
            Vec3f p1 = p0.add(EDGE_END_OFFSETS[axisIndex].mul(sampleScale).toVec3f());

            Vec4f p = ApproximateLevenCrossingPosition(p0, p1);
            Vec4f normal = CalculateSurfaceNormal(p);

            edgeInfo[index] = new Vec4f(normal.getVec3f(), p.w);
        }
        return edgeInfo;
    }

    private int FindActiveVoxels(int chunkSize, Vec3i chunkMin, int from, int to,
                         int[] materials,
                         int[] voxelOccupancy,
                         int[] voxelEdgeInfo,
                         int[] voxelPositions,
                         int[] voxelMaterials) {
        int size = 0;
        for (int k = from; k < to; k++) {
            int indexShift = VoxelHelperUtils.log2(meshGen.getVoxelsPerChunk()); // max octree depth
            int x = (k >> (indexShift * 0)) & meshGen.getVoxelsPerChunk() - 1;
            int y = (k >> (indexShift * 1)) & meshGen.getVoxelsPerChunk() - 1;
            int z = (k >> (indexShift * 2)) & meshGen.getVoxelsPerChunk() - 1;

            int index = x + (y * meshGen.getVoxelsPerChunk()) + (z * meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk());
            Vec3i pos = new Vec3i(x, y, z);

            int[] cornerMaterials = {
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[0]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[1]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[2]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[3]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[4]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[5]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[6]))],
                    materials[field_index(pos.add(CHILD_MIN_OFFSETS[7]))],
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
            voxelPositions[index] = codeForPosition(pos, meshGen.MAX_OCTREE_DEPTH);
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

    private void createLeafNodes(int from, int to, int[] voxelPositions, int[] voxelEdgeInfo, Vec4f[] edgeDataTable, Map<Integer, Integer> nodes,
                         QEFData[] leafQEFs, Vec4f[] vertexNormals)
    {
        for (int index = from; index < to; index++) {
            int encodedPosition = voxelPositions[index];
            Vec3i position = positionForCode(encodedPosition);
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
                int edgeIndex = (encodeVoxelIndex(hermiteIndexPosition) << 2) | axis;

                Integer dataIndex = nodes.get(edgeIndex);
                if (dataIndex!=null && dataIndex != ~0) {
                    Vec4f edgeData = edgeDataTable[dataIndex];
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

    private void solveQEFs(int[] d_nodeCodes, int chunkSize, int voxelsPerChunk, Vec3i chunkMin, int from, int to,
                           QEFData[] qefs, Vec4f[] solvedPositions){
        for (int index = from; index < to; index++) {
            //Vec3i pos = LinearOctreeTest.positionForCode(d_nodeCodes[index]);
            int leafSize = (chunkSize / voxelsPerChunk);
            //Vec3i leaf = pos.mul(leafSize).add(chunkMin);

            Vec4f solvedPos;
            if(leafSize == meshGen.leafSizeScale) {
                solvedPos = qefs[index].solve();       // run solver only for LOD 0
            } else {
                solvedPos = qefs[index].getMasspoint();// for other LOD's get masspoint - to increase performance
            }

            solvedPos = solvedPos.mul(leafSize).add(chunkMin);

            // clamping
            //Vec4f massPoint = qefs[index].getMasspoint().mul(leafSize).add(chunkMin);
            //solvedPos = VoxelHelperUtils.isOutFromBounds(solvedPos.getVec3f(), leaf.toVec3f(), leafSize) ? massPoint : solvedPos;
            solvedPositions[index] = solvedPos;
        }
    }

    private int findSeamNodes(int[] nodeCodes, int[] isSeamNode, int from, int to) {
        int res = 0;
        for (int index = from; index < to; index++) {
            int code = nodeCodes[index];
            Vec3i position = positionForCode(code);
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

    private int generateMesh(Map<Integer, Integer> nodes, int[] octreeNodeCodes, int[] octreeMaterials,
                             int[] meshIndexBuffer, int[] trianglesValid) {
        int size = 0;
        for (int index = 0; index < octreeNodeCodes.length; index++) {
            int code = octreeNodeCodes[index];
            int triIndex = index * 3;

            Vec3i offset = positionForCode(code);
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
                    int c = codeForPosition(p, meshGen.MAX_OCTREE_DEPTH);
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
                                 List<OctreeNode> seamNodes) {
        for (int index = from; index < to; index++) {
            if (isSeamNode==null || isSeamNode[index]==1) {
                Vec3i min = positionForCode(octreeCodes[index]).mul(leafSize).add(chunkMin);
                PointerBasedOctreeNode node = new PointerBasedOctreeNode(min, leafSize, OctreeNodeType.Node_Leaf);
                node.corners = octreeMaterials[index];
                OctreeDrawInfo drawInfo = new OctreeDrawInfo();
                drawInfo.position = octreePositions[index].getVec3f();
                drawInfo.color = color;
                drawInfo.averageNormal = octreeNormals[index].getVec3f();
                node.drawInfo = drawInfo;
                node.nodeNum = positionForCode(octreeCodes[index]);
                seamNodes.add(node);
            }
        }
    }
}