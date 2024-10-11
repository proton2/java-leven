package dc.impl.notused.gpu;

import core.math.Vec3i;
import core.math.Vec4f;
import core.math.Vec4i;
import core.utils.BufferUtil;
import dc.AbstractDualContouring;
import dc.ChunkNode;
import dc.OctreeNode;
import dc.VoxelOctree;
import dc.csg.ICSGOperations;
import dc.entities.MeshBuffer;
import dc.impl.MeshGenerationContext;
import dc.impl.notused.gpu.opencl.*;

import java.util.List;
import java.util.Map;

/*
    Nick Gildea Leven OpenCL kernels Dual contouring implementation translated to java calling OpenCL kernels
    Some holes in seams is not fixed.
    The first raw version will still improve.
 */

public class LevenLinearGPUOctreeImpl extends AbstractDualContouring implements VoxelOctree {
    private final KernelsHolder kernels;
    private final ComputeContext ctx;
    protected Map<Vec4i, GpuOctree> octreeCache;

    public LevenLinearGPUOctreeImpl(KernelsHolder kernels, MeshGenerationContext meshGenerationContext,
                                    ComputeContext ctx, ICSGOperations csgOperations, Map<Vec4i, GpuOctree> octreeCache) {
        super(meshGenerationContext, csgOperations);
        this.kernels = kernels;
        this.ctx = ctx;
        this.octreeCache = octreeCache;
    }

    @Override
    public void computeFreeChunkOctree(Vec3i min, int clipmapNodeSize) {
        Vec4i key = new Vec4i(min, clipmapNodeSize);
        octreeCache.remove(key);
    }

    @Override
    public boolean createLeafVoxelNodes(ChunkNode node, List<OctreeNode> seamNodes, MeshBuffer buffer) {
        GPUDensityField field = new GPUDensityField();
        field.setMin(node.min);
        field.setSize(node.size);
        BufferGpuService bufferGpuService = new BufferGpuService(ctx);
        OpenCLCalculateMaterialsService calculateMaterialsService = new OpenCLCalculateMaterialsService(ctx, meshGen.getFieldSize(),
                meshGen, field, bufferGpuService);
        calculateMaterialsService.run(kernels, null);

        ScanOpenCLService scanService = new ScanOpenCLService(ctx, kernels.getKernel(KernelNames.SCAN), bufferGpuService);
        FindDefaultEdgesOpenCLService findDefEdges = new FindDefaultEdgesOpenCLService(ctx, meshGen, field, scanService, bufferGpuService);
        int compactEdgesSize = findDefEdges.findFieldEdgesKernel(kernels, meshGen.getHermiteIndexSize(), null, null);
        if(compactEdgesSize<=0){
            return false;
        }
        findDefEdges.compactEdgeKernel(kernels, field);
        findDefEdges.FindEdgeIntersectionInfoKernel(kernels, null);

        GpuOctree gpuOctree = new GpuOctree();
        ConstructOctreeFromFieldService constructOctreeFromFieldService = new ConstructOctreeFromFieldService(ctx, meshGen,
                field, gpuOctree, scanService, bufferGpuService);
        int octreeNumNodes = constructOctreeFromFieldService.findActiveVoxelsKernel(kernels, null, null, null, null);
        if (octreeNumNodes<=0){
            return false;
        }

        int[] d_nodeCodes = new int[octreeNumNodes];
        int[] d_nodeMaterials = new int[octreeNumNodes];
        constructOctreeFromFieldService.compactVoxelsKernel(kernels, d_nodeCodes, d_nodeMaterials);
        constructOctreeFromFieldService.createLeafNodesKernel(kernels, null, null);
        Vec4f[] d_vertexPositions = new Vec4f[octreeNumNodes];
        constructOctreeFromFieldService.solveQefKernel(kernels, d_vertexPositions);

        //////////////////////////////
        MeshBufferGPU meshBufferGPU = new MeshBufferGPU();
        GenerateMeshFromOctreeService generateMeshFromOctreeService = new GenerateMeshFromOctreeService(ctx,
                meshGen, scanService, gpuOctree, meshBufferGPU, field, bufferGpuService);
        int numTriangles = generateMeshFromOctreeService.generateMeshKernel(kernels, null, null);
        if(numTriangles<0){
            bufferGpuService.releaseAll();
            return false;
        }

        if(numTriangles>0) {
            int[] d_compactIndexBuffer = new int[numTriangles * 3];
            generateMeshFromOctreeService.compactMeshTrianglesKernel(kernels, numTriangles, d_compactIndexBuffer);
            buffer.setNumVertices(octreeNumNodes);
            buffer.setNumIndicates(numTriangles * 3);
            buffer.setIndicates(BufferUtil.createFlippedBuffer(d_compactIndexBuffer));
            generateMeshFromOctreeService.run(kernels, field.getSize());
            generateMeshFromOctreeService.exportMeshBuffer(meshBufferGPU, buffer);
        }

        int[] isSeamNode = new int[octreeNumNodes];
        int numSeamNodes = generateMeshFromOctreeService.findSeamNodesKernel(kernels, isSeamNode);
        if(numSeamNodes<=0){
            return false;
        }
        generateMeshFromOctreeService.gatherSeamNodesFromOctree(kernels, node.min, node.size/meshGen.getVoxelsPerChunk(), seamNodes, numSeamNodes);
        bufferGpuService.releaseAll();

//        Map<Vec3i, OctreeNode> seamNodesMap = new HashMap<>();
//        for (OctreeNode seamNode : seamNodes) {
//            if (seamNode.size > meshGen.leafSizeScale) {
//                seamNodesMap.put(seamNode.min, seamNode);
//            }
//        }
//        List<OctreeNode> addedNodes = findAndCreateBorderNodes(seamNodesMap);
//        seamNodes.addAll(addedNodes);
        return true;
    }
}
