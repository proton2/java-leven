package dc.impl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.math.Vec4i;
import core.utils.BufferUtil;
import core.utils.Constants;
import dc.*;
import dc.entities.MeshBuffer;
import dc.entities.MeshVertex;
import dc.impl.opencl.*;
import dc.solver.LevenQefSolver;
import dc.solver.QefSolver;
import dc.utils.VoxelHelperUtils;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/*
    Nick Gildea Leven OpenCL kernels Dual contouring implementation translated to java calling OpenCL kernels
    Some holes in seams is not fixed.
    The first raw version will still improve.
 */

public class LevenLinearOpenCLOctreeImpl extends AbstractDualContouring implements VoxelOctree {
    private final KernelsHolder kernels;
    private final ComputeContext ctx;

    public LevenLinearOpenCLOctreeImpl(KernelsHolder kernels, MeshGenerationContext meshGenerationContext, ComputeContext ctx) {
        super(meshGenerationContext);
        this.kernels = kernels;
        this.ctx = ctx;
    }

    @Override
    public boolean createLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                        float[] densityField,
                                        List<PointerBasedOctreeNode> seamNodes, MeshBuffer buffer, GPUDensityField field)
    {
        field.setMin(chunkMin);
        field.setSize(chunkSize);
        OpenCLCalculateMaterialsService calculateMaterialsService = new OpenCLCalculateMaterialsService(ctx, meshGen.getFieldSize(), meshGen, field);
        calculateMaterialsService.run(kernels, null);

        ScanOpenCLService scanService = new ScanOpenCLService(ctx, kernels.getKernel(KernelNames.SCAN));
        FindDefaultEdgesOpenCLService findDefEdges = new FindDefaultEdgesOpenCLService(ctx, meshGen, field, scanService);
        int compactEdgesSize = findDefEdges.findFieldEdgesKernel(kernels, meshGen.getHermiteIndexSize(), null, null);
        if(compactEdgesSize<=0){
            return false;
        }
        findDefEdges.compactEdgeKernel(kernels, field);
        findDefEdges.FindEdgeIntersectionInfoKernel(kernels, null);

        GpuOctree gpuOctree = new GpuOctree();
        ConstructOctreeFromFieldService constructOctreeFromFieldService = new ConstructOctreeFromFieldService(ctx, meshGen, field, gpuOctree, scanService);
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
                meshGen, scanService, gpuOctree, meshBufferGPU);
        int numTriangles = generateMeshFromOctreeService.generateMeshKernel(kernels, null, null);
        if(numTriangles<=0){
            return false;
        }

        int[] d_compactIndexBuffer = new int[numTriangles * 3];
        generateMeshFromOctreeService.compactMeshTrianglesKernel(kernels, numTriangles, d_compactIndexBuffer);

        buffer.setNumVertices(octreeNumNodes);
        buffer.setNumIndicates(numTriangles * 3);
        buffer.setIndicates(BufferUtil.createFlippedBuffer(d_compactIndexBuffer));

        generateMeshFromOctreeService.run(kernels, field.getSize());
        generateMeshFromOctreeService.exportMeshBuffer(meshBufferGPU, buffer);

        int[] isSeamNode = new int[octreeNumNodes];
        int numSeamNodes = generateMeshFromOctreeService.findSeamNodesKernel(kernels, isSeamNode);
        generateMeshFromOctreeService.gatherSeamNodesFromOctree(kernels, chunkMin, chunkSize/meshGen.getVoxelsPerChunk(), seamNodes, numSeamNodes);

        findDefEdges.destroy();
        calculateMaterialsService.destroy();
        return true;
    }
}
