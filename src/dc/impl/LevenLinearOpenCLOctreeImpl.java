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
import dc.solver.GlslSvd;
import dc.solver.QefSolver;
import dc.utils.VoxelHelperUtils;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/*
    Nick Gildea Leven OpenCL kernels Dual contouring implementation translated to java
    Some holes in seams is not fixed.
    The first raw version will still improve.
 */

public class LevenLinearOpenCLOctreeImpl extends AbstractDualContouring implements VoxelOctree {
    private final KernelsHolder kernels;

    public LevenLinearOpenCLOctreeImpl(KernelsHolder kernels, MeshGenerationContext meshGenerationContext) {
        super(meshGenerationContext);
        this.kernels = kernels;
    }

    @Override
    public boolean createLeafVoxelNodes(int chunkSize, Vec3i chunkMin,
                                        float[] densityField,
                                        List<PointerBasedOctreeNode> seamNodes, MeshBuffer buffer, GPUDensityField field)
    {
        int[] materials = new int[meshGen.getFieldSize()*meshGen.getFieldSize()*meshGen.getFieldSize()];

        field.setMin(chunkMin);
        field.setSize(chunkSize);
        ComputeContext ctx = OCLUtils.getOpenCLContext();
        OpenCLCalculateMaterialsService calculateMaterialsService = new OpenCLCalculateMaterialsService(ctx, meshGen.getFieldSize(), meshGen);
        calculateMaterialsService.run(kernels, materials, field);

        ScanOpenCLService scanService = new ScanOpenCLService(ctx, kernels.getKernel(KernelNames.SCAN));
        FindDefaultEdgesOpenCLService findDefEdges = new FindDefaultEdgesOpenCLService(ctx, scanService, meshGen);
        int compactEdgesSize = findDefEdges.run(kernels, field, meshGen.getVoxelsPerChunk(), meshGen.getHermiteIndexSize());
        if(compactEdgesSize==0){
            return false;
        }

        GpuOctree gpuOctree = new GpuOctree();
        ConstructOctreeFromFieldService constructOctreeFromFieldService = new ConstructOctreeFromFieldService(ctx,
                meshGen, field, gpuOctree, scanService);
        constructOctreeFromFieldService.run(kernels);

        if (gpuOctree.getNumNodes() > 0) {
            MeshBufferGPU meshBufferGPU = new MeshBufferGPU();
            GenerateMeshFromOctreeService generateMeshFromOctreeService = new GenerateMeshFromOctreeService(ctx,
                    meshGen, scanService, gpuOctree, meshBufferGPU);
            generateMeshFromOctreeService.run(kernels, field.getSize());
            generateMeshFromOctreeService.exportMeshBuffer(meshBufferGPU, buffer);

            generateMeshFromOctreeService.destroy();
        }

        findDefEdges.destroy();
        return true;
    }
}
