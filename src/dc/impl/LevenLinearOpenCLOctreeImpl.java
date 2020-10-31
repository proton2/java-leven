package dc.impl;

import core.math.Vec3i;
import dc.AbstractDualContouring;
import dc.PointerBasedOctreeNode;
import dc.VoxelOctree;
import dc.entities.MeshBuffer;
import dc.impl.opencl.*;

import java.util.List;

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
            generateMeshFromOctreeService.gatherSeamNodesFromOctree(kernels, chunkMin, chunkSize, seamNodes);

            generateMeshFromOctreeService.destroy();
        }

        findDefEdges.destroy();
        return true;
    }
}
