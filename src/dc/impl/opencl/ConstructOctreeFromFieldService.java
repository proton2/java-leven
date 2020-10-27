package dc.impl.opencl;

import dc.impl.GPUDensityField;
import dc.impl.GpuOctree;
import dc.impl.MeshGenerationContext;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import static org.lwjgl.opencl.CL10.*;
import static org.lwjgl.opencl.CL10.clSetKernelArg1p;

public class ConstructOctreeFromFieldService {
    private final MeshGenerationContext meshGen;
    private final GPUDensityField field;
    private final GpuOctree octree;
    private final ComputeContext ctx;
    private final ScanOpenCLService scanService;



    public ConstructOctreeFromFieldService(ComputeContext computeContext, MeshGenerationContext mesGen,
                                           GPUDensityField densityField, GpuOctree gpuOctree, ScanOpenCLService scanService) {
        this.ctx = computeContext;
        this.meshGen = mesGen;
        this.field = densityField;
        this.octree = gpuOctree;
        this.scanService = scanService;
    }

    public int run(KernelsHolder kernels) {
        int chunkBufferSize = meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk();

        long leafOccupancyBuf = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, chunkBufferSize * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        long leafEdgeInfoBuf = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, chunkBufferSize * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        long leafCodesBuf = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, chunkBufferSize * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        long leafMaterialsBuf = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, chunkBufferSize * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        long voxelScanBuf = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, chunkBufferSize * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        long findActiveKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "FindActiveVoxels", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(findActiveKernel, 0, field.getMaterials());
        clSetKernelArg1p(findActiveKernel, 1, leafOccupancyBuf);
        clSetKernelArg1p(findActiveKernel, 2, leafEdgeInfoBuf);
        clSetKernelArg1p(findActiveKernel, 3, leafCodesBuf);
        clSetKernelArg1p(findActiveKernel, 4, leafMaterialsBuf);// ToDo check FindDominantMaterial

        final int dimensions = 3;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions);
        globalWorkSize.put(0, meshGen.getVoxelsPerChunk());
        globalWorkSize.put(1, meshGen.getVoxelsPerChunk());
        globalWorkSize.put(2, meshGen.getVoxelsPerChunk());

        int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), findActiveKernel, dimensions, null, globalWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);

        int numNodes = scanService.exclusiveScan(leafOccupancyBuf, voxelScanBuf, chunkBufferSize);
        if(numNodes<=0){
            octree.setNumNodes(0);
            System.out.println("no voxels");
            return numNodes;
        }
        octree.setNumNodes(numNodes);


        long compactLeafEdgeInfoBuf = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, octree.getNumNodes() * Integer.BYTES, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        octree.setNodeCodesBuf(CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, octree.getNumNodes() * Integer.BYTES, ctx.getErrcode_ret()));
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        octree.setNodeMaterialsBuf(CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, octree.getNumNodes() * Integer.BYTES, ctx.getErrcode_ret()));
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        octree.setVertexPositionsBuf(CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, octree.getNumNodes() * Integer.BYTES, ctx.getErrcode_ret()));
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        octree.setVertexNormalsBuf(CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, octree.getNumNodes() * Integer.BYTES, ctx.getErrcode_ret()));

        long compactVoxelsKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "CompactVoxels", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(compactVoxelsKernel, 0, leafOccupancyBuf);
        clSetKernelArg1p(compactVoxelsKernel, 1, leafEdgeInfoBuf);
        clSetKernelArg1p(compactVoxelsKernel, 2, leafCodesBuf);
        clSetKernelArg1p(compactVoxelsKernel, 3, leafMaterialsBuf);
        clSetKernelArg1p(compactVoxelsKernel, 4, voxelScanBuf);
        clSetKernelArg1p(compactVoxelsKernel, 5, octree.getNodeCodesBuf());
        clSetKernelArg1p(compactVoxelsKernel, 6, compactLeafEdgeInfoBuf);
        clSetKernelArg1p(compactVoxelsKernel, 7, octree.getNodeMaterialsBuf());

        PointerBuffer compactVoxelsWorkSize = BufferUtils.createPointerBuffer(1);
        compactVoxelsWorkSize.put(0, chunkBufferSize);
        errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), compactVoxelsKernel, 1, null, compactVoxelsWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);

//        int[] result1 = OCLUtils.getIntBuffer(octree.getNodeCodesBuf(), octree.getNumNodes());
//        int[] result2 = OCLUtils.getIntBuffer(compactLeafEdgeInfoBuf, octree.getNumNodes());
//        int[] result3 = OCLUtils.getIntBuffer(octree.getNodeMaterialsBuf(), octree.getNumNodes());

        long d_qefsBuf = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, octree.getNumNodes() * Float.BYTES * 16, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        CuckooHashOpenCLService edgeHashTable = new CuckooHashOpenCLService(ctx, meshGen, scanService, kernels, field.getNumEdges());
        edgeHashTable.insertKeys(field.getEdgeIndices(), field.getNumEdges());

        int sampleScale = field.getSize() / (meshGen.getVoxelsPerChunk() * meshGen.leafSizeScale);
        long createLeafNodesKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "CreateLeafNodes", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        clSetKernelArg1i(createLeafNodesKernel, 0, sampleScale);
        clSetKernelArg1p(createLeafNodesKernel, 1, octree.getNodeCodesBuf());
        clSetKernelArg1p(createLeafNodesKernel, 2, compactLeafEdgeInfoBuf);
        clSetKernelArg1p(createLeafNodesKernel, 3, field.getNormals());
        clSetKernelArg1p(createLeafNodesKernel, 4, octree.getVertexNormalsBuf());
        clSetKernelArg1p(createLeafNodesKernel, 5, d_qefsBuf);
        clSetKernelArg1p(createLeafNodesKernel, 6, edgeHashTable.getTable());
        clSetKernelArg1p(createLeafNodesKernel, 7, edgeHashTable.getStash());
        clSetKernelArg1i(createLeafNodesKernel, 8, edgeHashTable.getPrime());
        clSetKernelArg1p(createLeafNodesKernel, 9, edgeHashTable.getHashParams());
        clSetKernelArg1i(createLeafNodesKernel, 10, edgeHashTable.getStashUsed());

        PointerBuffer createLeafNodesWorkSize = BufferUtils.createPointerBuffer(1);
        createLeafNodesWorkSize.put(0, octree.getNumNodes());
        errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), createLeafNodesKernel, 1, null, createLeafNodesWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);

        //OCLUtils.QEFData[] qefData = OCLUtils.getQEFData(d_qefsBuf, octree.getNumNodes());

        long solveQEFsKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "SolveQEFs", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg4i(solveQEFsKernel, 0, field.getMin().x, field.getMin().y, field.getMin().z, 0);
        clSetKernelArg1p(solveQEFsKernel, 1, d_qefsBuf);
        clSetKernelArg1p(solveQEFsKernel, 2, octree.getVertexPositionsBuf());

        PointerBuffer solveQEFsWorkSize = BufferUtils.createPointerBuffer(1);
        solveQEFsWorkSize.put(0, octree.getNumNodes());
        errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), solveQEFsKernel, 1, null, solveQEFsWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);

        CuckooHashOpenCLService octreeHashTable = new CuckooHashOpenCLService(ctx, meshGen, scanService, kernels, octree.getNumNodes());
        octreeHashTable.insertKeys(octree.getNodeCodesBuf(), octree.getNumNodes());
        octree.setHashTable(octreeHashTable);

        int err = CL10.clReleaseKernel(findActiveKernel);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseKernel(compactVoxelsKernel);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseKernel(createLeafNodesKernel);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseKernel(solveQEFsKernel);
        OCLUtils.checkCLError(err);

        err = CL10.clReleaseMemObject(leafOccupancyBuf);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(leafEdgeInfoBuf);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(leafCodesBuf);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(leafMaterialsBuf);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(voxelScanBuf);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(compactLeafEdgeInfoBuf);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(d_qefsBuf);
        OCLUtils.checkCLError(err);

        return CL_SUCCESS;
    }
}