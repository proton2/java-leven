package dc.impl.nonused.opencl;

import core.math.Vec4f;
import dc.impl.MeshGenerationContext;
import dc.impl.nonused.GPUDensityField;
import dc.impl.nonused.GpuOctree;
import dc.solver.QEFData;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import static org.lwjgl.opencl.CL10.*;

public class ConstructOctreeFromFieldService {
    private final MeshGenerationContext meshGen;
    private final GPUDensityField field;
    private final GpuOctree octree;
    private final ComputeContext ctx;
    private final ScanOpenCLService scanService;
    private BufferGpu leafEdgeInfoBuf, leafCodesBuf, leafMaterialsBuf, compactLeafEdgeInfoBuf, leafOccupancyBuf, voxelScanBuf, d_qefsBuf;
    private final BufferGpuService bufferGpuService;

    public ConstructOctreeFromFieldService(ComputeContext computeContext, MeshGenerationContext mesGen,
                                           GPUDensityField densityField, GpuOctree gpuOctree, ScanOpenCLService scanService, BufferGpuService bufferGpuService) {
        this.ctx = computeContext;
        this.meshGen = mesGen;
        this.field = densityField;
        this.octree = gpuOctree;
        this.scanService = scanService;
        this.bufferGpuService = bufferGpuService;
    }

    public int findActiveVoxelsKernel(KernelsHolder kernels,
                                      int[] d_leafOccupancy, int[] d_leafEdgeInfo, int[] d_leafCodes, int[] d_leafMaterials){

        int chunkBufferSize = meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk();

        leafOccupancyBuf = bufferGpuService.create("leafOccupancyBuf", chunkBufferSize * 4, CL10.CL_MEM_READ_WRITE);
        leafEdgeInfoBuf = bufferGpuService.create("leafEdgeInfoBuf", chunkBufferSize * 4, CL_MEM_READ_WRITE);
        leafCodesBuf = bufferGpuService.create("leafCodesBuf", chunkBufferSize * 4, CL10.CL_MEM_READ_WRITE);
        leafMaterialsBuf = bufferGpuService.create("leafMaterialsBuf", chunkBufferSize * 4, CL10.CL_MEM_READ_WRITE);
        voxelScanBuf = bufferGpuService.create("voxelScanBuf", chunkBufferSize * 4, CL10.CL_MEM_READ_WRITE);

        long findActiveKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "FindActiveVoxels", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(findActiveKernel, 0, field.getMaterials().getMem());
        clSetKernelArg1p(findActiveKernel, 1, leafOccupancyBuf.getMem());
        clSetKernelArg1p(findActiveKernel, 2, leafEdgeInfoBuf.getMem());
        clSetKernelArg1p(findActiveKernel, 3, leafCodesBuf.getMem());
        clSetKernelArg1p(findActiveKernel, 4, leafMaterialsBuf.getMem());// ToDo check FindDominantMaterial

        final int dimensions = 3;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions);
        globalWorkSize.put(0, meshGen.getVoxelsPerChunk());
        globalWorkSize.put(1, meshGen.getVoxelsPerChunk());
        globalWorkSize.put(2, meshGen.getVoxelsPerChunk());

        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), findActiveKernel, dimensions, null, globalWorkSize, null, null, null);
        OCLUtils.checkCLError(err);

        bufferGpuService.release(field.getMaterials());

        int numNodes = scanService.exclusiveScan(leafOccupancyBuf, voxelScanBuf, chunkBufferSize);
        octree.setNumNodes(numNodes);
        if(numNodes<=0){
            //System.out.println("no voxels");
            bufferGpuService.releaseAll();
            return -1;
        }
        OCLUtils.getIntBuffer(leafOccupancyBuf, d_leafOccupancy);
        OCLUtils.getIntBuffer(leafEdgeInfoBuf, d_leafEdgeInfo);
        OCLUtils.getIntBuffer(leafCodesBuf, d_leafCodes);
        OCLUtils.getIntBuffer(leafMaterialsBuf, d_leafMaterials);

        err = CL10.clReleaseKernel(findActiveKernel);
        OCLUtils.checkCLError(err);
        return numNodes;
    }

    public void compactVoxelsKernel(KernelsHolder kernels, int[] nodeCodesBuf, int[] nodeMaterialsBuf){
        int chunkBufferSize = meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk() * meshGen.getVoxelsPerChunk();

        compactLeafEdgeInfoBuf = bufferGpuService.create("compactLeafEdgeInfoBuf", octree.getNumNodes() * Integer.BYTES, CL10.CL_MEM_READ_WRITE);
        octree.setNodeCodesBuf(bufferGpuService.create("nodeCodeBuf", octree.getNumNodes() * Integer.BYTES, CL10.CL_MEM_READ_WRITE));
        octree.setNodeMaterialsBuf(bufferGpuService.create("nodematerialsBuf", octree.getNumNodes() * Integer.BYTES, CL_MEM_READ_WRITE));
        octree.setVertexPositionsBuf(bufferGpuService.create("vertexPositionsBuf", octree.getNumNodes() * Integer.BYTES * 4, CL10.CL_MEM_READ_WRITE));
        octree.setVertexNormalsBuf(bufferGpuService.create("vertexNormalsBuf", octree.getNumNodes() * Integer.BYTES * 4, CL10.CL_MEM_READ_WRITE));

        long compactVoxelsKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "CompactVoxels", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(compactVoxelsKernel, 0, leafOccupancyBuf.getMem());
        clSetKernelArg1p(compactVoxelsKernel, 1, leafEdgeInfoBuf.getMem());
        clSetKernelArg1p(compactVoxelsKernel, 2, leafCodesBuf.getMem());
        clSetKernelArg1p(compactVoxelsKernel, 3, leafMaterialsBuf.getMem());
        clSetKernelArg1p(compactVoxelsKernel, 4, voxelScanBuf.getMem());
        clSetKernelArg1p(compactVoxelsKernel, 5, octree.getNodeCodesBuf().getMem());
        clSetKernelArg1p(compactVoxelsKernel, 6, compactLeafEdgeInfoBuf.getMem());
        clSetKernelArg1p(compactVoxelsKernel, 7, octree.getNodeMaterialsBuf().getMem());

        PointerBuffer compactVoxelsWorkSize = BufferUtils.createPointerBuffer(1);
        compactVoxelsWorkSize.put(0, chunkBufferSize);
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), compactVoxelsKernel, 1, null, compactVoxelsWorkSize, null, null, null);
        OCLUtils.checkCLError(err);

        OCLUtils.getIntBuffer(octree.getNodeCodesBuf(), nodeCodesBuf);
//        OCLUtils.getIntBuffer(compactLeafEdgeInfoBuf, compactLeafEdgeInfo);
        OCLUtils.getIntBuffer(octree.getNodeMaterialsBuf(), nodeMaterialsBuf);

        err = CL10.clReleaseKernel(compactVoxelsKernel);
        OCLUtils.checkCLError(err);
    }

    public void createLeafNodesKernel(KernelsHolder kernels, QEFData[] qefs, Vec4f[] d_vertexNormals){
        d_qefsBuf = bufferGpuService.create("d_qefsBuf", octree.getNumNodes() * Float.BYTES * 16, CL10.CL_MEM_READ_WRITE);
        CuckooHashOpenCLService edgeHashTable = new CuckooHashOpenCLService(ctx, meshGen, scanService, kernels, field.getNumEdges(), bufferGpuService);
        edgeHashTable.insertKeys(field.getEdgeIndices(), field.getNumEdges());

        int sampleScale = field.getSize() / (meshGen.getVoxelsPerChunk() * meshGen.leafSizeScale);
        long createLeafNodesKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "CreateLeafNodes", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        clSetKernelArg1i(createLeafNodesKernel, 0, sampleScale);
        clSetKernelArg1p(createLeafNodesKernel, 1, octree.getNodeCodesBuf().getMem());
        clSetKernelArg1p(createLeafNodesKernel, 2, compactLeafEdgeInfoBuf.getMem());
        clSetKernelArg1p(createLeafNodesKernel, 3, field.getNormals().getMem());
        clSetKernelArg1p(createLeafNodesKernel, 4, octree.getVertexNormalsBuf().getMem());
        clSetKernelArg1p(createLeafNodesKernel, 5, d_qefsBuf.getMem());
        clSetKernelArg1p(createLeafNodesKernel, 6, edgeHashTable.getTable().getMem());
        clSetKernelArg1p(createLeafNodesKernel, 7, edgeHashTable.getStash().getMem());
        clSetKernelArg1i(createLeafNodesKernel, 8, edgeHashTable.getPrime());
        clSetKernelArg1p(createLeafNodesKernel, 9, edgeHashTable.getHashParams().getMem());
        clSetKernelArg1i(createLeafNodesKernel, 10, edgeHashTable.getStashUsed());

        PointerBuffer createLeafNodesWorkSize = BufferUtils.createPointerBuffer(1);
        createLeafNodesWorkSize.put(0, octree.getNumNodes());
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), createLeafNodesKernel, 1, null, createLeafNodesWorkSize, null, null, null);
        OCLUtils.checkCLError(err);

        OCLUtils.getQEFData(d_qefsBuf, qefs);
        OCLUtils.getNormals(octree.getVertexNormalsBuf(), d_vertexNormals);

        err = CL10.clReleaseKernel(createLeafNodesKernel);
        OCLUtils.checkCLError(err);
        bufferGpuService.release(field.getEdgeIndices());
        bufferGpuService.release(compactLeafEdgeInfoBuf);
        bufferGpuService.release(field.getNormals());
        edgeHashTable.destroy();
    }

    public int solveQefKernel(KernelsHolder kernels, Vec4f[] vertexPositions) {
        long solveQEFsKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "SolveQEFs", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg4f(solveQEFsKernel, 0, field.getMin().x, field.getMin().y, field.getMin().z, 0);
        clSetKernelArg1p(solveQEFsKernel, 1, d_qefsBuf.getMem());
        clSetKernelArg1p(solveQEFsKernel, 2, octree.getVertexPositionsBuf().getMem());

        PointerBuffer solveQEFsWorkSize = BufferUtils.createPointerBuffer(1);
        solveQEFsWorkSize.put(0, octree.getNumNodes());
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), solveQEFsKernel, 1, null, solveQEFsWorkSize, null, null, null);
        OCLUtils.checkCLError(err);
        OCLUtils.getNormals(octree.getVertexPositionsBuf(), vertexPositions);

        err = CL10.clReleaseKernel(solveQEFsKernel);
        OCLUtils.checkCLError(err);

        bufferGpuService.release(d_qefsBuf);
        return CL_SUCCESS;
    }
}