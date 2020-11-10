package dc.impl.opencl;

import core.math.Vec4f;
import dc.impl.GPUDensityField;
import dc.impl.MeshGenerationContext;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import static org.lwjgl.opencl.CL10.*;

public final class FindDefaultEdgesOpenCLService {
    private BufferGpu edgeOccupancyBuffer, edgeScanBuffer, edgeIndicesNonCompactBuffer;
    private final ScanOpenCLService scanOpenCLService;
    private int edgeBufferSize;
    private final ComputeContext ctx;
    private final MeshGenerationContext meshGen;
    private final GPUDensityField field;
    private final BufferGpuService bufferGpuService;

    public FindDefaultEdgesOpenCLService(ComputeContext computeContext, MeshGenerationContext meshGenerationContext,
                                         GPUDensityField field, ScanOpenCLService scanOpenCLService, BufferGpuService bufferGpuService) {
        this.meshGen = meshGenerationContext;
        this.scanOpenCLService = scanOpenCLService;
        this.ctx = computeContext;
        this.field = field;
        this.bufferGpuService = bufferGpuService;
    }

    public int findFieldEdgesKernel(KernelsHolder kernels, int hermiteIndexSize, int[] edgeOccupancy, int[] edgeIndicesNonCompact) {
        edgeBufferSize = hermiteIndexSize * hermiteIndexSize * hermiteIndexSize * 3;
        edgeOccupancyBuffer = bufferGpuService.create("edgeOccupancyBuffer", edgeBufferSize * 4, CL10.CL_MEM_READ_WRITE);
        edgeIndicesNonCompactBuffer = bufferGpuService.create("edgeIndicesNonCompactBuffer", edgeBufferSize * 4, CL10.CL_MEM_READ_WRITE);

        long findFieldEdgesKernel = clCreateKernel(kernels.getKernel(KernelNames.FIND_DEFAULT_EDGES), "FindFieldEdges", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(findFieldEdgesKernel, 0, field.getMaterials().getMem());
        clSetKernelArg1p(findFieldEdgesKernel, 1, edgeOccupancyBuffer.getMem());
        clSetKernelArg1p(findFieldEdgesKernel, 2, edgeIndicesNonCompactBuffer.getMem());

        final int dimensions = 3;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions);
        globalWorkSize.put(0, hermiteIndexSize);
        globalWorkSize.put(1, hermiteIndexSize);
        globalWorkSize.put(2, hermiteIndexSize);

        // Run the specified number of work units using our OpenCL program kernel
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), findFieldEdgesKernel, dimensions, null, globalWorkSize, null, null, null);
        OCLUtils.checkCLError(err);

        err = CL10.clReleaseKernel(findFieldEdgesKernel);
        OCLUtils.checkCLError(err);

        OCLUtils.getIntBuffer(edgeOccupancyBuffer, edgeOccupancy);
        OCLUtils.getIntBuffer(edgeIndicesNonCompactBuffer, edgeIndicesNonCompact);

        edgeScanBuffer = bufferGpuService.create("edgeScanBuffer", edgeBufferSize * 4, CL_MEM_READ_WRITE);
        int numEdges = scanOpenCLService.exclusiveScan(edgeOccupancyBuffer, edgeScanBuffer, edgeBufferSize);
        field.setNumEdges(numEdges);
        if (field.getNumEdges() <= 0) {
            System.out.println("FindDefaultEdges: ExclusiveScan error=%d\n" + field.getNumEdges());
            bufferGpuService.releaseAll();
            return -1;
        }
        return field.getNumEdges();
    }

    public void compactEdgeKernel(KernelsHolder kernels, GPUDensityField field){
        BufferGpu compactActiveEdgesBuffer = bufferGpuService.create("compactActiveEdgesBuffer", field.getNumEdges() * 4, CL_MEM_READ_WRITE);
        long compactEdgesKernel = clCreateKernel(kernels.getKernel(KernelNames.FIND_DEFAULT_EDGES), "CompactEdges", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        clSetKernelArg1p(compactEdgesKernel, 0, edgeOccupancyBuffer.getMem());
        clSetKernelArg1p(compactEdgesKernel, 1, edgeScanBuffer.getMem());
        clSetKernelArg1p(compactEdgesKernel, 2, edgeIndicesNonCompactBuffer.getMem());
        clSetKernelArg1p(compactEdgesKernel, 3, compactActiveEdgesBuffer.getMem());

        PointerBuffer globalWorkEdgeBufferSize = BufferUtils.createPointerBuffer(1);
        globalWorkEdgeBufferSize.put(0, edgeBufferSize);
        int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), compactEdgesKernel, 1, null, globalWorkEdgeBufferSize, null, null, null);
        OCLUtils.checkCLError(errcode);
        field.setEdgeIndices(compactActiveEdgesBuffer);
        //int[] compact = OCLUtils.getIntBuffer(field.getEdgeIndices(), field.getNumEdges());

        int err = CL10.clReleaseKernel(compactEdgesKernel);
        OCLUtils.checkCLError(err);

        bufferGpuService.release(edgeScanBuffer);
        bufferGpuService.release(edgeOccupancyBuffer);
        bufferGpuService.release(edgeIndicesNonCompactBuffer);
    }

    public void FindEdgeIntersectionInfoKernel(KernelsHolder kernels, Vec4f[] normals) {
        field.setNormals(bufferGpuService.create("fieldNormals", field.getNumEdges() * 4 * 4, CL_MEM_WRITE_ONLY));

        int sampleScale = field.getSize() / (meshGen.getVoxelsPerChunk() * meshGen.leafSizeScale);
        long kFindInfoKernel = clCreateKernel(kernels.getKernel(KernelNames.FIND_DEFAULT_EDGES), "FindEdgeIntersectionInfo", ctx.getErrcode_ret());
        clSetKernelArg4i(kFindInfoKernel, 0,
                field.getMin().x/meshGen.leafSizeScale,
                field.getMin().y/meshGen.leafSizeScale,
                field.getMin().z/meshGen.leafSizeScale,
                0);
        clSetKernelArg1i(kFindInfoKernel, 1, sampleScale);
        clSetKernelArg1p(kFindInfoKernel, 2, field.getEdgeIndices().getMem());
        clSetKernelArg1p(kFindInfoKernel, 3, field.getNormals().getMem());

        PointerBuffer globalWorkNumEdgesSize = BufferUtils.createPointerBuffer(1);
        globalWorkNumEdgesSize.put(0, field.getNumEdges());
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), kFindInfoKernel, 1, null, globalWorkNumEdgesSize, null, null, null);
        OCLUtils.checkCLError(err);

        OCLUtils.getNormals(field.getNormals(), normals);

        CL10.clReleaseKernel(kFindInfoKernel);
        CL10.clFinish(ctx.getClQueue());
    }
}