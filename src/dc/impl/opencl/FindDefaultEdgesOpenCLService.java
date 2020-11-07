package dc.impl.opencl;

import core.math.Vec4f;
import dc.impl.GPUDensityField;
import dc.impl.MeshGenerationContext;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import java.nio.IntBuffer;

import static org.lwjgl.opencl.CL10.*;

public final class FindDefaultEdgesOpenCLService {
    private long edgeOccupancyBuffer, edgeIndicesNonCompactBuffer;
    private long edgeScanBuffer, compactActiveEdgesBuffer;
    private final ScanOpenCLService scanOpenCLService;
    private int numEdges, edgeBufferSize;
    private ComputeContext ctx;
    private final MeshGenerationContext meshGen;
    private final GPUDensityField field;

    public FindDefaultEdgesOpenCLService(ComputeContext computeContext, MeshGenerationContext meshGenerationContext, GPUDensityField field, ScanOpenCLService scanOpenCLService) {
        this.meshGen = meshGenerationContext;
        this.scanOpenCLService = scanOpenCLService;
        this.ctx = computeContext;
        this.field = field;
    }

    public int findFieldEdgesKernel(KernelsHolder kernels, int hermiteIndexSize, int[] edgeOccupancy, int[] edgeIndicesNonCompact) {
        edgeBufferSize = hermiteIndexSize * hermiteIndexSize * hermiteIndexSize * 3;
        edgeOccupancyBuffer = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, edgeBufferSize * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        edgeIndicesNonCompactBuffer = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, edgeBufferSize * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        long findFieldEdgesKernel = clCreateKernel(kernels.getKernel(KernelNames.FIND_DEFAULT_EDGES), "FindFieldEdges", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(findFieldEdgesKernel, 0, field.getMaterials());
        clSetKernelArg1p(findFieldEdgesKernel, 1, edgeOccupancyBuffer);
        clSetKernelArg1p(findFieldEdgesKernel, 2, edgeIndicesNonCompactBuffer);

        final int dimensions = 3;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions);
        globalWorkSize.put(0, hermiteIndexSize);
        globalWorkSize.put(1, hermiteIndexSize);
        globalWorkSize.put(2, hermiteIndexSize);

        // Run the specified number of work units using our OpenCL program kernel
        int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), findFieldEdgesKernel, dimensions, null, globalWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);

        OCLUtils.getIntBuffer(edgeOccupancyBuffer, edgeOccupancy);
        OCLUtils.getIntBuffer(edgeIndicesNonCompactBuffer, edgeIndicesNonCompact);

        edgeScanBuffer = CL10.clCreateBuffer(ctx.getClContext(), CL_MEM_READ_WRITE, edgeBufferSize * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        numEdges = scanOpenCLService.exclusiveScan(edgeOccupancyBuffer, edgeScanBuffer, edgeBufferSize);
        field.setNumEdges(numEdges);
        if (field.getNumEdges() <= 0) {
            System.out.println("FindDefaultEdges: ExclusiveScan error=%d\n" + field.getNumEdges());
            return -1;
        }
        CL10.clReleaseKernel(findFieldEdgesKernel);
        return field.getNumEdges();
    }

    public void compactEdgeKernel(KernelsHolder kernels, GPUDensityField field){
        compactActiveEdgesBuffer = CL10.clCreateBuffer(ctx.getClContext(), CL_MEM_READ_WRITE, field.getNumEdges() * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        long compactEdgesKernel = clCreateKernel(kernels.getKernel(KernelNames.FIND_DEFAULT_EDGES), "CompactEdges", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        clSetKernelArg1p(compactEdgesKernel, 0, edgeOccupancyBuffer);
        clSetKernelArg1p(compactEdgesKernel, 1, edgeScanBuffer);
        clSetKernelArg1p(compactEdgesKernel, 2, edgeIndicesNonCompactBuffer);
        clSetKernelArg1p(compactEdgesKernel, 3, compactActiveEdgesBuffer);

        PointerBuffer globalWorkEdgeBufferSize = BufferUtils.createPointerBuffer(1);
        globalWorkEdgeBufferSize.put(0, edgeBufferSize);
        int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), compactEdgesKernel, 1, null, globalWorkEdgeBufferSize, null, null, null);
        OCLUtils.checkCLError(errcode);
        field.setEdgeIndices(compactActiveEdgesBuffer);
        //int[] compact = OCLUtils.getIntBuffer(field.getEdgeIndices(), field.getNumEdges());

        CL10.clReleaseKernel(compactEdgesKernel);
    }

    public void FindEdgeIntersectionInfoKernel(KernelsHolder kernels, Vec4f[] normals) {
        field.setNormals(CL10.clCreateBuffer(ctx.getClContext(), CL_MEM_WRITE_ONLY, field.getNumEdges() * 4 * 4, ctx.getErrcode_ret()));
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        int sampleScale = field.getSize() / (meshGen.getVoxelsPerChunk() * meshGen.leafSizeScale);
        long kFindInfoKernel = clCreateKernel(kernels.getKernel(KernelNames.FIND_DEFAULT_EDGES), "FindEdgeIntersectionInfo", ctx.getErrcode_ret());
        clSetKernelArg4i(kFindInfoKernel, 0,
                field.getMin().x/meshGen.leafSizeScale,
                field.getMin().y/meshGen.leafSizeScale,
                field.getMin().z/meshGen.leafSizeScale,
                0);
        clSetKernelArg1i(kFindInfoKernel, 1, sampleScale);
        clSetKernelArg1p(kFindInfoKernel, 2, field.getEdgeIndices());
        clSetKernelArg1p(kFindInfoKernel, 3, field.getNormals());

        PointerBuffer globalWorkNumEdgesSize = BufferUtils.createPointerBuffer(1);
        globalWorkNumEdgesSize.put(0, field.getNumEdges());
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), kFindInfoKernel, 1, null, globalWorkNumEdgesSize, null, null, null);
        OCLUtils.checkCLError(err);

        OCLUtils.getNormals(field.getNormals(), normals);

        CL10.clReleaseKernel(kFindInfoKernel);
        CL10.clFinish(ctx.getClQueue());
    }

    public void destroy(){
        int err = CL10.clReleaseMemObject(edgeOccupancyBuffer);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(edgeIndicesNonCompactBuffer);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(edgeScanBuffer);
        OCLUtils.checkCLError(err);
    }
}