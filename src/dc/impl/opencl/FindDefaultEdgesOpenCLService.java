package dc.impl.opencl;

import core.math.Vec4f;
import dc.impl.GPUDensityField;
import dc.impl.MeshGenerationContext;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;

import static org.lwjgl.opencl.CL10.*;

public final class FindDefaultEdgesOpenCLService {
    private long edgeOccupancyBuffer, edgeIndicesNonCompactBuffer;
    private final ScanOpenCLService scanOpenCLService;
    private int numEdges, edgeBufferSize;
    private ComputeContext ctx;
    private final MeshGenerationContext meshGen;

    public FindDefaultEdgesOpenCLService(ComputeContext computeContext, ScanOpenCLService scanOpenCLService, MeshGenerationContext meshGenerationContext) {
        this.meshGen = meshGenerationContext;
        this.scanOpenCLService = scanOpenCLService;
        this.ctx = computeContext;
    }

    public int run(KernelsHolder kernels, GPUDensityField field, int voxelPerChunk, int hermiteIndexSize) {
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

        long edgeScanBuffer = CL10.clCreateBuffer(ctx.getClContext(), CL_MEM_READ_WRITE, edgeBufferSize * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        numEdges = scanOpenCLService.exclusiveScan(edgeOccupancyBuffer, edgeScanBuffer, edgeBufferSize);
        field.setNumEdges(numEdges);
        if (field.getNumEdges() < 0) {
            System.out.println("FindDefaultEdges: ExclusiveScan error=%d\n" + field.getNumEdges());
            return field.getNumEdges();
        }
        if(field.getNumEdges()==0){
            return 0;
        }

        long compactActiveEdgesBuffer = CL10.clCreateBuffer(ctx.getClContext(), CL_MEM_READ_WRITE, field.getNumEdges() * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        long compactEdgesKernel = clCreateKernel(kernels.getKernel(KernelNames.FIND_DEFAULT_EDGES), "CompactEdges", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        clSetKernelArg1p(compactEdgesKernel, 0, edgeOccupancyBuffer);
        clSetKernelArg1p(compactEdgesKernel, 1, edgeScanBuffer);
        clSetKernelArg1p(compactEdgesKernel, 2, edgeIndicesNonCompactBuffer);
        clSetKernelArg1p(compactEdgesKernel, 3, compactActiveEdgesBuffer);

        PointerBuffer globalWorkEdgeBufferSize = BufferUtils.createPointerBuffer(1);
        globalWorkEdgeBufferSize.put(0, edgeBufferSize);
        errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), compactEdgesKernel, 1, null, globalWorkEdgeBufferSize, null, null, null);
        OCLUtils.checkCLError(errcode);
        field.setEdgeIndices(compactActiveEdgesBuffer);
        //int[] compact = OCLUtils.getIntBuffer(field.getEdgeIndices(), field.getNumEdges());

        field.setNormals(CL10.clCreateBuffer(ctx.getClContext(), CL_MEM_WRITE_ONLY, field.getNumEdges() * 4 * 4, ctx.getErrcode_ret()));
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        int sampleScale = field.getSize() / (voxelPerChunk * meshGen.leafSizeScale);
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
        errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), kFindInfoKernel, 1, null, globalWorkNumEdgesSize, null, null, null);
        OCLUtils.checkCLError(errcode);

        Vec4f[] vec4f = getNormals(field.getNormals());

        CL10.clReleaseMemObject(edgeScanBuffer);
        CL10.clReleaseKernel(compactEdgesKernel);
        CL10.clReleaseKernel(kFindInfoKernel);
        CL10.clReleaseKernel(findFieldEdgesKernel);
        CL10.clFinish(ctx.getClQueue());
        return field.getNumEdges();
    }

    public Vec4f[] getNormals(long normBuffer){
        FloatBuffer resultBuff = BufferUtils.createFloatBuffer(numEdges * 4);
        CL10.clEnqueueReadBuffer(ctx.getClQueue(), normBuffer, true, 0, resultBuff, null, null);
        Vec4f[] normalsBuffer = new Vec4f[numEdges];
        for (int i = 0; i < numEdges; i++) {
            int index = i * 4;
            Vec4f normal = new Vec4f();
            normal.x = resultBuff.get(index+0);
            normal.y = resultBuff.get(index+1);
            normal.z = resultBuff.get(index+2);
            normal.w = resultBuff.get(index+3);
            normalsBuffer[i] = normal;
        }
        return normalsBuffer;
    }

    public int[] getEdgeIndicesCompact(long compactEdgeIndicates){
        IntBuffer resultBuff = BufferUtils.createIntBuffer(numEdges);
        CL10.clEnqueueReadBuffer(ctx.getClQueue(), compactEdgeIndicates, true, 0, resultBuff,
                null, null);
        int[] returnBuffer = new int[numEdges];
        resultBuff.get(returnBuffer);
        return returnBuffer;
    }

    public int[] getEdgeOccupancyBuffer(){
        IntBuffer resultBuff = BufferUtils.createIntBuffer(edgeBufferSize);
        CL10.clEnqueueReadBuffer(ctx.getClQueue(), edgeOccupancyBuffer, true, 0, resultBuff,
                null, null);
        int[] returnBuffer = new int[edgeBufferSize];
        resultBuff.get(returnBuffer);
        CL10.clReleaseMemObject(edgeOccupancyBuffer);
        return returnBuffer;
    }

    public int[] getEdgeIndicesNonCompactBuffer(){
        IntBuffer resultBuff = BufferUtils.createIntBuffer(edgeBufferSize);
        CL10.clEnqueueReadBuffer(ctx.getClQueue(), edgeIndicesNonCompactBuffer, true, 0, resultBuff,
                null, null);
        int[] returnBuffer = new int[edgeBufferSize];
        resultBuff.get(returnBuffer);
        CL10.clReleaseMemObject(edgeIndicesNonCompactBuffer);
        return returnBuffer;
    }
}