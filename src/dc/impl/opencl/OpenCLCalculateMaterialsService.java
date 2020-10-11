package dc.impl.opencl;

import dc.impl.GPUDensityField;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import java.nio.IntBuffer;

import static org.lwjgl.opencl.CL10.*;

public final class OpenCLCalculateMaterialsService {
    private long clKernel;
    private final int size;
    private ComputeContext ctx;

    public OpenCLCalculateMaterialsService(ComputeContext computeContext, int size) {
        this.size = size;
        this.ctx = computeContext;
    }

    public void run(KernelsHolder kernels, int sampleScale, int[] result, GPUDensityField field) {
        // init kernel with constants
        clKernel = clCreateKernel(kernels.getKernel(KernelNames.DENSITY), "GenerateDefaultField", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        createMemory(field);

        clSetKernelArg4i(clKernel, 0, field.getMin().x, field.getMin().y, field.getMin().z, 0);
        clSetKernelArg1i(clKernel, 1, sampleScale);
        clSetKernelArg1p(clKernel, 2, field.getMaterials());

        final int dimensions = 3;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions); // In here we put the total number of work items we want in each dimension.
        globalWorkSize.put(0, size); // Size is a variable we defined a while back showing how many elements are in our arrays.
        globalWorkSize.put(1, size); // Size is a variable we defined a while back showing how many elements are in our arrays.
        globalWorkSize.put(2, size); // Size is a variable we defined a while back showing how many elements are in our arrays.

        // Run the specified number of work units using our OpenCL program kernel
        int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), clKernel, dimensions, null, globalWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);

        CL10.clFinish(ctx.getClQueue());

        getResults(result, field);
        cleanup();
    }

    private void getResults(int[] result, GPUDensityField field) {
        // This reads the result memory buffer
        IntBuffer resultBuff = BufferUtils.createIntBuffer(size * size * size);
        // We read the buffer in blocking mode so that when the method returns we know that the result buffer is full
        //CL10.clEnqueueReadBuffer(clQueue, resultMemory, CL10.CL_TRUE, 0, resultBuff, null, null);
        CL10.clEnqueueReadBuffer(ctx.getClQueue(), field.getMaterials(), true, 0, resultBuff, null, null);
        // Print the values in the result buffer
        resultBuff.get(result);
    }

    private void createMemory(GPUDensityField field) {
        // Remember the length argument here is in bytes. 4 bytes per float.
        long resultMemory = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, size * size * size * 4, ctx.getErrcode_ret());
        field.setMaterials(resultMemory);
        OCLUtils.checkCLError(ctx.getErrcode_ret());
    }

    private void cleanup() {
        CL10.clReleaseKernel(clKernel);
    }
}