package dc.impl.opencl;

import core.math.Vec3i;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import java.nio.IntBuffer;

import static org.lwjgl.opencl.CL10.*;

public final class OpenCLCalculateMaterialsService {

    private long clKernel;

    private long resultMemory;
    private final int size;

    public OpenCLCalculateMaterialsService(int size) {
        this.size = size;
    }

    public void run(MeshGenerationContext meshGen, Vec3i chunkMin, int sampleScale, int[] result) {
        ComputeContext openCLContext = OCLUtils.getOpenCLContext();
        // init kernel with constants
        clKernel = clCreateKernel(meshGen.getDensityProgram(), "GenerateDefaultField", openCLContext.getErrcode_ret());
        OCLUtils.checkCLError(openCLContext.getErrcode_ret());

        createMemory(openCLContext);

        clSetKernelArg4i(clKernel, 0, chunkMin.x, chunkMin.y, chunkMin.z, 0);
        clSetKernelArg1i(clKernel, 1, sampleScale);
        clSetKernelArg1p(clKernel, 2, resultMemory);

        final int dimensions = 3;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions); // In here we put the total number of work items we want in each dimension.
        globalWorkSize.put(0, size); // Size is a variable we defined a while back showing how many elements are in our arrays.
        globalWorkSize.put(1, size); // Size is a variable we defined a while back showing how many elements are in our arrays.
        globalWorkSize.put(2, size); // Size is a variable we defined a while back showing how many elements are in our arrays.

        // Run the specified number of work units using our OpenCL program kernel
        int errcode = clEnqueueNDRangeKernel(openCLContext.getClQueue(), clKernel, dimensions, null, globalWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);

        CL10.clFinish(openCLContext.getClQueue());

        getResults(openCLContext, result);
        cleanup();
    }

    private void getResults(ComputeContext openCLContext, int[] result) {
        // This reads the result memory buffer
        IntBuffer resultBuff = BufferUtils.createIntBuffer(size * size * size);
        // We read the buffer in blocking mode so that when the method returns we know that the result buffer is full
        //CL10.clEnqueueReadBuffer(clQueue, resultMemory, CL10.CL_TRUE, 0, resultBuff, null, null);
        CL10.clEnqueueReadBuffer(openCLContext.getClQueue(), resultMemory, true, 0, resultBuff, null, null);
        // Print the values in the result buffer

        resultBuff.get(result);
    }

    private void createMemory(ComputeContext openCLContext) {
        // Remember the length argument here is in bytes. 4 bytes per float.
        resultMemory = CL10.clCreateBuffer(openCLContext.getClContext(), CL10.CL_MEM_READ_ONLY, size * size * size * 4, openCLContext.getErrcode_ret());
        OCLUtils.checkCLError(openCLContext.getErrcode_ret());
    }

    private void cleanup() {
        CL10.clReleaseKernel(clKernel);
        // Destroy our memory objects
        CL10.clReleaseMemObject(resultMemory);
    }
}