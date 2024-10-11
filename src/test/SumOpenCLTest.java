package test;

import core.utils.ResourceLoader;
import dc.impl.nonused.opencl.BufferGpu;
import dc.impl.nonused.opencl.BufferGpuService;
import dc.impl.nonused.opencl.ComputeContext;
import dc.impl.nonused.opencl.OCLUtils;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL;
import org.lwjgl.opencl.CL10;

import java.nio.FloatBuffer;

import static org.lwjgl.opencl.CL10.*;
import static org.lwjgl.system.MemoryUtil.NULL;

/*
    Take from http://forum.lwjgl.org/index.php?topic=6521.0
    @Author OFFICIALHOPSOF
 */

public final class SumOpenCLTest {
    private long clKernel;
    private long sumProgram;
    private BufferGpu aMemory;
    private BufferGpu bMemory;
    private BufferGpu resultMemory;
    private static final int size = 100;
    private static BufferGpuService bufferGpuService;
    private static ComputeContext ctx;

    public void run() {
        ctx = OCLUtils.getOpenCLContext();
        bufferGpuService = new BufferGpuService(ctx);
        String sumProgramSource = ResourceLoader.loadShader("opencl/test/sumProgramSource.cl");
        sumProgram = CL10.clCreateProgramWithSource(ctx.getClContext(), sumProgramSource, ctx.getErrcode_ret());
        int errcode = clBuildProgram(sumProgram, ctx.getClDevice(), "", null, NULL);
        OCLUtils.checkCLError(errcode);

        // init kernel with constants
        clKernel = clCreateKernel(sumProgram, "sum", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        createMemory();

        clSetKernelArg1p(clKernel, 0, aMemory.getMem());
        clSetKernelArg1p(clKernel, 1, bMemory.getMem());
        clSetKernelArg1p(clKernel, 2, resultMemory.getMem());
        clSetKernelArg1i(clKernel, 3, size);

        final int dimensions = 1;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions); // In here we put
        // the total number of work items we want in each dimension.
        globalWorkSize.put(0, size); // Size is a variable we defined a while back showing how many elements are in our arrays.

        // Run the specified number of work units using our OpenCL program kernel
        errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), clKernel, dimensions, null, globalWorkSize, null,
                null, null);
        OCLUtils.checkCLError(errcode);

        CL10.clFinish(ctx.getClQueue());
        printResults();
        cleanup();
    }

    private void printResults() {
        // This reads the result memory buffer
        FloatBuffer resultBuff = BufferUtils.createFloatBuffer(size);
        // We read the buffer in blocking mode so that when the method returns we know that the result buffer is full
        //CL10.clEnqueueReadBuffer(clQueue, resultMemory, CL10.CL_TRUE, 0, resultBuff, null, null);
        CL10.clEnqueueReadBuffer(ctx.getClQueue(), resultMemory.getMem(), true, 0, resultBuff, null, null);
        // Print the values in the result buffer
        for (int i = 0; i < resultBuff.capacity(); i++) {
            System.out.println("result at " + i + " = " + resultBuff.get(i));
        }
        // This should print out 100 lines of result floats, each being 99.
    }

    private void createMemory() {
        // Create OpenCL memory object containing the first buffer's list of numbers
        aMemory = bufferGpuService.create("aMemory", getABuffer(), CL10.CL_MEM_WRITE_ONLY | CL10.CL_MEM_COPY_HOST_PTR);
        // Create OpenCL memory object containing the second buffer's list of numbers
        bMemory = bufferGpuService.create("bMemory", getBBuffer(), CL10.CL_MEM_WRITE_ONLY | CL10.CL_MEM_COPY_HOST_PTR);
        // Remember the length argument here is in bytes. 4 bytes per float.
        resultMemory = bufferGpuService.create("resultMemory", size * 4, CL10.CL_MEM_READ_ONLY);
    }

    private FloatBuffer getABuffer() {
        // Create float array from 0 to size-1.
        FloatBuffer aBuff = BufferUtils.createFloatBuffer(size);
        float[] tempData = new float[size];
        for (int i = 0; i < size; i++) {
            tempData[i] = i;
            System.out.println("a[" + i + "]=" + i);
        }
        aBuff.put(tempData);
        aBuff.rewind();
        return aBuff;
    }

    private FloatBuffer getBBuffer() {
        // Create float array from size-1 to 0. This means that the result should be size-1 for each
        // element.
        FloatBuffer bBuff = BufferUtils.createFloatBuffer(size);
        float[] tempData = new float[size];
        for (int j = 0, i = size - 1; j < size; j++, i--) {
            tempData[j] = i;
            System.out.println("b[" + j + "]=" + i);
        }
        bBuff.put(tempData);
        bBuff.rewind();
        return bBuff;
    }

    private void cleanup() {
        // Destroy our kernel and program
        CL10.clReleaseCommandQueue(ctx.getClQueue());
        CL10.clReleaseKernel(clKernel);
        CL10.clReleaseProgram(sumProgram);

        // Destroy our memory objects
        bufferGpuService.release(aMemory);
        bufferGpuService.release(bMemory);
        bufferGpuService.release(resultMemory);

        // Not strictly necessary
        CL.destroy();
    }

    public static void main(String... args) {
        SumOpenCLTest clApp = new SumOpenCLTest();
        clApp.run();
    }
}