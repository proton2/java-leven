package test;

import core.utils.ResourceLoader;
import dc.impl.opencl.OCLUtils;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL;
import org.lwjgl.opencl.CL10;
import org.lwjgl.opencl.CLCapabilities;
import org.lwjgl.opencl.CLContextCallback;
import org.lwjgl.system.MemoryStack;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;

import static org.lwjgl.opencl.CL10.*;
import static org.lwjgl.opencl.CL11.CL_DEVICE_OPENCL_C_VERSION;
import static org.lwjgl.system.MemoryStack.stackPush;
import static org.lwjgl.system.MemoryUtil.NULL;
import static org.lwjgl.system.MemoryUtil.memUTF8;

/*
    Take from http://forum.lwjgl.org/index.php?topic=6521.0
    @Author OFFICIALHOPSOF
 */

public final class SumOpenCLTest {

    private CLContextCallback clContextCB;
    private long clContext;
    private IntBuffer errcode_ret;
    private long clKernel;
    private long clDevice;
    private long clQueue;
    private long sumProgram;
    private long aMemory;
    private long bMemory;
    private long clPlatform;
    private CLCapabilities clPlatformCapabilities;
    private long resultMemory;
    private static final int size = 100;


    public void run() {
        initializeCL();
        String sumProgramSource = ResourceLoader.loadShader("opencl/test/sumProgramSource.cl");
        sumProgram = CL10.clCreateProgramWithSource(clContext, sumProgramSource, errcode_ret);
        int errcode = clBuildProgram(sumProgram, clDevice, "", null, NULL);
        OCLUtils.checkCLError(errcode);

        // init kernel with constants
        clKernel = clCreateKernel(sumProgram, "sum", errcode_ret);
        OCLUtils.checkCLError(errcode_ret);

        createMemory();

        clSetKernelArg1p(clKernel, 0, aMemory);
        clSetKernelArg1p(clKernel, 1, bMemory);
        clSetKernelArg1p(clKernel, 2, resultMemory);
        clSetKernelArg1i(clKernel, 3, size);

        final int dimensions = 1;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions); // In here we put
        // the total number of work items we want in each dimension.
        globalWorkSize.put(0, size); // Size is a variable we defined a while back showing how many elements are in our arrays.

        // Run the specified number of work units using our OpenCL program kernel
        errcode = clEnqueueNDRangeKernel(clQueue, clKernel, dimensions, null, globalWorkSize, null,
                null, null);
        OCLUtils.checkCLError(errcode);

        CL10.clFinish(clQueue);
        printResults();
        cleanup();
    }

    private void printResults() {
        // This reads the result memory buffer
        FloatBuffer resultBuff = BufferUtils.createFloatBuffer(size);
        // We read the buffer in blocking mode so that when the method returns we know that the result buffer is full
        //CL10.clEnqueueReadBuffer(clQueue, resultMemory, CL10.CL_TRUE, 0, resultBuff, null, null);
        CL10.clEnqueueReadBuffer(clQueue, resultMemory, true, 0, resultBuff, null, null);
        // Print the values in the result buffer
        for (int i = 0; i < resultBuff.capacity(); i++) {
            System.out.println("result at " + i + " = " + resultBuff.get(i));
        }
        // This should print out 100 lines of result floats, each being 99.
    }

    private void createMemory() {
        // Create OpenCL memory object containing the first buffer's list of numbers
        aMemory = CL10.clCreateBuffer(clContext, CL10.CL_MEM_WRITE_ONLY | CL10.CL_MEM_COPY_HOST_PTR, getABuffer(), errcode_ret);
        OCLUtils.checkCLError(errcode_ret);

        // Create OpenCL memory object containing the second buffer's list of numbers
        bMemory = CL10.clCreateBuffer(clContext, CL10.CL_MEM_WRITE_ONLY | CL10.CL_MEM_COPY_HOST_PTR, getBBuffer(), errcode_ret);
        OCLUtils.checkCLError(errcode_ret);

        // Remember the length argument here is in bytes. 4 bytes per float.
        resultMemory = CL10.clCreateBuffer(clContext, CL10.CL_MEM_READ_ONLY, size * 4, errcode_ret);
        OCLUtils.checkCLError(errcode_ret);
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
        CL10.clReleaseCommandQueue(clQueue);
        CL10.clReleaseKernel(clKernel);
        CL10.clReleaseProgram(sumProgram);

        // Destroy our memory objects
        CL10.clReleaseMemObject(aMemory);
        CL10.clReleaseMemObject(bMemory);
        CL10.clReleaseMemObject(resultMemory);

        // Not strictly necessary
        CL.destroy();
    }

    public void initializeCL() {
        errcode_ret = BufferUtils.createIntBuffer(1);

        // Get the first available platform
        try (MemoryStack stack = stackPush()) {
            IntBuffer pi = stack.mallocInt(1);
            OCLUtils.checkCLError(clGetPlatformIDs(null, pi));
            if (pi.get(0) == 0) {
                throw new IllegalStateException("No OpenCL platforms found.");
            }

            PointerBuffer platformIDs = stack.mallocPointer(pi.get(0));
            OCLUtils.checkCLError(clGetPlatformIDs(platformIDs, (IntBuffer) null));

            for (int i = 0; i < platformIDs.capacity() && i == 0; i++) {
                long platform = platformIDs.get(i);
                clPlatformCapabilities = CL.createPlatformCapabilities(platform);
                clPlatform = platform;
            }
        }

        clDevice = getDevice(clPlatform, clPlatformCapabilities, CL_DEVICE_TYPE_GPU);

        // Create the context
        PointerBuffer ctxProps = BufferUtils.createPointerBuffer(7);
        ctxProps.put(CL_CONTEXT_PLATFORM).put(clPlatform).put(NULL).flip();

        clContext = clCreateContext(ctxProps,
                clDevice, clContextCB = CLContextCallback.create((errinfo, private_info, cb,
                                                                  user_data) -> System.out.printf("cl_context_callback\n\tInfo: %s", memUTF8(errinfo))),
                NULL, errcode_ret);

        // create command queue
        clQueue = clCreateCommandQueue(clContext, clDevice, NULL, errcode_ret);
        OCLUtils.checkCLError(errcode_ret);
    }

    private static long getDevice(long platform, CLCapabilities platformCaps, int deviceType) {
        try (MemoryStack stack = stackPush()) {
            IntBuffer pi = stack.mallocInt(1);
            OCLUtils.checkCLError(clGetDeviceIDs(platform, deviceType, null, pi));

            PointerBuffer devices = stack.mallocPointer(pi.get(0));
            OCLUtils.checkCLError(clGetDeviceIDs(platform, deviceType, devices, (IntBuffer) null));

            for (int i = 0; i < devices.capacity(); i++) {
                long device = devices.get(i);

                CLCapabilities caps = CL.createDeviceCapabilities(device, platformCaps);
                if (!(caps.cl_khr_gl_sharing || caps.cl_APPLE_gl_sharing)) {
                    continue;
                }

                System.out.printf("\n\t** NEW DEVICE: [0x%X]\n", device);

                System.out.println("\tCL_DEVICE_TYPE = " + OCLUtils.getDeviceInfoLong(device, CL_DEVICE_TYPE));
                System.out.println("\tCL_DEVICE_VENDOR_ID = " + OCLUtils.getDeviceInfoInt(device, CL_DEVICE_VENDOR_ID));
                System.out.println("\tCL_DEVICE_MAX_COMPUTE_UNITS = " + OCLUtils.getDeviceInfoInt(device, CL_DEVICE_MAX_COMPUTE_UNITS));
                System.out.println("\tCL_DEVICE_MAX_WORK_ITEM_DIMENSIONS = " + OCLUtils.getDeviceInfoInt(device, CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS));
                System.out.println("\tCL_DEVICE_MAX_WORK_GROUP_SIZE = " + OCLUtils.getDeviceInfoPointer(device, CL_DEVICE_MAX_WORK_GROUP_SIZE));
                System.out.println("\tCL_DEVICE_MAX_CLOCK_FREQUENCY = " + OCLUtils.getDeviceInfoInt(device, CL_DEVICE_MAX_CLOCK_FREQUENCY));
                System.out.println("\tCL_DEVICE_ADDRESS_BITS = " + OCLUtils.getDeviceInfoInt(device, CL_DEVICE_ADDRESS_BITS));
                System.out.println("\tCL_DEVICE_AVAILABLE = " + (OCLUtils.getDeviceInfoInt(device, CL_DEVICE_AVAILABLE) != 0));
                System.out.println("\tCL_DEVICE_COMPILER_AVAILABLE = " + (OCLUtils.getDeviceInfoInt(device, CL_DEVICE_COMPILER_AVAILABLE) != 0));

                OCLUtils.printDeviceInfo(device, "CL_DEVICE_NAME", CL_DEVICE_NAME);
                OCLUtils.printDeviceInfo(device, "CL_DEVICE_VENDOR", CL_DEVICE_VENDOR);
                OCLUtils.printDeviceInfo(device, "CL_DRIVER_VERSION", CL_DRIVER_VERSION);
                OCLUtils.printDeviceInfo(device, "CL_DEVICE_PROFILE", CL_DEVICE_PROFILE);
                OCLUtils.printDeviceInfo(device, "CL_DEVICE_VERSION", CL_DEVICE_VERSION);
                OCLUtils.printDeviceInfo(device, "CL_DEVICE_EXTENSIONS", CL_DEVICE_EXTENSIONS);
                if (caps.OpenCL11) {
                    OCLUtils.printDeviceInfo(device, "CL_DEVICE_OPENCL_C_VERSION", CL_DEVICE_OPENCL_C_VERSION);
                }

                return device;
            }
        }

        return NULL;
    }

    public static void main(String... args) {
        SumOpenCLTest clApp = new SumOpenCLTest();
        clApp.run();
    }
}