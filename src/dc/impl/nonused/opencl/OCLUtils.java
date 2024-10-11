package dc.impl.nonused.opencl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.math.Vec4i;
import dc.OctreeDrawInfo;
import dc.OctreeNode;
import dc.OctreeNodeType;
import dc.PointerBasedOctreeNode;
import dc.entities.MeshVertex;
import dc.solver.LevenQefSolver;
import dc.solver.QEFData;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL;
import org.lwjgl.opencl.CL10;
import org.lwjgl.opencl.CLCapabilities;
import org.lwjgl.opencl.CLContextCallback;
import org.lwjgl.system.MemoryStack;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.LongBuffer;
import java.util.List;

import static org.lwjgl.opencl.CL10.*;
import static org.lwjgl.opencl.CL11.CL_DEVICE_OPENCL_C_VERSION;
import static org.lwjgl.system.MemoryStack.stackPush;
import static org.lwjgl.system.MemoryUtil.*;

public class OCLUtils {

    // Error codes
    public static final int CL_SUCCESS                                  = 0;
    public static final int CL_DEVICE_NOT_FOUND                         = -1;
    public static final int CL_DEVICE_NOT_AVAILABLE                     = -2;
    public static final int CL_COMPILER_NOT_AVAILABLE                   = -3;
    public static final int CL_MEM_OBJECT_ALLOCATION_FAILURE            = -4;
    public static final int CL_OUT_OF_RESOURCES                         = -5;
    public static final int CL_OUT_OF_HOST_MEMORY                       = -6;
    public static final int CL_PROFILING_INFO_NOT_AVAILABLE             = -7;
    public static final int CL_MEM_COPY_OVERLAP                         = -8;
    public static final int CL_IMAGE_FORMAT_MISMATCH                    = -9;
    public static final int CL_IMAGE_FORMAT_NOT_SUPPORTED               = -10;
    public static final int CL_BUILD_PROGRAM_FAILURE                    = -11;
    public static final int CL_MAP_FAILURE                              = -12;
    public static final int CL_MISALIGNED_SUB_BUFFER_OFFSET             = -13;
    public static final int CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST= -14;
    // OPENCL_1_2
    public static final int CL_COMPILE_PROGRAM_FAILURE                  = -15;
    public static final int CL_LINKER_NOT_AVAILABLE                     = -16;
    public static final int CL_LINK_PROGRAM_FAILURE                     = -17;
    public static final int CL_DEVICE_PARTITION_FAILED                  = -18;
    public static final int CL_KERNEL_ARG_INFO_NOT_AVAILABLE            = -19;

    public static final int CL_INVALID_VALUE                            = -30;
    public static final int CL_INVALID_DEVICE_TYPE                      = -31;
    public static final int CL_INVALID_PLATFORM                         = -32;
    public static final int CL_INVALID_DEVICE                           = -33;
    public static final int CL_INVALID_CONTEXT                          = -34;
    public static final int CL_INVALID_QUEUE_PROPERTIES                 = -35;
    public static final int CL_INVALID_COMMAND_QUEUE                    = -36;
    public static final int CL_INVALID_HOST_PTR                         = -37;
    public static final int CL_INVALID_MEM_OBJECT                       = -38;
    public static final int CL_INVALID_IMAGE_FORMAT_DESCRIPTOR          = -39;
    public static final int CL_INVALID_IMAGE_SIZE                       = -40;
    public static final int CL_INVALID_SAMPLER                          = -41;
    public static final int CL_INVALID_BINARY                           = -42;
    public static final int CL_INVALID_BUILD_OPTIONS                    = -43;
    public static final int CL_INVALID_PROGRAM                          = -44;
    public static final int CL_INVALID_PROGRAM_EXECUTABLE               = -45;
    public static final int CL_INVALID_KERNEL_NAME                      = -46;
    public static final int CL_INVALID_KERNEL_DEFINITION                = -47;
    public static final int CL_INVALID_KERNEL                           = -48;
    public static final int CL_INVALID_ARG_INDEX                        = -49;
    public static final int CL_INVALID_ARG_VALUE                        = -50;
    public static final int CL_INVALID_ARG_SIZE                         = -51;
    public static final int CL_INVALID_KERNEL_ARGS                      = -52;
    public static final int CL_INVALID_WORK_DIMENSION                   = -53;
    public static final int CL_INVALID_WORK_GROUP_SIZE                  = -54;
    public static final int CL_INVALID_WORK_ITEM_SIZE                   = -55;
    public static final int CL_INVALID_GLOBAL_OFFSET                    = -56;
    public static final int CL_INVALID_EVENT_WAIT_LIST                  = -57;
    public static final int CL_INVALID_EVENT                            = -58;
    public static final int CL_INVALID_OPERATION                        = -59;
    public static final int CL_INVALID_GL_OBJECT                        = -60;
    public static final int CL_INVALID_BUFFER_SIZE                      = -61;
    public static final int CL_INVALID_MIP_LEVEL                        = -62;
    public static final int CL_INVALID_GLOBAL_WORK_SIZE                 = -63;
    // OPENCL_1_2
    public static final int CL_INVALID_PROPERTY                         = -64;
    public static final int CL_INVALID_IMAGE_DESCRIPTOR                 = -65;
    public static final int CL_INVALID_COMPILER_OPTIONS                 = -66;
    public static final int CL_INVALID_LINKER_OPTIONS                   = -67;
    public static final int CL_INVALID_DEVICE_PARTITION_COUNT           = -68;

    // OPENCL_2_0
    public static final int CL_INVALID_PIPE_SIZE                        = -69;
    public static final int CL_INVALID_DEVICE_QUEUE                     = -70;


    public static final int CL_JOCL_INTERNAL_ERROR                      = -16384;
    public static final int CL_INVALID_GL_SHAREGROUP_REFERENCE_KHR      = -1000;
    public static final int CL_PLATFORM_NOT_FOUND_KHR                   = -1001;

    public static String stringFor_errorCode(int n)
    {
        switch (n)
        {
            case CL_SUCCESS: return "CL_SUCCESS";
            case CL_DEVICE_NOT_FOUND: return "CL_DEVICE_NOT_FOUND";
            case CL_DEVICE_NOT_AVAILABLE: return "CL_DEVICE_NOT_AVAILABLE";
            case CL_COMPILER_NOT_AVAILABLE: return "CL_COMPILER_NOT_AVAILABLE";
            case CL_MEM_OBJECT_ALLOCATION_FAILURE: return "CL_MEM_OBJECT_ALLOCATION_FAILURE";
            case CL_OUT_OF_RESOURCES: return "CL_OUT_OF_RESOURCES";
            case CL_OUT_OF_HOST_MEMORY: return "CL_OUT_OF_HOST_MEMORY";
            case CL_PROFILING_INFO_NOT_AVAILABLE: return "CL_PROFILING_INFO_NOT_AVAILABLE";
            case CL_MEM_COPY_OVERLAP: return "CL_MEM_COPY_OVERLAP";
            case CL_IMAGE_FORMAT_MISMATCH: return "CL_IMAGE_FORMAT_MISMATCH";
            case CL_IMAGE_FORMAT_NOT_SUPPORTED: return "CL_IMAGE_FORMAT_NOT_SUPPORTED";
            case CL_BUILD_PROGRAM_FAILURE: return "CL_BUILD_PROGRAM_FAILURE";
            case CL_MAP_FAILURE: return "CL_MAP_FAILURE";
            case CL_MISALIGNED_SUB_BUFFER_OFFSET: return "CL_MISALIGNED_SUB_BUFFER_OFFSET";
            case CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST: return "CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST";
            case CL_COMPILE_PROGRAM_FAILURE: return "CL_COMPILE_PROGRAM_FAILURE";
            case CL_LINKER_NOT_AVAILABLE: return "CL_LINKER_NOT_AVAILABLE";
            case CL_LINK_PROGRAM_FAILURE: return "CL_LINK_PROGRAM_FAILURE";
            case CL_DEVICE_PARTITION_FAILED: return "CL_DEVICE_PARTITION_FAILED";
            case CL_KERNEL_ARG_INFO_NOT_AVAILABLE: return "CL_KERNEL_ARG_INFO_NOT_AVAILABLE";
            case CL_INVALID_VALUE: return "CL_INVALID_VALUE";
            case CL_INVALID_DEVICE_TYPE: return "CL_INVALID_DEVICE_TYPE";
            case CL_INVALID_PLATFORM: return "CL_INVALID_PLATFORM";
            case CL_INVALID_DEVICE: return "CL_INVALID_DEVICE";
            case CL_INVALID_CONTEXT: return "CL_INVALID_CONTEXT";
            case CL_INVALID_QUEUE_PROPERTIES: return "CL_INVALID_QUEUE_PROPERTIES";
            case CL_INVALID_COMMAND_QUEUE: return "CL_INVALID_COMMAND_QUEUE";
            case CL_INVALID_HOST_PTR: return "CL_INVALID_HOST_PTR";
            case CL_INVALID_MEM_OBJECT: return "CL_INVALID_MEM_OBJECT";
            case CL_INVALID_IMAGE_FORMAT_DESCRIPTOR: return "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR";
            case CL_INVALID_IMAGE_SIZE: return "CL_INVALID_IMAGE_SIZE";
            case CL_INVALID_SAMPLER: return "CL_INVALID_SAMPLER";
            case CL_INVALID_BINARY: return "CL_INVALID_BINARY";
            case CL_INVALID_BUILD_OPTIONS: return "CL_INVALID_BUILD_OPTIONS";
            case CL_INVALID_PROGRAM: return "CL_INVALID_PROGRAM";
            case CL_INVALID_PROGRAM_EXECUTABLE: return "CL_INVALID_PROGRAM_EXECUTABLE";
            case CL_INVALID_KERNEL_NAME: return "CL_INVALID_KERNEL_NAME";
            case CL_INVALID_KERNEL_DEFINITION: return "CL_INVALID_KERNEL_DEFINITION";
            case CL_INVALID_KERNEL: return "CL_INVALID_KERNEL";
            case CL_INVALID_ARG_INDEX: return "CL_INVALID_ARG_INDEX";
            case CL_INVALID_ARG_VALUE: return "CL_INVALID_ARG_VALUE";
            case CL_INVALID_ARG_SIZE: return "CL_INVALID_ARG_SIZE";
            case CL_INVALID_KERNEL_ARGS: return "CL_INVALID_KERNEL_ARGS";
            case CL_INVALID_WORK_DIMENSION: return "CL_INVALID_WORK_DIMENSION";
            case CL_INVALID_WORK_GROUP_SIZE: return "CL_INVALID_WORK_GROUP_SIZE";
            case CL_INVALID_WORK_ITEM_SIZE: return "CL_INVALID_WORK_ITEM_SIZE";
            case CL_INVALID_GLOBAL_OFFSET: return "CL_INVALID_GLOBAL_OFFSET";
            case CL_INVALID_EVENT_WAIT_LIST: return "CL_INVALID_EVENT_WAIT_LIST";
            case CL_INVALID_EVENT: return "CL_INVALID_EVENT";
            case CL_INVALID_OPERATION: return "CL_INVALID_OPERATION";
            case CL_INVALID_GL_OBJECT: return "CL_INVALID_GL_OBJECT";
            case CL_INVALID_BUFFER_SIZE: return "CL_INVALID_BUFFER_SIZE";
            case CL_INVALID_MIP_LEVEL: return "CL_INVALID_MIP_LEVEL";
            case CL_INVALID_GLOBAL_WORK_SIZE: return "CL_INVALID_GLOBAL_WORK_SIZE";
            case CL_INVALID_PROPERTY: return "CL_INVALID_PROPERTY";
            case CL_INVALID_IMAGE_DESCRIPTOR: return "CL_INVALID_IMAGE_DESCRIPTOR";
            case CL_INVALID_COMPILER_OPTIONS: return "CL_INVALID_COMPILER_OPTIONS";
            case CL_INVALID_LINKER_OPTIONS: return "CL_INVALID_LINKER_OPTIONS";
            case CL_INVALID_DEVICE_PARTITION_COUNT: return "CL_INVALID_DEVICE_PARTITION_COUNT";
            case CL_INVALID_PIPE_SIZE: return "CL_INVALID_PIPE_SIZE";
            case CL_INVALID_DEVICE_QUEUE: return "CL_INVALID_DEVICE_QUEUE";
            case CL_JOCL_INTERNAL_ERROR: return "CL_JOCL_INTERNAL_ERROR";
            case CL_INVALID_GL_SHAREGROUP_REFERENCE_KHR: return "CL_INVALID_GL_SHAREGROUP_REFERENCE_KHR";
            case CL_PLATFORM_NOT_FOUND_KHR: return "CL_PLATFORM_NOT_FOUND_KHR";

            // Some OpenCL implementation return 1 for glBuildProgram
            // if the source code contains errors...
            case 1: return "Error in program source code";
        }
        return "INVALID error code: "+n;
    }

    private OCLUtils() { }

    private static ComputeContext openCLContext = null;

    public static ComputeContext getOpenCLContext() {
        if(openCLContext==null) {
            try {
                openCLContext = createOpenCLContext();
            } catch (Throwable e) {
                return null;
            }
        }
        return openCLContext;
    }

    public static void checkCLErrorEx(int errcode) throws OclException {
        if (errcode != CL_SUCCESS) {
            throw new OclException("OpenCL error " + stringFor_errorCode(errcode));
        }
    }

    public static long getMaxWorkGroupSize(long kernel) {
        ByteBuffer rkwgs = BufferUtils.createByteBuffer(8);
        int err = CL10.clGetKernelWorkGroupInfo(kernel, openCLContext.getClDevice(), CL10.CL_KERNEL_WORK_GROUP_SIZE, rkwgs, null);
        OCLUtils.checkCLError(err);
        return rkwgs.getLong(0);
    }

    private static ComputeContext createOpenCLContext() throws Throwable {
        long clPlatform = 0L;
        CLCapabilities clPlatformCapabilities = null;
        CLContextCallback clContextCB;
        IntBuffer errcode_ret = BufferUtils.createIntBuffer(1);

        // Get the first available platform
        try (MemoryStack stack = stackPush()) {
            IntBuffer pi = stack.mallocInt(1);
            checkCLErrorEx(clGetPlatformIDs(null, pi));
            if (pi.get(0) == 0) {
                throw new IllegalStateException("No OpenCL platforms found.");
            }

            PointerBuffer platformIDs = stack.mallocPointer(pi.get(0));
            checkCLError(clGetPlatformIDs(platformIDs, (IntBuffer) null));

            for (int i = 0; i < platformIDs.capacity() && i == 0; i++) {
                long platform = platformIDs.get(i);
                clPlatformCapabilities = CL.createPlatformCapabilities(platform);
                clPlatform = platform;
            }
        }

        long clDevice = getDevice(clPlatform, clPlatformCapabilities, CL_DEVICE_TYPE_GPU);

        // Create the context
        PointerBuffer ctxProps = BufferUtils.createPointerBuffer(7);
        ctxProps.put(CL_CONTEXT_PLATFORM).put(clPlatform).put(NULL).flip();

        long clContext = clCreateContext(ctxProps,
                clDevice, clContextCB = CLContextCallback.create((errinfo, private_info, cb,
                                                                  user_data) -> System.out.printf("cl_context_callback\n\tInfo: %s", memUTF8(errinfo))),
                NULL, errcode_ret);

        // create command queue
        long clQueue = clCreateCommandQueue(clContext, clDevice, NULL, errcode_ret);
        checkCLError(errcode_ret);

        return new ComputeContext(clDevice, clQueue, clContext, errcode_ret);
    }

    private static long getDevice(long platform, CLCapabilities platformCaps, int deviceType) {
        try (MemoryStack stack = stackPush()) {
            IntBuffer pi = stack.mallocInt(1);
            checkCLError(clGetDeviceIDs(platform, deviceType, null, pi));

            PointerBuffer devices = stack.mallocPointer(pi.get(0));
            checkCLError(clGetDeviceIDs(platform, deviceType, devices, (IntBuffer) null));

            for (int i = 0; i < devices.capacity(); i++) {
                long device = devices.get(i);

                CLCapabilities caps = CL.createDeviceCapabilities(device, platformCaps);
                if (!(caps.cl_khr_gl_sharing || caps.cl_APPLE_gl_sharing)) {
                    continue;
                }

                System.out.printf("\n\t** NEW DEVICE: [0x%X]\n", device);

                System.out.println("\tCL_DEVICE_TYPE = " + getDeviceInfoLong(device, CL_DEVICE_TYPE));
                System.out.println("\tCL_DEVICE_VENDOR_ID = " + getDeviceInfoInt(device, CL_DEVICE_VENDOR_ID));
                System.out.println("\tCL_DEVICE_MAX_COMPUTE_UNITS = " + getDeviceInfoInt(device, CL_DEVICE_MAX_COMPUTE_UNITS));
                System.out.println("\tCL_DEVICE_MAX_WORK_ITEM_DIMENSIONS = " + getDeviceInfoInt(device, CL_DEVICE_MAX_WORK_ITEM_DIMENSIONS));
                System.out.println("\tCL_DEVICE_MAX_WORK_GROUP_SIZE = " + getDeviceInfoPointer(device, CL_DEVICE_MAX_WORK_GROUP_SIZE));
                System.out.println("\tCL_DEVICE_MAX_CLOCK_FREQUENCY = " + getDeviceInfoInt(device, CL_DEVICE_MAX_CLOCK_FREQUENCY));
                System.out.println("\tCL_DEVICE_ADDRESS_BITS = " + getDeviceInfoInt(device, CL_DEVICE_ADDRESS_BITS));
                System.out.println("\tCL_DEVICE_AVAILABLE = " + (getDeviceInfoInt(device, CL_DEVICE_AVAILABLE) != 0));
                System.out.println("\tCL_DEVICE_COMPILER_AVAILABLE = " + (getDeviceInfoInt(device, CL_DEVICE_COMPILER_AVAILABLE) != 0));

                printDeviceInfo(device, "CL_DEVICE_NAME", CL_DEVICE_NAME);
                printDeviceInfo(device, "CL_DEVICE_VENDOR", CL_DEVICE_VENDOR);
                printDeviceInfo(device, "CL_DRIVER_VERSION", CL_DRIVER_VERSION);
                printDeviceInfo(device, "CL_DEVICE_PROFILE", CL_DEVICE_PROFILE);
                printDeviceInfo(device, "CL_DEVICE_VERSION", CL_DEVICE_VERSION);
                printDeviceInfo(device, "CL_DEVICE_EXTENSIONS", CL_DEVICE_EXTENSIONS);
                if (caps.OpenCL11) {
                    printDeviceInfo(device, "CL_DEVICE_OPENCL_C_VERSION", CL_DEVICE_OPENCL_C_VERSION);
                }

                return device;
            }
        }
        return NULL;
    }




    static String getPlatformInfoStringASCII(long cl_platform_id, int param_name) {
        try (MemoryStack stack = stackPush()) {
            PointerBuffer pp = stack.mallocPointer(1);
            checkCLError(clGetPlatformInfo(cl_platform_id, param_name, (ByteBuffer)null, pp));
            int bytes = (int)pp.get(0);

            ByteBuffer buffer = stack.malloc(bytes);
            checkCLError(clGetPlatformInfo(cl_platform_id, param_name, buffer, null));

            return memASCII(buffer, bytes - 1);
        }
    }

    static String getPlatformInfoStringUTF8(long cl_platform_id, int param_name) {
        try (MemoryStack stack = stackPush()) {
            PointerBuffer pp = stack.mallocPointer(1);
            checkCLError(clGetPlatformInfo(cl_platform_id, param_name, (ByteBuffer)null, pp));
            int bytes = (int)pp.get(0);

            ByteBuffer buffer = stack.malloc(bytes);
            checkCLError(clGetPlatformInfo(cl_platform_id, param_name, buffer, null));

            return memUTF8(buffer, bytes - 1);
        }
    }

    public static int getDeviceInfoInt(long cl_device_id, int param_name) {
        try (MemoryStack stack = stackPush()) {
            IntBuffer pl = stack.mallocInt(1);
            checkCLError(clGetDeviceInfo(cl_device_id, param_name, pl, null));
            return pl.get(0);
        }
    }

    public static long getDeviceInfoLong(long cl_device_id, int param_name) {
        try (MemoryStack stack = stackPush()) {
            LongBuffer pl = stack.mallocLong(1);
            checkCLError(clGetDeviceInfo(cl_device_id, param_name, pl, null));
            return pl.get(0);
        }
    }

    public static long getDeviceInfoPointer(long cl_device_id, int param_name) {
        try (MemoryStack stack = stackPush()) {
            PointerBuffer pp = stack.mallocPointer(1);
            checkCLError(clGetDeviceInfo(cl_device_id, param_name, pp, null));
            return pp.get(0);
        }
    }

    static String getDeviceInfoStringUTF8(long cl_device_id, int param_name) {
        try (MemoryStack stack = stackPush()) {
            PointerBuffer pp = stack.mallocPointer(1);
            checkCLError(clGetDeviceInfo(cl_device_id, param_name, (ByteBuffer)null, pp));
            int bytes = (int)pp.get(0);

            ByteBuffer buffer = stack.malloc(bytes);
            checkCLError(clGetDeviceInfo(cl_device_id, param_name, buffer, null));

            return memUTF8(buffer, bytes - 1);
        }
    }

    static int getProgramBuildInfoInt(long cl_program_id, long cl_device_id, int param_name) {
        try (MemoryStack stack = stackPush()) {
            IntBuffer pl = stack.mallocInt(1);
            checkCLError(clGetProgramBuildInfo(cl_program_id, cl_device_id, param_name, pl, null));
            return pl.get(0);
        }
    }

    static String getProgramBuildInfoStringASCII(long cl_program_id, long cl_device_id, int param_name) {
        try (MemoryStack stack = stackPush()) {
            PointerBuffer pp = stack.mallocPointer(1);
            checkCLError(clGetProgramBuildInfo(cl_program_id, cl_device_id, param_name, (ByteBuffer)null, pp));
            int bytes = (int)pp.get(0);

            ByteBuffer buffer = stack.malloc(bytes);
            checkCLError(clGetProgramBuildInfo(cl_program_id, cl_device_id, param_name, buffer, null));

            return memASCII(buffer, bytes - 1);
        }
    }

    public static void checkCLError(IntBuffer errcode) {
        checkCLError(errcode.get(errcode.position()));
    }

    public static void checkCLError(int errcode) {
        if (errcode != CL_SUCCESS) {
            throw new RuntimeException("OpenCL error " + stringFor_errorCode(errcode));
        }
    }

    static void printPlatformInfo(long platform, String param_name, int param) {
        System.out.println("\t" + param_name + " = " + getPlatformInfoStringUTF8(platform, param));
    }

    public static void printDeviceInfo(long device, String param_name, int param) {
        System.out.println("\t" + param_name + " = " + getDeviceInfoStringUTF8(device, param));
    }

    public static IntBuffer getIntBuffer(int[] inputArray) {
        IntBuffer aBuff = BufferUtils.createIntBuffer(inputArray.length);
        aBuff.put(inputArray);
        aBuff.rewind();
        return aBuff;
    }

    public static void getIntBuffer(long buffer, int[] returnBuffer){
        if(returnBuffer!=null) {
            IntBuffer resultBuff = BufferUtils.createIntBuffer(returnBuffer.length);
            int err = CL10.clEnqueueReadBuffer(openCLContext.getClQueue(), buffer, true, 0, resultBuff, null, null);
            OCLUtils.checkCLError(err);
            resultBuff.get(returnBuffer);
        }
    }

    public static void getIntBuffer(BufferGpu buffer, int[] returnBuffer){
        if(returnBuffer!=null) {
            IntBuffer resultBuff = BufferUtils.createIntBuffer(returnBuffer.length);
            int err = CL10.clEnqueueReadBuffer(openCLContext.getClQueue(), buffer.getMem(), true, 0, resultBuff, null, null);
            OCLUtils.checkCLError(err);
            resultBuff.get(returnBuffer);
        }
    }

    public static int[] getIntBuffer(long buffer, int size){
        IntBuffer resultBuff = BufferUtils.createIntBuffer(size);
        CL10.clEnqueueReadBuffer(openCLContext.getClQueue(), buffer, true, 0, resultBuff, null, null);
        int[] returnBuffer = new int[size];
        resultBuff.get(returnBuffer);
        return returnBuffer;
    }

    public static int[] getIntBuffer(BufferGpu buffer, int size){
        IntBuffer resultBuff = BufferUtils.createIntBuffer(size);
        CL10.clEnqueueReadBuffer(openCLContext.getClQueue(), buffer.getMem(), true, 0, resultBuff, null, null);
        int[] returnBuffer = new int[size];
        resultBuff.get(returnBuffer);
        return returnBuffer;
    }

    public static void printResults(long buffer, int size) {
        IntBuffer resultBuff = BufferUtils.createIntBuffer(size);
        CL10.clEnqueueReadBuffer(openCLContext.getClQueue(), buffer, true, 0, resultBuff, null, null);
        for (int i = 0; i < resultBuff.capacity(); i++) {
            System.out.println("result at " + i + " = " + resultBuff.get(i));
        }
    }

    public static void validateExpression(boolean exp, boolean check, String message){
        if (exp!=check){
            throw new RuntimeException(message);
        }
    }

    public static Vec4f[] getNormals(long normBuffer, int size){
        FloatBuffer resultBuff = BufferUtils.createFloatBuffer(size * Integer.BYTES);
        CL10.clEnqueueReadBuffer(openCLContext.getClQueue(), normBuffer, true, 0, resultBuff, null, null);
        Vec4f[] normalsBuffer = new Vec4f[size];
        for (int i = 0; i < size; i++) {
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

    public static void getNormals(BufferGpu normBuffer, Vec4f[] normalsBuffer){
        if(normalsBuffer!=null) {
            FloatBuffer resultBuff = BufferUtils.createFloatBuffer(normalsBuffer.length * Integer.BYTES);
            CL10.clEnqueueReadBuffer(openCLContext.getClQueue(), normBuffer.getMem(), true, 0, resultBuff, null, null);
            for (int i = 0; i < normalsBuffer.length; i++) {
                int index = i * 4;
                Vec4f normal = new Vec4f();
                normal.x = resultBuff.get(index + 0);
                normal.y = resultBuff.get(index + 1);
                normal.z = resultBuff.get(index + 2);
                normal.w = resultBuff.get(index + 3);
                normalsBuffer[i] = normal;
            }
        }
    }

    static class QEFDataT{
        float[] mat3x3_tri_ATA = new float[6];
        float[] pad = new float[2];
        Vec4f ATb = new Vec4f();
        Vec4f massPoint = new Vec4f();
    }

    public static void getQEFData(BufferGpu buffer, QEFData[] qefData){
        if(qefData!=null) {
            ByteBuffer byteBuffer = BufferUtils.createByteBuffer(qefData.length * Float.BYTES * 16);
            int err = CL10.clEnqueueReadBuffer(openCLContext.getClQueue(), buffer.getMem(), true, 0, byteBuffer, null, null);
            OCLUtils.checkCLError(err);
            FloatBuffer resultBuff = byteBuffer.asFloatBuffer();
            for (int i = 0; i < qefData.length; i++) {
                int index = i * 16;
                QEFData q = new QEFData(new LevenQefSolver());
                q.mat3x3_tri_ATA[0] = resultBuff.get(index + 0);
                q.mat3x3_tri_ATA[1] = resultBuff.get(index + 1);
                q.mat3x3_tri_ATA[2] = resultBuff.get(index + 2);
                q.mat3x3_tri_ATA[3] = resultBuff.get(index + 3);
                q.mat3x3_tri_ATA[4] = resultBuff.get(index + 4);
                q.mat3x3_tri_ATA[5] = resultBuff.get(index + 5);
//            q.pad[0] = resultBuff.get(index+6);
//            q.pad[1] = resultBuff.get(index+7);
                q.atb.x = resultBuff.get(index + 8);
                q.atb.y = resultBuff.get(index + 9);
                q.atb.z = resultBuff.get(index + 10);
                q.atb.w = resultBuff.get(index + 11);
                q.massPoint.x = resultBuff.get(index + 12);
                q.massPoint.y = resultBuff.get(index + 13);
                q.massPoint.z = resultBuff.get(index + 14);
                q.massPoint.w = resultBuff.get(index + 15);
                qefData[i] = q;
            }
        }
    }

    public static MeshVertex[] getVertexBuffer(BufferGpu buffer, int numVertices){
        FloatBuffer resultBuff = BufferUtils.createFloatBuffer(4 * 3 * numVertices);
        int err = CL10.clEnqueueReadBuffer(openCLContext.getClQueue(), buffer.getMem(), true, 0, resultBuff, null, null);
        OCLUtils.checkCLError(err);
        MeshVertex[] meshVertexBuffer = new MeshVertex[numVertices];
        for (int i = 0; i < numVertices; i++) {
            int index = i * 12;
            MeshVertex meshVertex = new MeshVertex();
            Vec4f pos = new Vec4f();
            pos.x = resultBuff.get(index+0);
            pos.y = resultBuff.get(index+1);
            pos.z = resultBuff.get(index+2);
            pos.w = resultBuff.get(index+3);
            meshVertex.setPos(pos.getVec3f());
            Vec4f normal = new Vec4f();
            normal.x = resultBuff.get(index+4);
            normal.y = resultBuff.get(index+5);
            normal.z = resultBuff.get(index+6);
            normal.w = resultBuff.get(index+7);
            meshVertex.setNormal(normal.getVec3f());
            Vec4f color = new Vec4f();
            color.x = resultBuff.get(index+8);
            color.y = resultBuff.get(index+9);
            color.z = resultBuff.get(index+10);
            color.w = resultBuff.get(index+11);
            meshVertex.setColor(color.getVec3f());
            meshVertexBuffer[i] = meshVertex;
        }
        return meshVertexBuffer;
    }

    public static IntBuffer getTrianglesAsIntBuffer(long buffer, int numTriangles){
        //ByteBuffer byteBuffer = BufferUtils.createByteBuffer(Integer.BYTES * numTriangles * 3);
        IntBuffer resultBuff = BufferUtils.createIntBuffer(numTriangles * 3);
        int err = CL10.clEnqueueReadBuffer(openCLContext.getClQueue(), buffer, true, 0, resultBuff, null, null);
        OCLUtils.checkCLError(err);
        //IntBuffer resultBuff = byteBuffer.asIntBuffer();
        return resultBuff;
    }

    public static void getListSeamNodesTriangles(BufferGpu buffer, int bufSize, Vec3i chunkMin, Vec3f color, int chunkSize, List<OctreeNode> seamNodes){
        ByteBuffer byteBuff = BufferUtils.createByteBuffer(Float.BYTES * 4 * 3 * bufSize);
        int err = CL10.clEnqueueReadBuffer(openCLContext.getClQueue(), buffer.getMem(), true, 0, byteBuff, null, null);
        OCLUtils.checkCLError(err);

        for (int i = 0; i < bufSize; i++) {
            int index = i * 12 * 4;
            Vec4i localspaceMin = new Vec4i();
            localspaceMin.x = byteBuff.getInt(index+0);
            localspaceMin.y = byteBuff.getInt(index+4);
            localspaceMin.z = byteBuff.getInt(index+8);
            localspaceMin.w = byteBuff.getInt(index+12);
            Vec3i min = localspaceMin.mul(chunkSize).add(chunkMin);
            PointerBasedOctreeNode node = new PointerBasedOctreeNode(min, chunkSize, OctreeNodeType.Node_Leaf);
            node.nodeNum = localspaceMin;
            node.corners = localspaceMin.w;

            OctreeDrawInfo drawInfo = new OctreeDrawInfo();
            Vec4f position = new Vec4f();
            position.x = byteBuff.getFloat(index+16);
            position.y = byteBuff.getFloat(index+20);
            position.z = byteBuff.getFloat(index+24);
            position.w = byteBuff.getFloat(index+28);
            drawInfo.position = position.getVec3f();
            drawInfo.color = color;

            Vec4f normal = new Vec4f();
            normal.x = byteBuff.getFloat(index+32);
            normal.y = byteBuff.getFloat(index+36);
            normal.z = byteBuff.getFloat(index+40);
            normal.w = byteBuff.getFloat(index+44);
            drawInfo.averageNormal = normal.getVec3f();

            node.drawInfo = drawInfo;
            seamNodes.add(node);
        }
    }

    public static long getMemoryAccessFlags(MemAccess ma) {
        switch (ma) {
            case READ_ONLY: return CL10.CL_MEM_READ_ONLY;
            case WRITE_ONLY: return CL10.CL_MEM_WRITE_ONLY;
            case READ_WRITE: return CL10.CL_MEM_READ_WRITE;
            default: throw new IllegalArgumentException("Unknown memory access: "+ma);
        }
    }
}
