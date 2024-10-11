package test;

import dc.impl.MeshGenerationContext;
import dc.impl.nonused.opencl.*;
import dc.utils.VoxelHelperUtils;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import static org.lwjgl.opencl.CL10.*;

public class CuckooServiceTest {
    public static void main(String[] args) {
        ComputeContext ctx = OCLUtils.getOpenCLContext();
        BufferGpuService bufferGpuService = new BufferGpuService(ctx);
        MeshGenerationContext meshGen = new MeshGenerationContext(64);
        KernelsHolder kernelHolder = new KernelsHolder(ctx);
        kernelHolder.buildKernel(KernelNames.SCAN, null);
        kernelHolder.buildKernel(KernelNames.CUCKOO, VoxelHelperUtils.getCuckooBuildOptions(meshGen));
        kernelHolder.buildKernel(KernelNames.TEST_CUCKOO, VoxelHelperUtils.getCuckooBuildOptions(meshGen));

        ScanOpenCLService scanService = new ScanOpenCLService(ctx, kernelHolder.getKernel(KernelNames.SCAN), bufferGpuService);

        int KEY_COUNT = 100;
        CuckooHashOpenCLService cuckooHashService = new CuckooHashOpenCLService(ctx, meshGen,
                scanService, kernelHolder, KEY_COUNT, bufferGpuService);
        int[] keys = new int[KEY_COUNT];
        for (int i = 0; i < 100; i++) {
            keys[i] = i;
        }

        BufferGpu buffer = bufferGpuService.create("cuckooTestBuffer", OCLUtils.getIntBuffer(keys), CL10.CL_MEM_READ_ONLY | CL10.CL_MEM_COPY_HOST_PTR);
        int result = cuckooHashService.insertKeys(buffer, KEY_COUNT);
        OCLUtils.validateExpression(result == CL10.CL_SUCCESS, true, "CuckooHash error");

        int resultBufferSize = 10;
        int inputArgument = 53;
        BufferGpu resultOutBuffer = bufferGpuService.create("resultOutBuffer", resultBufferSize * 4, CL10.CL_MEM_READ_WRITE);

        long testCuckooKernel = clCreateKernel(kernelHolder.getKernel(KernelNames.TEST_CUCKOO), "testCuckoo", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        clSetKernelArg1i(testCuckooKernel, 0, inputArgument);
        clSetKernelArg1p(testCuckooKernel, 1, resultOutBuffer.getMem());
        clSetKernelArg1p(testCuckooKernel, 2, cuckooHashService.getTable().getMem());
        clSetKernelArg1p(testCuckooKernel, 3, cuckooHashService.getStash().getMem());
        clSetKernelArg1i(testCuckooKernel, 4, cuckooHashService.getPrime());
        clSetKernelArg1p(testCuckooKernel, 5, cuckooHashService.getHashParams().getMem());
        clSetKernelArg1i(testCuckooKernel, 6, cuckooHashService.getStashUsed());

        final int dimensions = 1;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions);
        globalWorkSize.put(0, resultBufferSize);
        int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), testCuckooKernel, dimensions, null, globalWorkSize, null,
                null, null);
        OCLUtils.checkCLError(errcode);

        int[] outputResult = OCLUtils.getIntBuffer(resultOutBuffer, resultBufferSize);
        OCLUtils.validateExpression(outputResult[0] == inputArgument, true, "Cuckoo hash error");

        bufferGpuService.release(resultOutBuffer);
        int ret = CL10.clReleaseCommandQueue(ctx.getClQueue());
        OCLUtils.checkCLError(ret);
        ret = CL10.clReleaseProgram(kernelHolder.getKernel(KernelNames.CUCKOO));
        OCLUtils.checkCLError(ret);
        ret = CL10.clReleaseProgram(kernelHolder.getKernel(KernelNames.SCAN));
        OCLUtils.checkCLError(ret);
        ret = CL10.clReleaseProgram(kernelHolder.getKernel(KernelNames.TEST_CUCKOO));
        OCLUtils.checkCLError(ret);
    }
}
