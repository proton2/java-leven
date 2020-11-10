package dc.impl.opencl;

import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import java.nio.IntBuffer;
import java.util.Random;

import static org.lwjgl.opencl.CL10.*;
/*
    gived from https://developer.nvidia.com/opencl
 */
public final class NVidiaScanOpenCLService {
    private final ComputeContext ctx;
    private final KernelsHolder kernelsHolder;
    private final BufferGpuService bufferGpuService;
    final int WORKGROUP_SIZE = 256;
    final int MAX_BATCH_ELEMENTS = 64 * 1048576;
    final int MIN_SHORT_ARRAY_SIZE = 4;
    final int MAX_SHORT_ARRAY_SIZE = 4 * WORKGROUP_SIZE;
    final int MIN_LARGE_ARRAY_SIZE = 8 * WORKGROUP_SIZE;
    final int MAX_LARGE_ARRAY_SIZE = 4 * WORKGROUP_SIZE * WORKGROUP_SIZE;

    BufferGpu d_Buffer;
    long ckScanExclusiveLocal1, ckScanExclusiveLocal2, ckUniformUpdate;
    BufferGpu d_Input, d_Output;
    int N = 13 * 1048576 / 2;
    int[] h_Input, h_OutputCPU, h_OutputGPU;

    public NVidiaScanOpenCLService(ComputeContext ctx, KernelsHolder kernelsHolder, BufferGpuService bufferGpuService) {
        this.ctx = ctx;
        this.kernelsHolder = kernelsHolder;
        this.bufferGpuService = bufferGpuService;
        h_Input = new int[N];
        h_OutputCPU = new int[N];
        h_OutputGPU = new int[N];

        Random rand = new Random(System.currentTimeMillis());
        int randValue = rand.nextInt(2009);
        for(int i = 0; i < N; i++)
            h_Input[i] = randValue;
    }

    public void scan(){
        ckScanExclusiveLocal1 = clCreateKernel(kernelsHolder.getKernel(KernelNames.NVIDIA_SCAN), "scanExclusiveLocal1", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        ckScanExclusiveLocal2 = clCreateKernel(kernelsHolder.getKernel(KernelNames.NVIDIA_SCAN), "scanExclusiveLocal2", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        ckUniformUpdate = clCreateKernel(kernelsHolder.getKernel(KernelNames.NVIDIA_SCAN), "uniformUpdate", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        int szScanExclusiveLocal1 = (int)OCLUtils.getMaxWorkGroupSize(ckScanExclusiveLocal1);
        int szScanExclusiveLocal2 = (int)OCLUtils.getMaxWorkGroupSize(ckScanExclusiveLocal2);
        int szUniformUpdate = (int)OCLUtils.getMaxWorkGroupSize(ckUniformUpdate);
        if( (szScanExclusiveLocal1 < WORKGROUP_SIZE) || (szScanExclusiveLocal2 < WORKGROUP_SIZE) || (szUniformUpdate < WORKGROUP_SIZE) ){
            System.out.println("\nERROR !!! Minimum work-group size %u required by this application is not supported on this device.\n\n" + WORKGROUP_SIZE);
            closeScan();
        }
        d_Buffer = bufferGpuService.create("d_Buffer", (MAX_BATCH_ELEMENTS / (4 * WORKGROUP_SIZE)) * 4, CL_MEM_READ_WRITE);
        d_Input = bufferGpuService.create("d_Input", OCLUtils.getIntBuffer(h_Input), CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR);
        d_Output = bufferGpuService.create("d_Output", N * 4, CL_MEM_READ_WRITE);

        boolean globalFlag = true; // init pass/fail flag to pass
        int iCycles = 100;
        for(int arrayLength = MIN_SHORT_ARRAY_SIZE; arrayLength <= MAX_SHORT_ARRAY_SIZE; arrayLength *= 2) {
            System.out.println("Running scan for %u elements (%u arrays)...\n" + arrayLength + N / arrayLength);
            CL10.clFinish(ctx.getClQueue());
            for (int i = 0; i<iCycles; i++) {
                scanExclusiveShort(d_Output, d_Input, N / arrayLength, arrayLength);
            }
            CL10.clFinish(ctx.getClQueue());

            System.out.println("Validating the results...\n");
            System.out.println(" ...reading back OpenCL memory\n");
            OCLUtils.getIntBuffer(d_Output, h_OutputGPU);

            System.out.println(" ...scanExclusiveHost()\n");
            scanExclusiveHost(h_OutputCPU, h_Input, N / arrayLength, arrayLength);

            // Compare GPU results with CPU results and accumulate error for this test
            System.out.println(" ...comparing the results\n");
            boolean localFlag = true;
            for(int i = 0; i < N; i++) {
                if(h_OutputCPU[i] != h_OutputGPU[i]) {
                    localFlag = false;
                    break;
                }
            }
            // Log message on individual test result, then accumulate to global flag
            System.out.println(" ...Results %s\n\n" + ((localFlag) ? "Match" : "DON'T Match !!!"));
            globalFlag = globalFlag && localFlag;
        }

        System.out.println("*** Running GPU scan for large arrays (%d identical iterations)...\n\n" + iCycles);
        for(int arrayLength = MIN_LARGE_ARRAY_SIZE; arrayLength <= MAX_LARGE_ARRAY_SIZE; arrayLength *= 2) {
            System.out.println("Running scan for %u elements (%u arrays)...\n" + arrayLength + N / arrayLength);
            clFinish(ctx.getClQueue());
            for (int i = 0; i<iCycles; i++) {
                scanExclusiveLarge(d_Output, d_Input, N / arrayLength, arrayLength);
            }
            clFinish(ctx.getClQueue());

            System.out.println("Validating the results...\n");
            System.out.println(" ...reading back OpenCL memory\n");

            OCLUtils.getIntBuffer(d_Output, h_OutputGPU);

            System.out.println(" ...scanExclusiveHost()\n");
            scanExclusiveHost(h_OutputCPU, h_Input, N / arrayLength, arrayLength);

            // Compare GPU results with CPU results and accumulate error for this test
            System.out.println(" ...comparing the results\n");
            boolean localFlag = true;
            for(int i = 0; i < N; i++) {
                if(h_OutputCPU[i] != h_OutputGPU[i]) {
                    localFlag = false;
                    break;
                }
            }
            // Log message on individual test result, then accumulate to global flag
            System.out.println(" ...Results %s\n\n" + ((localFlag) ? "Match" : "DON'T Match !!!"));
            globalFlag = globalFlag && localFlag;
        }

        System.out.println("Shutting down...\n");
        //Release kernels and program
        closeScan();

        bufferGpuService.release(d_Output);
        bufferGpuService.release(d_Input);

        int ciErrNum = clReleaseCommandQueue(ctx.getClQueue());
        OCLUtils.checkCLError(ciErrNum);
        ciErrNum = clReleaseContext(ctx.getClContext());
        OCLUtils.checkCLError(ciErrNum);
    }

    public int[] getIntBuffer(long buffer, int size){
        IntBuffer resultBuff = BufferUtils.createIntBuffer(size);
        CL10.clEnqueueReadBuffer(ctx.getClQueue(), buffer, true, 0, resultBuff, null, null);
        int[] returnBuffer = new int[size];
        resultBuff.get(returnBuffer);
        int count = 0;
        for (int i=0; i<size; i++){
            if(returnBuffer[i]==1){
                ++count;
            }
        }
        System.out.println(count);
        return returnBuffer;
    }

    void scanExclusiveShort(BufferGpu d_Dst, BufferGpu d_Src, int batchSize, int arrayLength){
        //
        int log2L = 0;
        int factorizationRemainder = factorRadix2(log2L, arrayLength);
        OCLUtils.validateExpression(factorizationRemainder == 1, true, "Check power-of-two factorization error");
        OCLUtils.validateExpression( (arrayLength >= MIN_SHORT_ARRAY_SIZE) && (arrayLength <= MAX_SHORT_ARRAY_SIZE), true, "Check supported size range error");
        OCLUtils.validateExpression( (batchSize * arrayLength) <= MAX_BATCH_ELEMENTS, true, "Check total batch size limit error");
        OCLUtils.validateExpression( (batchSize * arrayLength) % (4 * WORKGROUP_SIZE) == 0, true, "Check all work-groups to be fully packed with data error");

        scanExclusiveLocal1(d_Dst, d_Src, batchSize, arrayLength);
    }

    void scanExclusiveLarge(BufferGpu d_Dst, BufferGpu d_Src, int batchSize, int arrayLength) {
        int log2L = 0;
        int factorizationRemainder = factorRadix2(log2L, arrayLength);
        OCLUtils.validateExpression(factorizationRemainder == 1, true, "Check power-of-two factorization");
        OCLUtils.validateExpression((arrayLength >= MIN_LARGE_ARRAY_SIZE) && (arrayLength <= MAX_LARGE_ARRAY_SIZE), true, "Check supported size range");
        OCLUtils.validateExpression((batchSize * arrayLength) <= MAX_BATCH_ELEMENTS, true, "Check total batch size limit");

        scanExclusiveLocal1(d_Dst, d_Src, (batchSize * arrayLength) / (4 * WORKGROUP_SIZE), 4 * WORKGROUP_SIZE);
        scanExclusiveLocal2(d_Buffer, d_Dst, d_Src, batchSize, arrayLength / (4 * WORKGROUP_SIZE));
        uniformUpdate(d_Dst, d_Buffer, (batchSize * arrayLength) / (4 * WORKGROUP_SIZE));
    }

    void scanExclusiveLocal1(BufferGpu d_Dst, BufferGpu d_Src, int n, int size){
        clSetKernelArg1p(ckScanExclusiveLocal1, 0, d_Dst.getMem());
        clSetKernelArg1p(ckScanExclusiveLocal1, 1, d_Src.getMem());
        clSetKernelArg(ckScanExclusiveLocal1, 2, 2 * WORKGROUP_SIZE * 4);
        clSetKernelArg1i(ckScanExclusiveLocal1, 3, size);

        PointerBuffer localWorkSize = BufferUtils.createPointerBuffer(1).put(0, WORKGROUP_SIZE);
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(1).put(0, (n * size) / 4);
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), ckScanExclusiveLocal1, 1, null, globalWorkSize, localWorkSize,null,null);
        OCLUtils.checkCLError(err);
    }

    void scanExclusiveLocal2(BufferGpu d_Buffer, BufferGpu d_Dst, BufferGpu d_Src, int n, int size){
        int elements = n * size;
        clSetKernelArg1p(ckScanExclusiveLocal2, 0, d_Buffer.getMem());
        clSetKernelArg1p(ckScanExclusiveLocal2, 1, d_Dst.getMem());
        clSetKernelArg1p(ckScanExclusiveLocal2, 2, d_Src.getMem());
        clSetKernelArg(ckScanExclusiveLocal2, 3, 2 * WORKGROUP_SIZE * 4);
        clSetKernelArg1i(ckScanExclusiveLocal2, 4, elements);
        clSetKernelArg1i(ckScanExclusiveLocal2, 5, size);

        PointerBuffer localWorkSize = BufferUtils.createPointerBuffer(1).put(0, WORKGROUP_SIZE);
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(1).put(0, iSnapUp(elements, WORKGROUP_SIZE));

        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), ckScanExclusiveLocal2, 1, null, globalWorkSize, localWorkSize, null, null);
        OCLUtils.checkCLError(err);
    }

    void uniformUpdate(BufferGpu d_Dst, BufferGpu d_Buffer, int n){
        clSetKernelArg1p(ckUniformUpdate, 0, d_Dst.getMem());
        clSetKernelArg1p(ckUniformUpdate, 1, d_Buffer.getMem());

        PointerBuffer localWorkSize = BufferUtils.createPointerBuffer(1).put(0, WORKGROUP_SIZE);
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(1).put(0, n * WORKGROUP_SIZE);

        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), ckUniformUpdate, 1, null, globalWorkSize, localWorkSize, null, null);
        OCLUtils.checkCLError(err);
    }

    void scanExclusiveHost(int[] dst, int[] src, int batchSize, int arrayLength){
        int srcOffcet=0, dstOffcet=0;
        for(int i = 0; i < batchSize; i++, srcOffcet += arrayLength, dstOffcet += arrayLength){
            dst[dstOffcet] = 0;
            for(int j = 1; j < arrayLength; j++)
                dst[j + dstOffcet] = src[(j + srcOffcet) - 1] + dst[(j + dstOffcet) - 1];
        }
    }

    static int iSnapUp(int dividend, int divisor){
        return ((dividend % divisor) == 0) ? dividend : (dividend - dividend % divisor + divisor);
    }

    static int factorRadix2(int log2L, int L){
        if(L==0){
            log2L = 0;
            return 0;
        }else{
            for(log2L = 0; (L & 1) == 0; L >>= 1, log2L++);
            return L;
        }
    }

    public void closeScan(){
        CL10.clReleaseKernel(ckScanExclusiveLocal1);
        CL10.clReleaseKernel(ckScanExclusiveLocal2);
        CL10.clReleaseKernel(ckUniformUpdate);
        bufferGpuService.release(d_Buffer);
        CL10.clReleaseProgram(kernelsHolder.getKernel(KernelNames.NVIDIA_SCAN));
    }
}