package dc.impl.opencl;

import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import java.nio.IntBuffer;

import static org.lwjgl.opencl.CL10.*;

public final class ScanOpenCLService {
    private final ComputeContext ctx;
    private final long scanProgram;
    private final BufferGpuService bufferGpuService;

    public ScanOpenCLService(ComputeContext ctx, long scanProgram, BufferGpuService bufferGpuService) {
        this.scanProgram = scanProgram;
        this.ctx = ctx;
        this.bufferGpuService = bufferGpuService;
    }

    public int exclusiveScan(BufferGpu data, BufferGpu scan, int count){
        scan(scanProgram, data, scan, count, true);

        IntBuffer lastValueBuff = BufferUtils.createIntBuffer(1);
        CL10.clEnqueueReadBuffer(ctx.getClQueue(), data.getMem(), false, (count - 1)*4, lastValueBuff, null, null);
        IntBuffer lastScanValueBuff = BufferUtils.createIntBuffer(1);
        CL10.clEnqueueReadBuffer(ctx.getClQueue(), scan.getMem(), true, (count - 1)*4, lastScanValueBuff, null, null);

        int lastValue = lastValueBuff.get(0);
        int lastScanValue = lastScanValueBuff.get(0);

        CL10.clFinish(ctx.getClQueue());
        return lastValue + lastScanValue;
    }

    private void scan(long scanProgram, BufferGpu data, BufferGpu scanData, int count, boolean exclusive) {
        int blockSize = pickScanBlockSize(count);
        int blockCount = count / blockSize;
        if (blockCount * blockSize < count) {
            blockCount++;
        }

        BufferGpu blockSumsBuffer = bufferGpuService.create("scanBlockSums", blockCount * 4, MemAccess.READ_WRITE);
        fillBufferInt(scanProgram, ctx.getClQueue(), blockSumsBuffer, blockCount, 0, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        String scanKernelName = exclusive ? "ExclusiveLocalScan" : "InclusiveLocalScan";
        long localScanKernel = clCreateKernel(scanProgram, scanKernelName, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        clSetKernelArg1p(localScanKernel, 0, blockSumsBuffer.getMem());
        clSetKernelArg(localScanKernel, 1, blockSize * 4);
        clSetKernelArg1i(localScanKernel, 2, blockSize);
        clSetKernelArg1i(localScanKernel, 3, count);
        clSetKernelArg1p(localScanKernel, 4, data.getMem());
        clSetKernelArg1p(localScanKernel, 5, scanData.getMem());

        final int dimensions = 1;
        PointerBuffer globWorkSize = BufferUtils.createPointerBuffer(dimensions).put(0, blockCount * blockSize);
        PointerBuffer locWorkSize = BufferUtils.createPointerBuffer(dimensions).put(0, blockSize);
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), localScanKernel, dimensions, null, globWorkSize, locWorkSize, null, null);
        OCLUtils.checkCLError(err);

        if (blockCount > 1) {
            scan(scanProgram, blockSumsBuffer, blockSumsBuffer, blockCount, false);
            long writeOutputKernel = clCreateKernel(scanProgram, "WriteScannedOutput", ctx.getErrcode_ret());
            OCLUtils.checkCLError(ctx.getErrcode_ret());
            clSetKernelArg1p(writeOutputKernel, 0, scanData.getMem());
            clSetKernelArg1p(writeOutputKernel, 1, blockSumsBuffer.getMem());
            clSetKernelArg1i(writeOutputKernel, 2, count);

            err = clEnqueueNDRangeKernel(ctx.getClQueue(), writeOutputKernel, dimensions, locWorkSize, globWorkSize, locWorkSize, null, null);
            OCLUtils.checkCLError(err);
            CL10.clReleaseKernel(writeOutputKernel);
        }
        err = CL10.clReleaseKernel(localScanKernel);
        OCLUtils.checkCLError(err);
        bufferGpuService.release(blockSumsBuffer);
    }

    private void fillBufferInt(long scanProgram, long clQueue, BufferGpu buffer, int count, int value, IntBuffer errcode_ret) {
        long fillBufferKernel = clCreateKernel(scanProgram, "FillBufferInt", errcode_ret);
        OCLUtils.checkCLError(errcode_ret);

        clSetKernelArg1p(fillBufferKernel, 0, buffer.getMem());
        clSetKernelArg1i(fillBufferKernel, 1, value);

        final int dimensions = 1;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions);
        globalWorkSize.put(0, count);

        int errcode = clEnqueueNDRangeKernel(clQueue, fillBufferKernel,
                dimensions, null, globalWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);
        CL10.clReleaseKernel(fillBufferKernel);
    }

    private int pickScanBlockSize(int count) {
        if(count == 0)		{ return 0; }
        else if(count <= 1)	 { return 1; }
        else if(count <= 2)	 { return 2; }
        else if(count <= 4)	 { return 4; }
        else if(count <= 8)	 { return 8; }
        else if(count <= 16)	{ return 16; }
        else if(count <= 32)	{ return 32; }
        else if(count <= 64)	{ return 64; }
        else if(count <= 128) { return 128; }
        else { return 256; }
    }
}