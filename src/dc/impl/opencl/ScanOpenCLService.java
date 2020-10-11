package dc.impl.opencl;

import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import java.nio.IntBuffer;

import static org.lwjgl.opencl.CL10.*;

public final class ScanOpenCLService {
    private final ComputeContext ctx;

    public ScanOpenCLService(ComputeContext ctx) {
        this.ctx = ctx;
    }

    public int exclusiveScan(long scanProgram, long data, long scan, int count){
        scan(scanProgram, data, scan, count, true);

        IntBuffer lastValueBuff = BufferUtils.createIntBuffer(1);
        CL10.clEnqueueReadBuffer(ctx.getClQueue(), data, false, (count - 1)*4, lastValueBuff, null, null);
        IntBuffer lastScanValueBuff = BufferUtils.createIntBuffer(1);
        CL10.clEnqueueReadBuffer(ctx.getClQueue(), scan, true, (count - 1)*4, lastScanValueBuff, null, null);

        int lastValue = lastValueBuff.get(0);
        int lastScanValue = lastScanValueBuff.get(0);

        //getBuffer(openCLContext, scan, count);

        CL10.clFinish(ctx.getClQueue());
        return lastValue + lastScanValue;
    }

    private void scan(long scanProgram, long data, long scanData, int count, boolean exclusive) {
        int blockSize = pickScanBlockSize(count);
        int blockCount = count / blockSize;
        if (blockCount * blockSize < count) {
            blockCount++;
        }

        long blockSumsBuffer = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, blockCount * 4, ctx.getErrcode_ret());
        fillBufferInt(scanProgram, ctx.getClQueue(), blockSumsBuffer, blockCount, 0, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        //getBuffer(openCLContext, blockSumsBuffer, blockCount);

        long blockSizeScratchBuffer = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, blockSize * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        String scanKernelName = exclusive ? "ExclusiveLocalScan" : "InclusiveLocalScan";
        long localScanKernel = clCreateKernel(scanProgram, scanKernelName, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        clSetKernelArg1p(localScanKernel, 0, blockSumsBuffer);
        clSetKernelArg1p(localScanKernel, 1, blockSizeScratchBuffer); //clSetKernelArg2i(localScanKernel, 1, blockSize * 4, 0);
        clSetKernelArg1i(localScanKernel, 2, blockSize);
        clSetKernelArg1i(localScanKernel, 3, count);
        clSetKernelArg1p(localScanKernel, 4, data);
        clSetKernelArg1p(localScanKernel, 5, scanData);

        final int dimensions = 1;
        PointerBuffer globWorkSize = BufferUtils.createPointerBuffer(dimensions).put(0, blockCount * blockSize);
        PointerBuffer locWorkSize = BufferUtils.createPointerBuffer(dimensions).put(0, blockSize);
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), localScanKernel, dimensions, null, globWorkSize, locWorkSize, null, null);
        OCLUtils.checkCLError(err);

        //getIntBuffer(scanData, count);
        getIntBuffer(blockSizeScratchBuffer, blockSize);
        CL10.clReleaseMemObject(blockSizeScratchBuffer);

        if (blockCount > 1) {
            scan(scanProgram, blockSumsBuffer, blockSumsBuffer, blockCount, false);
            long writeOutputKernel = clCreateKernel(scanProgram, "WriteScannedOutput", ctx.getErrcode_ret());
            OCLUtils.checkCLError(ctx.getErrcode_ret());
            clSetKernelArg1p(writeOutputKernel, 0, scanData);
            clSetKernelArg1p(writeOutputKernel, 1, blockSumsBuffer);
            clSetKernelArg1i(writeOutputKernel, 2, count);

            err = clEnqueueNDRangeKernel(ctx.getClQueue(), writeOutputKernel, dimensions, locWorkSize, globWorkSize, locWorkSize, null, null);
            OCLUtils.checkCLError(err);
            CL10.clReleaseKernel(writeOutputKernel);
        }
        CL10.clReleaseKernel(localScanKernel);
        CL10.clReleaseMemObject(blockSumsBuffer);
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

    private void printResults(long buffer, int size) {
        IntBuffer resultBuff = BufferUtils.createIntBuffer(size);
        CL10.clEnqueueReadBuffer(ctx.getClQueue(), buffer, true, 0, resultBuff, null, null);
        for (int i = 0; i < resultBuff.capacity(); i++) {
            System.out.println("result at " + i + " = " + resultBuff.get(i));
        }
    }

    private void fillBufferInt(long scanProgram, long clQueue, long buffer, int count, int value, IntBuffer errcode_ret) {
        long fillBufferKernel = clCreateKernel(scanProgram, "FillBufferInt", errcode_ret);
        OCLUtils.checkCLError(errcode_ret);

        clSetKernelArg1p(fillBufferKernel, 0, buffer);
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