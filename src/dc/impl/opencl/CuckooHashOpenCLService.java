package dc.impl.opencl;

import dc.impl.MeshGenerationContext;
import dc.utils.VoxelHelperUtils;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import java.io.File;
import java.nio.IntBuffer;
import java.nio.file.Paths;

import static org.lwjgl.opencl.CL10.*;

public class CuckooHashOpenCLService {
    private long table, stash, hashParams;
    private int prime;
    private int stashUsed = 0;                    // int rather than bool as bools seem somewhat iffy in OpenCL
    private int insertedKeys = 0;

    public long getTable() {
        return table;
    }

    public long getStash() {
        return stash;
    }

    public long getHashParams() {
        return hashParams;
    }

    public int getPrime() {
        return prime;
    }

    public int getStashUsed() {
        return stashUsed;
    }

    public int getInsertedKeys() {
        return insertedKeys;
    }

    int MIN_TABLE_SIZE = 2048;
    private final ComputeContext ctx;
    private final KernelsHolder kernelProgram;
    private final ScanOpenCLService scanOpenCLService;
    private final MeshGenerationContext meshGen;

    public CuckooHashOpenCLService(ComputeContext ctx, MeshGenerationContext meshGen, ScanOpenCLService scanService,
                                   KernelsHolder kernelHolder, int tableSize) {
        this.meshGen = meshGen;
        this.scanOpenCLService = scanService;
        this.ctx = ctx;
        this.kernelProgram = kernelHolder;
        prime = findNextPrime(Math.max(MIN_TABLE_SIZE, tableSize * 2));
        table = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, prime * 8, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        fillBufferLong(kernelProgram.getKernel(KernelNames.CUCKOO), ctx.getClQueue(), table, prime, meshGen.CUCKOO_EMPTY_VALUE, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        stash = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, meshGen.CUCKOO_STASH_SIZE * 8, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        fillBufferLong(kernelProgram.getKernel(KernelNames.CUCKOO), ctx.getClQueue(), stash, meshGen.CUCKOO_STASH_SIZE, meshGen.CUCKOO_EMPTY_VALUE, ctx.getErrcode_ret());

        int[] params = initHashValues();
        hashParams = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_ONLY | CL10.CL_MEM_COPY_HOST_PTR, OCLUtils.getIntBuffer(params), ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
    }

    private int[] initHashValues(){
        int[] params = new int[meshGen.CUCKOO_HASH_FN_COUNT * 2];
        for (int i = 0; i < meshGen.CUCKOO_HASH_FN_COUNT; i++) {
            params[i * 2 + 0] = Rnd.get(1 << 15, 1 << 30);
            params[i * 2 + 1] = Rnd.get(1 << 15, 1 << 30);
        }
        return params;
    }

    public int insertKeys(long d_keys, int count) {
        long d_inserted = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, count * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        long d_stashUsed = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, count * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        long d_insertedScan = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, count * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        long d_stashUsedScan = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, count * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        long k_InsertKeys = clCreateKernel(kernelProgram.getKernel(KernelNames.CUCKOO), "Cuckoo_InsertKeys", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        int numRetries = 0;
        int insertedCount = 0, stashUsedCount = 0;
        do {
            if (insertedCount > 0) {
                System.out.printf("Cuckoo: insert keys failed (failed to insert %d key from %d). Retry #%d...\n%n", count - insertedCount, count, ++numRetries);
                int[] params = initHashValues();
                int errcode = CL10.clEnqueueWriteBuffer(ctx.getClQueue(), hashParams, false, 0, OCLUtils.getIntBuffer(params), null, null);
                OCLUtils.checkCLError(errcode);
                fillBufferLong(kernelProgram.getKernel(KernelNames.CUCKOO), ctx.getClQueue(), table, prime, meshGen.CUCKOO_EMPTY_VALUE, ctx.getErrcode_ret());
                fillBufferLong(kernelProgram.getKernel(KernelNames.CUCKOO), ctx.getClQueue(), stash, meshGen.CUCKOO_STASH_SIZE, meshGen.CUCKOO_EMPTY_VALUE, ctx.getErrcode_ret());
            }

            clSetKernelArg1p(k_InsertKeys, 0, d_keys);
            clSetKernelArg1p(k_InsertKeys, 1, table);
            clSetKernelArg1p(k_InsertKeys, 2, stash);
            clSetKernelArg1i(k_InsertKeys, 3, prime);
            clSetKernelArg1p(k_InsertKeys, 4, hashParams);
            clSetKernelArg1p(k_InsertKeys, 5, d_inserted);
            clSetKernelArg1p(k_InsertKeys, 6, d_stashUsed);

            final int dimensions = 1;
            PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions);
            globalWorkSize.put(0, count);
            int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), k_InsertKeys, dimensions, null, globalWorkSize, null,
                    null, null);
            OCLUtils.checkCLError(errcode);

            insertedCount = scanOpenCLService.exclusiveScan(d_inserted, d_insertedScan, count);
            if (insertedCount < 0) {
                return insertedCount;
            }
            stashUsedCount = scanOpenCLService.exclusiveScan(d_stashUsed, d_stashUsedScan, count);
            if (stashUsedCount < 0) {
                // i.e. an error
                return stashUsedCount;
            }
        }
        while (insertedCount < count);

        stashUsed |= stashUsedCount != 0 ? 1 : 0;
        insertedKeys += insertedCount;

        return CL10.CL_SUCCESS;
    }

    private void fillBufferLong(long kernelProgram, long clQueue, long buffer, int count, long value, IntBuffer errcode_ret) {
        long fillBufferKernel = clCreateKernel(kernelProgram, "FillBufferLong", errcode_ret);
        OCLUtils.checkCLError(errcode_ret);

        clSetKernelArg1p(fillBufferKernel, 0, buffer);
        clSetKernelArg1l(fillBufferKernel, 1, value);

        final int dimensions = 1;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions);
        globalWorkSize.put(0, count);

        int errcode = clEnqueueNDRangeKernel(clQueue, fillBufferKernel,
                dimensions, null, globalWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);
        CL10.clReleaseKernel(fillBufferKernel);
    }

    private static boolean IsPrime(int x) {
        int o = 4;
        int i = 5;
        while (true) {
            int q = x / i;
            if (q < i) {
                return true;
            }
            if (x == (q * i)) {
                return false;
            }
            o ^= 6;
            i += o;
        }
    }

    // see http://stackoverflow.com/questions/4475996/given-prime-number-n-compute-the-next-prime
    private int findNextPrime(int n) {
        if (n <= 2) {
            return 2;
        } else if (n == 3) {
            return 3;
        } else if (n <= 5) {
            return 5;
        }
        int k = n / 6;
        int i = n - (6 * k);
        int o = i < 2 ? 1 : 5;
        int x = (6 * k) + o;
        for (i = (3 + o) / 2; !IsPrime(x); x += i) {
            i ^= 6;
        }
        return x;
    }

    public static void main(String[] args) {
        ComputeContext ctx = OCLUtils.getOpenCLContext();
        MeshGenerationContext meshGen = new MeshGenerationContext(64);
        KernelsHolder kernelHolder = new KernelsHolder(ctx);
        kernelHolder.buildKernel(KernelNames.SCAN, null);
        kernelHolder.buildKernel(KernelNames.CUCKOO, VoxelHelperUtils.getCuckooBuildOptions(meshGen));
        kernelHolder.buildKernel(KernelNames.TEST_CUCKOO, VoxelHelperUtils.getCuckooBuildOptions(meshGen));

        ScanOpenCLService scanService = new ScanOpenCLService(ctx, kernelHolder.getKernel(KernelNames.SCAN));

        int KEY_COUNT = 100;
        CuckooHashOpenCLService cuckooHashService = new CuckooHashOpenCLService(ctx, meshGen,
                scanService, kernelHolder, KEY_COUNT);
        int[] keys = new int[KEY_COUNT];
        for (int i = 0; i < 100; i++) {
            keys[i] = i;
        }
        long buffer = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_ONLY | CL10.CL_MEM_COPY_HOST_PTR, OCLUtils.getIntBuffer(keys), ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        int result = cuckooHashService.insertKeys(buffer, KEY_COUNT);
        OCLUtils.validateExpression(result == CL10.CL_SUCCESS, true, "CuckooHash error");

        int resultBufferSize = 10;
        int inputArgument = 53;
        long resultOutBuffer = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, resultBufferSize * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        long testCuckooKernel = clCreateKernel(kernelHolder.getKernel(KernelNames.TEST_CUCKOO), "testCuckoo", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        clSetKernelArg1i(testCuckooKernel, 0, inputArgument);
        clSetKernelArg1p(testCuckooKernel, 1, resultOutBuffer);
        clSetKernelArg1p(testCuckooKernel, 2, cuckooHashService.getTable());
        clSetKernelArg1p(testCuckooKernel, 3, cuckooHashService.getStash());
        clSetKernelArg1i(testCuckooKernel, 4, cuckooHashService.getPrime());
        clSetKernelArg1p(testCuckooKernel, 5, cuckooHashService.getHashParams());
        clSetKernelArg1i(testCuckooKernel, 6, cuckooHashService.getStashUsed());

        final int dimensions = 1;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions);
        globalWorkSize.put(0, resultBufferSize);
        int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), testCuckooKernel, dimensions, null, globalWorkSize, null,
                null, null);
        OCLUtils.checkCLError(errcode);

        int[] outputResult = OCLUtils.getIntBuffer(resultOutBuffer, resultBufferSize);
        OCLUtils.validateExpression(outputResult[0] == inputArgument, true, "Cuckoo hash error");

        CL10.clReleaseMemObject(resultOutBuffer);
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
