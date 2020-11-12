package dc.impl.opencl;

import dc.impl.MeshGenerationContext;
import dc.utils.VoxelHelperUtils;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import java.nio.IntBuffer;

import static org.lwjgl.opencl.CL10.*;

public class CuckooHashOpenCLService {
    private BufferGpu table, stash, hashParams;
    private int prime;
    private int stashUsed = 0;                    // int rather than bool as bools seem somewhat iffy in OpenCL
    private int insertedKeys = 0;

    public BufferGpu getTable() {
        return table;
    }

    public BufferGpu getStash() {
        return stash;
    }

    public BufferGpu getHashParams() {
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
    private final BufferGpuService bufferGpuService;

    public CuckooHashOpenCLService(ComputeContext ctx, MeshGenerationContext meshGen, ScanOpenCLService scanService,
                                   KernelsHolder kernelHolder, int tableSize, BufferGpuService bufferGpuService) {
        this.meshGen = meshGen;
        this.bufferGpuService = bufferGpuService;
        this.scanOpenCLService = scanService;
        this.ctx = ctx;
        this.kernelProgram = kernelHolder;
        prime = findNextPrime(Math.max(MIN_TABLE_SIZE, tableSize * 2));

        table = bufferGpuService.create("cockooTable", prime * 8, CL10.CL_MEM_READ_WRITE);
        fillBufferLong(kernelProgram.getKernel(KernelNames.CUCKOO), ctx.getClQueue(), table, prime, meshGen.CUCKOO_EMPTY_VALUE, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        stash = bufferGpuService.create("cuckooStash", meshGen.CUCKOO_STASH_SIZE * 8, CL10.CL_MEM_READ_WRITE);
        fillBufferLong(kernelProgram.getKernel(KernelNames.CUCKOO), ctx.getClQueue(), stash, meshGen.CUCKOO_STASH_SIZE, meshGen.CUCKOO_EMPTY_VALUE, ctx.getErrcode_ret());

        int[] params = initHashValues();
        hashParams = bufferGpuService.create("cuckooHashParams", OCLUtils.getIntBuffer(params), CL10.CL_MEM_READ_ONLY | CL10.CL_MEM_COPY_HOST_PTR);
    }

    public void destroy(){
        bufferGpuService.release(table);
        bufferGpuService.release(stash);
        bufferGpuService.release(hashParams);
    }

    private int[] initHashValues(){
        int[] params = new int[meshGen.CUCKOO_HASH_FN_COUNT * 2];
        for (int i = 0; i < meshGen.CUCKOO_HASH_FN_COUNT; i++) {
            params[i * 2 + 0] = Rnd.get(1 << 15, 1 << 30);
            params[i * 2 + 1] = Rnd.get(1 << 15, 1 << 30);
        }
        return params;
    }

    public int insertKeys(BufferGpu d_keys, int count) {
        BufferGpu d_inserted = bufferGpuService.create("d_inserted", count * 4, CL10.CL_MEM_READ_WRITE);
        BufferGpu d_stashUsed = bufferGpuService.create("d_stashUsed", count * 4, CL10.CL_MEM_READ_WRITE);
        BufferGpu d_insertedScan = bufferGpuService.create("d_insertedScan", count * 4, CL10.CL_MEM_READ_WRITE);
        BufferGpu d_stashUsedScan = bufferGpuService.create("d_stashUsedScan", count * 4, CL10.CL_MEM_READ_WRITE);

        long k_InsertKeys = clCreateKernel(kernelProgram.getKernel(KernelNames.CUCKOO), "Cuckoo_InsertKeys", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        int numRetries = 0;
        int insertedCount = 0, stashUsedCount = 0;
        do {
            if (insertedCount > 0) {
                System.out.printf("Cuckoo: insert keys failed (failed to insert %d key from %d). Retry #%d...\n%n", count - insertedCount, count, ++numRetries);
                int[] params = initHashValues();
                int errcode = CL10.clEnqueueWriteBuffer(ctx.getClQueue(), hashParams.getMem(), false, 0, OCLUtils.getIntBuffer(params), null, null);
                OCLUtils.checkCLError(errcode);
                fillBufferLong(kernelProgram.getKernel(KernelNames.CUCKOO), ctx.getClQueue(), table, prime, meshGen.CUCKOO_EMPTY_VALUE, ctx.getErrcode_ret());
                fillBufferLong(kernelProgram.getKernel(KernelNames.CUCKOO), ctx.getClQueue(), stash, meshGen.CUCKOO_STASH_SIZE, meshGen.CUCKOO_EMPTY_VALUE, ctx.getErrcode_ret());
            }

            clSetKernelArg1p(k_InsertKeys, 0, d_keys.getMem());
            clSetKernelArg1p(k_InsertKeys, 1, table.getMem());
            clSetKernelArg1p(k_InsertKeys, 2, stash.getMem());
            clSetKernelArg1i(k_InsertKeys, 3, prime);
            clSetKernelArg1p(k_InsertKeys, 4, hashParams.getMem());
            clSetKernelArg1p(k_InsertKeys, 5, d_inserted.getMem());
            clSetKernelArg1p(k_InsertKeys, 6, d_stashUsed.getMem());

            final int dimensions = 1;
            PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions);
            globalWorkSize.put(0, count);
            int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), k_InsertKeys, dimensions, null, globalWorkSize, null,
                    null, null);
            OCLUtils.checkCLError(errcode);

            insertedCount = scanOpenCLService.exclusiveScan(d_inserted, d_insertedScan, count);
            if (insertedCount < 0) {
                bufferGpuService.release(d_inserted);
                bufferGpuService.release(d_stashUsed);
                bufferGpuService.release(d_insertedScan);
                bufferGpuService.release(d_stashUsedScan);
                int err = CL10.clReleaseKernel(k_InsertKeys);
                OCLUtils.checkCLError(err);
                return insertedCount;
            }
            stashUsedCount = scanOpenCLService.exclusiveScan(d_stashUsed, d_stashUsedScan, count);
            if (stashUsedCount < 0) {
                bufferGpuService.release(d_inserted);
                bufferGpuService.release(d_stashUsed);
                bufferGpuService.release(d_insertedScan);
                bufferGpuService.release(d_stashUsedScan);
                int err = CL10.clReleaseKernel(k_InsertKeys);
                OCLUtils.checkCLError(err);
                // i.e. an error
                return stashUsedCount;
            }
        }
        while (insertedCount < count);

        stashUsed |= stashUsedCount != 0 ? 1 : 0;
        insertedKeys += insertedCount;

        bufferGpuService.release(d_inserted);
        bufferGpuService.release(d_stashUsed);
        bufferGpuService.release(d_insertedScan);
        bufferGpuService.release(d_stashUsedScan);
        int err = CL10.clReleaseKernel(k_InsertKeys);
        OCLUtils.checkCLError(err);

        return CL10.CL_SUCCESS;
    }

    private void fillBufferLong(long kernelProgram, long clQueue, BufferGpu buffer, int count, long value, IntBuffer errcode_ret) {
        long fillBufferKernel = clCreateKernel(kernelProgram, "FillBufferLong", errcode_ret);
        OCLUtils.checkCLError(errcode_ret);

        clSetKernelArg1p(fillBufferKernel, 0, buffer.getMem());
        clSetKernelArg1l(fillBufferKernel, 1, value);

        final int dimensions = 1;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions);
        globalWorkSize.put(0, count);

        int err = clEnqueueNDRangeKernel(clQueue, fillBufferKernel,
                dimensions, null, globalWorkSize, null, null, null);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseKernel(fillBufferKernel);
        OCLUtils.checkCLError(err);
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
}
