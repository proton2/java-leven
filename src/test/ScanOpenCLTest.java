package test;

import dc.impl.opencl.*;
import org.lwjgl.BufferUtils;
import org.lwjgl.opencl.CL;
import org.lwjgl.opencl.CL10;

import java.nio.IntBuffer;

import static dc.ChunkOctree.VOXELS_PER_CHUNK;
import static dc.ChunkOctree.log2;
import static org.lwjgl.opencl.CL10.CL_MEM_READ_WRITE;

public final class ScanOpenCLTest {

    StringBuilder createScanOpenCLTestBuildOptions(){
        int indexShift = log2(VOXELS_PER_CHUNK) + 1;
        int hermiteIndexSize = VOXELS_PER_CHUNK + 1;
        int fieldSize = hermiteIndexSize + 1;
        int indexMask = (1 << indexShift) - 1;

        StringBuilder buildOptions = new StringBuilder();
        buildOptions.append("-cl-denorms-are-zero ");
        buildOptions.append("-cl-finite-math-only ");
        buildOptions.append("-cl-no-signed-zeros ");
        buildOptions.append("-cl-fast-relaxed-math ");
        buildOptions.append("-Werror ");
        buildOptions.append("-DFIELD_DIM=").append(fieldSize).append(" ");
        buildOptions.append("-DFIND_EDGE_INFO_STEPS=" + 16 + " ");
        buildOptions.append("-DFIND_EDGE_INFO_INCREMENT=" + (1.f/16.f) + " ");
        buildOptions.append("-DVOXELS_PER_CHUNK=").append(VOXELS_PER_CHUNK).append(" ");
        buildOptions.append("-DVOXEL_INDEX_SHIFT=").append(indexShift).append(" ");
        buildOptions.append("-DVOXEL_INDEX_MASK=").append(indexMask).append(" ");
        buildOptions.append("-DHERMITE_INDEX_SIZE=").append(hermiteIndexSize).append(" ");
        return buildOptions;
    }

    public void run() {
        ComputeContext ctx = OCLUtils.getOpenCLContext();
        KernelsHolder meshGen = new KernelsHolder(ctx);
        meshGen.buildKernel(KernelNames.SCAN, createScanOpenCLTestBuildOptions());

        int[] inputArray = {0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1};
        IntBuffer intBuffer = OCLUtils.getIntBuffer(inputArray);
        long inputData = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE | CL10.CL_MEM_COPY_HOST_PTR, intBuffer, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        long scanData = CL10.clCreateBuffer(ctx.getClContext(), CL_MEM_READ_WRITE, inputArray.length * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        ScanOpenCLService scanOpenCLService = new ScanOpenCLService(ctx, meshGen.getKernel(KernelNames.SCAN));
        int numEdges = scanOpenCLService.exclusiveScan(inputData, scanData, inputArray.length);
        System.out.println(numEdges);
        int[] outputArray = OCLUtils.getIntBuffer(scanData, inputArray.length);
        System.out.println(outputArray);

        meshGen.destroyContext();
        CL.destroy();
    }

    public static void main(String... args) {
        ScanOpenCLTest clApp = new ScanOpenCLTest();
        clApp.run();
    }
}