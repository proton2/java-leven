package dc.impl.notused.gpu.test;

import dc.impl.notused.gpu.opencl.*;
import dc.utils.VoxelHelperUtils;
import org.lwjgl.opencl.CL;
import org.lwjgl.opencl.CL10;

import java.nio.IntBuffer;

public final class ScanOpenCLTest {

    StringBuilder createScanOpenCLTestBuildOptions(){
        int voxelsPerChunk = 64;
        int indexShift = VoxelHelperUtils.log2(voxelsPerChunk) + 1;
        int hermiteIndexSize = voxelsPerChunk + 1;
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
        buildOptions.append("-DVOXELS_PER_CHUNK=").append(voxelsPerChunk).append(" ");
        buildOptions.append("-DVOXEL_INDEX_SHIFT=").append(indexShift).append(" ");
        buildOptions.append("-DVOXEL_INDEX_MASK=").append(indexMask).append(" ");
        buildOptions.append("-DHERMITE_INDEX_SIZE=").append(hermiteIndexSize).append(" ");
        return buildOptions;
    }

    public void run() {
        ComputeContext ctx = OCLUtils.getOpenCLContext();
        BufferGpuService bufferGpuService = new BufferGpuService(ctx);
        KernelsHolder meshGen = new KernelsHolder(ctx);
        meshGen.buildKernel(KernelNames.SCAN, createScanOpenCLTestBuildOptions());

        int[] inputArray = {0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1};
        IntBuffer intBuffer = OCLUtils.getIntBuffer(inputArray);
        BufferGpu inputData = bufferGpuService.create("inputData", intBuffer, CL10.CL_MEM_READ_WRITE | CL10.CL_MEM_COPY_HOST_PTR);
        BufferGpu scanData = bufferGpuService.create("scanData", inputArray.length * 4, MemAccess.READ_WRITE);

        ScanOpenCLService scanOpenCLService = new ScanOpenCLService(ctx, meshGen.getKernel(KernelNames.SCAN), bufferGpuService);
        int numEdges = scanOpenCLService.exclusiveScan(inputData, scanData, inputArray.length);
        System.out.println(numEdges);
        int[] outputArray = OCLUtils.getIntBuffer(scanData, inputArray.length);
        System.out.println(outputArray);

        bufferGpuService.release(inputData);
        bufferGpuService.release(scanData);

        meshGen.destroyContext();
        CL.destroy();
    }

    public static void main(String... args) {
        ScanOpenCLTest clApp = new ScanOpenCLTest();
        clApp.run();
    }
}