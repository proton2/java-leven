package dc.impl.notused.gpu.test;

import dc.impl.notused.gpu.opencl.*;
import org.lwjgl.opencl.CL;

public final class NVidiaScanOpenCLTest {

    private StringBuilder createScanOpenCLTestBuildOptions(){
        StringBuilder buildOptions = new StringBuilder();
        buildOptions.append("-D WORKGROUP_SIZE=256");
        return buildOptions;
    }

    public void run() {
        ComputeContext ctx = OCLUtils.getOpenCLContext();
        KernelsHolder meshGen = new KernelsHolder(ctx);
        BufferGpuService bufferGpuService = new BufferGpuService(ctx);
        meshGen.buildKernel(KernelNames.NVIDIA_SCAN, createScanOpenCLTestBuildOptions());

        NVidiaScanOpenCLService scanOpenCLService = new NVidiaScanOpenCLService(ctx, meshGen, bufferGpuService);
        scanOpenCLService.scan();

        meshGen.destroyContext();
        CL.destroy();
    }

    public static void main(String... args) {
        NVidiaScanOpenCLTest clApp = new NVidiaScanOpenCLTest();
        clApp.run();
    }
}