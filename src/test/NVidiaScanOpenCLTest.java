package test;

import dc.impl.opencl.*;
import org.lwjgl.BufferUtils;
import org.lwjgl.opencl.CL;
import org.lwjgl.opencl.CL10;

import java.nio.IntBuffer;

import static dc.ChunkOctree.VOXELS_PER_CHUNK;
import static dc.ChunkOctree.log2;
import static org.lwjgl.opencl.CL10.CL_MEM_READ_WRITE;

public final class NVidiaScanOpenCLTest {

    private StringBuilder createScanOpenCLTestBuildOptions(){
        StringBuilder buildOptions = new StringBuilder();
        buildOptions.append("-D WORKGROUP_SIZE=256");
        return buildOptions;
    }

    public void run() {
        ComputeContext ctx = OCLUtils.getOpenCLContext();
        KernelsHolder meshGen = new KernelsHolder(ctx);
        meshGen.buildKernel(KernelNames.NVIDIA_SCAN, createScanOpenCLTestBuildOptions());

        NVidiaScanOpenCLService scanOpenCLService = new NVidiaScanOpenCLService(ctx, meshGen);
        scanOpenCLService.scan();

        meshGen.destroyContext();
        CL.destroy();
    }

    public static void main(String... args) {
        NVidiaScanOpenCLTest clApp = new NVidiaScanOpenCLTest();
        clApp.run();
    }
}