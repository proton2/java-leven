package dc.impl.notused.gpu.test;

import dc.impl.MeshGenerationContext;
import dc.impl.notused.gpu.opencl.ComputeContext;
import dc.impl.notused.gpu.opencl.KernelNames;
import dc.impl.notused.gpu.opencl.KernelsHolder;
import dc.impl.notused.gpu.opencl.OCLUtils;
import dc.utils.VoxelHelperUtils;
import org.lwjgl.opencl.CL;

import java.util.ArrayList;

public final class BuildScriptsOpenCLTest {

    public void run() {
        ComputeContext ctx = OCLUtils.getOpenCLContext();
        KernelsHolder meshGen = new KernelsHolder(ctx);

        MeshGenerationContext meshGenCtx = new MeshGenerationContext(64);
        StringBuilder kernelBuildOptions = VoxelHelperUtils.createMainBuildOptions(meshGenCtx);

        ArrayList<String> headers = new ArrayList<>();
        headers.add("opencl/test/shared_constants.cl");
        headers.add("opencl/test/simplex.cl");
        headers.add("opencl/test/noise.cl");
        meshGen.buildKernel(KernelNames.DENSITY_L, kernelBuildOptions, headers);

        meshGen.destroyContext();
        CL.destroy();
    }

    public static void main(String... args) {
        BuildScriptsOpenCLTest clApp = new BuildScriptsOpenCLTest();
        clApp.run();
    }
}