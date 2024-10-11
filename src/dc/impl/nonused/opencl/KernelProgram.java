package dc.impl.nonused.opencl;

import core.utils.ResourceLoader;
import org.lwjgl.opencl.CL;
import org.lwjgl.opencl.CL10;

import static org.lwjgl.system.MemoryUtil.NULL;

public class KernelProgram {
    private final long kernelProgram;
    private final ComputeContext computeContext;

    public KernelProgram(KernelNames kernelName, ComputeContext computeContext, StringBuilder buildingOptions) {
        this.computeContext = computeContext;
        String programSource = ResourceLoader.loadShader(kernelName.getName());
        this.kernelProgram = CL10.clCreateProgramWithSource(computeContext.getClContext(), programSource, computeContext.getErrcode_ret());
        int errcode = CL10.clBuildProgram(kernelProgram, computeContext.getClDevice(), buildingOptions==null ? "": buildingOptions, null, NULL);
        OCLUtils.checkCLError(errcode);
    }

    public long getKernel() {
        return kernelProgram;
    }

    public void destroyContext(){
        int ret = CL10.clReleaseCommandQueue(computeContext.getClQueue());
        OCLUtils.checkCLError(ret);
        ret = CL10.clReleaseProgram(kernelProgram);
        OCLUtils.checkCLError(ret);
        CL.destroy(); // Not strictly necessary
    }
}
