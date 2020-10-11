package dc.impl.opencl;

import core.utils.ResourceLoader;
import org.lwjgl.opencl.CL;
import org.lwjgl.opencl.CL10;

import java.util.HashMap;
import java.util.Map;

import static org.lwjgl.system.MemoryUtil.NULL;

public class KernelsHolder {
    Map<String, Long> kernels = new HashMap<>();
    private final ComputeContext computeContext;

    public KernelsHolder(ComputeContext computeContext) {
        this.computeContext = computeContext;
    }

    public void buildKernel(KernelNames kernelName, StringBuilder buildingOptions){
        String programSource = ResourceLoader.loadShader(kernelName.getName());
        long kernelProgram = CL10.clCreateProgramWithSource(computeContext.getClContext(), programSource, computeContext.getErrcode_ret());
        int errcode = CL10.clBuildProgram(kernelProgram, computeContext.getClDevice(), buildingOptions==null ? "": buildingOptions, null, NULL);
        OCLUtils.checkCLError(errcode);
        kernels.put(kernelName.name(), kernelProgram);
    }

    public long getKernel(KernelNames kernelName){
        return kernels.get(kernelName.name());
    }

    public void destroyContext(){
        int ret;
        ret = CL10.clReleaseCommandQueue(computeContext.getClQueue());
        OCLUtils.checkCLError(ret);
        for (Map.Entry<String, Long> item : kernels.entrySet()){
            ret = CL10.clReleaseProgram(item.getValue());
            OCLUtils.checkCLError(ret);
        }
        CL.destroy(); // Not strictly necessary
    }
}
