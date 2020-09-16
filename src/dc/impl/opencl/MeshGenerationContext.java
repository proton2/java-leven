package dc.impl.opencl;

import core.utils.ResourceLoader;
import org.lwjgl.opencl.CL;
import org.lwjgl.opencl.CL10;

import static org.lwjgl.system.MemoryUtil.NULL;

public class MeshGenerationContext {
    private long densityProgram;
    private final ComputeContext computeContext;

    public long getDensityProgram() {
        return densityProgram;
    }

    public MeshGenerationContext(ComputeContext computeContext) {
        this.computeContext = computeContext;
    }

    public void createContext() {
        String noiseProgramSource = ResourceLoader.loadShader("opencl/Noise.cl");
        String dencityProgramSource = ResourceLoader.loadShader("opencl/dencity_field.cl");
        String uniteProgramSource = noiseProgramSource + " \n" + dencityProgramSource;

        densityProgram = CL10.clCreateProgramWithSource(computeContext.getClContext(), uniteProgramSource, computeContext.getErrcode_ret());
        int errcode = CL10.clBuildProgram(densityProgram, computeContext.getClDevice(), "", null, NULL);
        OCLUtils.checkCLError(errcode);
    }

    public void destroyContext(){
        // Destroy our kernel and program
        CL10.clReleaseCommandQueue(computeContext.getClQueue());
        CL10.clReleaseProgram(densityProgram);
        // Not strictly necessary
        CL.destroy();
    }
}
