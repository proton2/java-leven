package dc.impl.opencl;

import core.utils.ResourceLoader;
import org.lwjgl.opencl.CL;
import org.lwjgl.opencl.CL10;

import static org.lwjgl.system.MemoryUtil.NULL;

public class MeshGenerationContext {
    private long densityProgram;
    //private final long noisePermLookupImage;
    private final ComputeContext computeContext;

    public long getDensityProgram() {
        return densityProgram;
    }
//    public long getNoisePermLookupImage() {
//        return noisePermLookupImage;
//    }

    public MeshGenerationContext(ComputeContext computeContext) {
        this.computeContext = computeContext;
        //NoiseTemp noiseTemp = new NoiseTemp();
        //noisePermLookupImage = noiseTemp.CreateNoisePermutationLookupImage(0x7d3af, computeContext);//0x7d3af
    }

    public void createContext() {
        String noiseProgramSource = ResourceLoader.loadShader("opencl/density_simplex.cl");
        //String dencityProgramSource = ResourceLoader.loadShader("opencl/successfull_simple.cl");
        //String uniteProgramSource = noiseProgramSource + " \n" + dencityProgramSource;

        densityProgram = CL10.clCreateProgramWithSource(computeContext.getClContext(), noiseProgramSource, computeContext.getErrcode_ret());
        int errcode = CL10.clBuildProgram(densityProgram, computeContext.getClDevice(), "", null, NULL);
        OCLUtils.checkCLError(errcode);
    }

    public void destroyContext(){
        // Destroy our kernel and program
        int ret;
        ret = CL10.clReleaseCommandQueue(computeContext.getClQueue());
        OCLUtils.checkCLError(ret);
        ret = CL10.clReleaseProgram(densityProgram);
        OCLUtils.checkCLError(ret);
        //ret = CL10.clReleaseMemObject(noisePermLookupImage);
        //OCLUtils.checkCLError(ret);
        // Not strictly necessary
        CL.destroy();
    }
}
