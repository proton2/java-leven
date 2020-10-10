package dc.impl.opencl;

import core.utils.ResourceLoader;
import org.lwjgl.opencl.CL;
import org.lwjgl.opencl.CL10;

import java.io.File;
import java.nio.file.Paths;

import static org.lwjgl.system.MemoryUtil.NULL;

public class MeshGenerationContext {
    private long densityProgram;
    private long findFieldEdgesProgram;
    private long scanProgram;
    private int voxelsPerChunk;
    private int indexShift;
    private int indexMask;
    private int hermiteIndexSize;
    private int fieldSize;

    public int getFieldSize() {
        return fieldSize;
    }

    public void setFieldSize(int fieldSize) {
        this.fieldSize = fieldSize;
    }

    public int getHermiteIndexSize() {
        return hermiteIndexSize;
    }

    public void setHermiteIndexSize(int hermiteIndexSize) {
        this.hermiteIndexSize = hermiteIndexSize;
    }

    public int getIndexShift() {
        return indexShift;
    }

    public void setIndexShift(int indexShift) {
        this.indexShift = indexShift;
    }

    public int getIndexMask() {
        return indexMask;
    }

    public void setIndexMask(int indexMask) {
        this.indexMask = indexMask;
    }

    public int getVoxelsPerChunk() {
        return voxelsPerChunk;
    }

    public void setVoxelsPerChunk(int voxelsPerChunk) {
        this.voxelsPerChunk = voxelsPerChunk;
    }

    public long getScanProgram() {
        return scanProgram;
    }

    public long getFindFieldEdgesProgram() {
        return findFieldEdgesProgram;
    }

    private final ComputeContext computeContext;

    public long getDensityProgram() {
        return densityProgram;
    }

    public MeshGenerationContext(ComputeContext computeContext) {
        this.computeContext = computeContext;
    }

    public void createContext() {
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

        File file = new File(Paths.get("res/opencl/scan.cl").toUri());
        if(file.exists()){
            buildOptions.append("-I ").append(file.getParent());
        }
        
        String noiseProgramSource = ResourceLoader.loadShader("opencl/density_simplex.cl");
        densityProgram = CL10.clCreateProgramWithSource(computeContext.getClContext(), noiseProgramSource, computeContext.getErrcode_ret());
        int errcode = CL10.clBuildProgram(densityProgram, computeContext.getClDevice(), buildOptions, null, NULL);
        OCLUtils.checkCLError(errcode);

        String findFieldEdgesProgramSource = ResourceLoader.loadShader("opencl/findDefaultEdges.cl");
        findFieldEdgesProgram = CL10.clCreateProgramWithSource(computeContext.getClContext(),  findFieldEdgesProgramSource, computeContext.getErrcode_ret());
        errcode = CL10.clBuildProgram(findFieldEdgesProgram, computeContext.getClDevice(), buildOptions, null, NULL);
        OCLUtils.checkCLError(errcode);

        String scanProgramSource = ResourceLoader.loadShader("opencl/scan.cl");
        scanProgram = CL10.clCreateProgramWithSource(computeContext.getClContext(), scanProgramSource, computeContext.getErrcode_ret());
        errcode = CL10.clBuildProgram(scanProgram, computeContext.getClDevice(), "", null, NULL);
        OCLUtils.checkCLError(errcode);
    }

    public void destroyContext(){
        // Destroy our kernel and program
        int ret;
        ret = CL10.clReleaseCommandQueue(computeContext.getClQueue());
        OCLUtils.checkCLError(ret);
        ret = CL10.clReleaseProgram(densityProgram);
        OCLUtils.checkCLError(ret);
        // Not strictly necessary
        CL.destroy();
    }
}
