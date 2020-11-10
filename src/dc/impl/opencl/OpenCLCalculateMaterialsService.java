package dc.impl.opencl;

import dc.impl.GPUDensityField;
import dc.impl.MeshGenerationContext;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import java.nio.IntBuffer;

import static org.lwjgl.opencl.CL10.*;

public final class OpenCLCalculateMaterialsService {
    private final int size;
    private final ComputeContext ctx;
    private final MeshGenerationContext meshGen;
    private final GPUDensityField field;
    private final BufferGpuService bufferGpuService;

    public OpenCLCalculateMaterialsService(ComputeContext computeContext, int size, MeshGenerationContext meshGenerationContext,
                                           GPUDensityField field, BufferGpuService bufferGpuService) {
        this.meshGen = meshGenerationContext;
        this.size = size;
        this.ctx = computeContext;
        this.field = field;
        this.bufferGpuService = bufferGpuService;
    }

    public void run(KernelsHolder kernels, int[] materials) {
        // init kernel with constants
        long clKernel = clCreateKernel(kernels.getKernel(KernelNames.DENSITY), "GenerateDefaultField", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        int sampleScale = field.getSize() / (meshGen.getVoxelsPerChunk() * meshGen.leafSizeScale);

        BufferGpu fieldMaterials = bufferGpuService.create("fieldMaterials", size * size * size * 4, MemAccess.READ_WRITE);
        field.setMaterials(fieldMaterials);
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        clSetKernelArg4i(clKernel, 0,
                field.getMin().x/meshGen.leafSizeScale,
                field.getMin().y/meshGen.leafSizeScale,
                field.getMin().z/meshGen.leafSizeScale,
                0);
        clSetKernelArg1i(clKernel, 1, sampleScale);
        clSetKernelArg1i(clKernel, 2, ctx.defaultMaterial);
        clSetKernelArg1p(clKernel, 3, field.getMaterials().getMem());

        final int dimensions = 3;
        PointerBuffer globalWorkSize = BufferUtils.createPointerBuffer(dimensions); // In here we put the total number of work items we want in each dimension.
        globalWorkSize.put(0, size); // Size is a variable we defined a while back showing how many elements are in our arrays.
        globalWorkSize.put(1, size); // Size is a variable we defined a while back showing how many elements are in our arrays.
        globalWorkSize.put(2, size); // Size is a variable we defined a while back showing how many elements are in our arrays.

        // Run the specified number of work units using our OpenCL program kernel
        int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), clKernel, dimensions, null, globalWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);

        CL10.clFinish(ctx.getClQueue());

        getResults(materials, field);
        CL10.clReleaseKernel(clKernel);
    }

    private void getResults(int[] result, GPUDensityField field) {
        if(result!=null) {
            // This reads the result memory buffer
            IntBuffer resultBuff = BufferUtils.createIntBuffer(size * size * size);
            // We read the buffer in blocking mode so that when the method returns we know that the result buffer is full
            //CL10.clEnqueueReadBuffer(clQueue, resultMemory, CL10.CL_TRUE, 0, resultBuff, null, null);
            CL10.clEnqueueReadBuffer(ctx.getClQueue(), field.getMaterials().getMem(), true, 0, resultBuff, null, null);
            // Print the values in the result buffer
            resultBuff.get(result);
        }
    }
}