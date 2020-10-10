package test;

import dc.impl.opencl.ComputeContext;
import dc.impl.opencl.MeshGenerationContext;
import dc.impl.opencl.OCLUtils;
import dc.impl.opencl.ScanOpenCLService;
import org.lwjgl.BufferUtils;
import org.lwjgl.opencl.CL;
import org.lwjgl.opencl.CL10;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;

import static dc.ChunkOctree.VOXELS_PER_CHUNK;
import static dc.ChunkOctree.log2;
import static org.lwjgl.opencl.CL10.CL_MEM_READ_WRITE;

public final class ScanOpenCLTest {

    public void run() {
        ComputeContext ctx = OCLUtils.getOpenCLContext();

        MeshGenerationContext meshGen = new MeshGenerationContext(ctx);
        meshGen.setVoxelsPerChunk(VOXELS_PER_CHUNK);
        meshGen.setHermiteIndexSize(meshGen.getVoxelsPerChunk() + 1);
        meshGen.setFieldSize(meshGen.getHermiteIndexSize() + 1);
        meshGen.setIndexShift(log2(VOXELS_PER_CHUNK) + 1);
        meshGen.setIndexMask((1 << meshGen.getIndexShift()) - 1);
        meshGen.createContext();

        int[] inputArray = {0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1};
        IntBuffer intBuffer = getIntBuffer(inputArray);
        long inputData = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE | CL10.CL_MEM_COPY_HOST_PTR, intBuffer, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        long scanData = CL10.clCreateBuffer(ctx.getClContext(), CL_MEM_READ_WRITE, inputArray.length * 4, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        ScanOpenCLService scanOpenCLService = new ScanOpenCLService(ctx);
        int numEdges = scanOpenCLService.exclusiveScan(meshGen.getScanProgram(), inputData, scanData, inputArray.length);
        System.out.println(numEdges);
        int[] outputArray = scanOpenCLService.getIntBuffer(scanData, inputArray.length);
        System.out.println(outputArray);

        meshGen.destroyContext();
        CL.destroy();
    }

    private IntBuffer getIntBuffer(int[] inputArray) {
        IntBuffer aBuff = BufferUtils.createIntBuffer(inputArray.length);
        aBuff.put(inputArray);
        aBuff.rewind();
        return aBuff;
    }

    public static void main(String... args) {
        ScanOpenCLTest clApp = new ScanOpenCLTest();
        clApp.run();
    }
}