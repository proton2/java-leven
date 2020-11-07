package dc.impl.opencl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.utils.BufferUtil;
import dc.PointerBasedOctreeNode;
import dc.entities.MeshBuffer;
import dc.entities.MeshVertex;
import dc.impl.GpuOctree;
import dc.impl.MeshGenerationContext;
import dc.utils.VoxelHelperUtils;
import org.lwjgl.BufferUtils;
import org.lwjgl.PointerBuffer;
import org.lwjgl.opencl.CL10;

import java.util.List;

import static org.lwjgl.opencl.CL10.*;

public class GenerateMeshFromOctreeService {
    private final MeshGenerationContext meshGen;
    private final ComputeContext ctx;
    private final GpuOctree octree;
    private final ScanOpenCLService scanService;
    private final MeshBufferGPU meshBufferGPU;
    private long d_indexBuffer, d_trianglesValid, d_trianglesScan;
    private int numVertices, indexBufferSize, trianglesValidSize;
    private long d_isSeamNode, d_isSeamNodeScan;

    public GenerateMeshFromOctreeService(ComputeContext ctx, MeshGenerationContext meshGen, ScanOpenCLService scanService,
                                         GpuOctree octree, MeshBufferGPU meshBufferGPU) {
        this.meshGen = meshGen;
        this.ctx = ctx;
        this.octree = octree;
        this.scanService = scanService;
        this.meshBufferGPU = meshBufferGPU;

        numVertices = octree.getNumNodes();
        indexBufferSize = numVertices * 6 * 3;
        trianglesValidSize = numVertices * 3;
    }

    public int generateMeshKernel(KernelsHolder kernels,
                                  int[] meshIndex, int[] trianglesValid) {
        d_indexBuffer = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, indexBufferSize * Integer.BYTES, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        d_trianglesValid = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, trianglesValidSize * Integer.BYTES, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        long k_GenerateMeshKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "GenerateMesh", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(k_GenerateMeshKernel, 0, octree.getNodeCodesBuf());
        clSetKernelArg1p(k_GenerateMeshKernel, 1, octree.getNodeMaterialsBuf());
        clSetKernelArg1p(k_GenerateMeshKernel, 2, d_indexBuffer);
        clSetKernelArg1p(k_GenerateMeshKernel, 3, d_trianglesValid);

        clSetKernelArg1p(k_GenerateMeshKernel, 4, octree.getHashTable().getTable());
        clSetKernelArg1p(k_GenerateMeshKernel, 5, octree.getHashTable().getStash());
        clSetKernelArg1i(k_GenerateMeshKernel, 6, octree.getHashTable().getPrime());
        clSetKernelArg1p(k_GenerateMeshKernel, 7, octree.getHashTable().getHashParams());
        clSetKernelArg1i(k_GenerateMeshKernel, 8, octree.getHashTable().getStashUsed());

        PointerBuffer createLeafNodesWorkSize = BufferUtils.createPointerBuffer(1);
        createLeafNodesWorkSize.put(0, octree.getNumNodes());
        int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), k_GenerateMeshKernel, 1, null, createLeafNodesWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);

        OCLUtils.getIntBuffer(d_indexBuffer, meshIndex);
        OCLUtils.getIntBuffer(d_trianglesValid, trianglesValid);

        d_trianglesScan = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, trianglesValidSize * Integer.BYTES, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        int numTriangles = scanService.exclusiveScan(d_trianglesValid, d_trianglesScan, trianglesValidSize);
        if (numTriangles <= 0) {
            return numTriangles; // < 0 is an error, so return that, 0 is ok just no tris to generate, return 0 which is CL_SUCCESS
        }
        numTriangles *= 2;
        OCLUtils.validateExpression(numTriangles < meshGen.MAX_MESH_TRIANGLES, true, "Mesh triangle count too high");
        OCLUtils.validateExpression( numVertices < meshGen.MAX_MESH_VERTICES, true, "Mesh vertex count too high");

        int err = CL10.clReleaseKernel(k_GenerateMeshKernel);
        OCLUtils.checkCLError(err);

        return numTriangles;
    }

    public void compactMeshTrianglesKernel(KernelsHolder kernels, int numTriangles, int[] compactIndexBuffer){
        long d_compactIndexBuffer = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, Integer.BYTES * numTriangles * 3, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        long k_CompactMeshTrianglesKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "CompactMeshTriangles", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(k_CompactMeshTrianglesKernel, 0, d_trianglesValid);
        clSetKernelArg1p(k_CompactMeshTrianglesKernel, 1, d_trianglesScan);
        clSetKernelArg1p(k_CompactMeshTrianglesKernel, 2, d_indexBuffer);
        clSetKernelArg1p(k_CompactMeshTrianglesKernel, 3, d_compactIndexBuffer);

        PointerBuffer trianglesValidSizeWorkSize = BufferUtils.createPointerBuffer(1);
        trianglesValidSizeWorkSize.put(0, trianglesValidSize);
        int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), k_CompactMeshTrianglesKernel, 1, null, trianglesValidSizeWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);

        meshBufferGPU.setTriangles(d_compactIndexBuffer);
        meshBufferGPU.setCountTriangles(numTriangles);

        OCLUtils.getIntBuffer(d_compactIndexBuffer, compactIndexBuffer);

        int err = CL10.clReleaseKernel(k_CompactMeshTrianglesKernel);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(d_compactIndexBuffer);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(d_indexBuffer);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(d_trianglesValid);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(d_trianglesScan);
        OCLUtils.checkCLError(err);
    }

    public int run(KernelsHolder kernels, int clipmapNodeSize) {
        long d_vertexBuffer = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, Float.BYTES * 4 * 3 * numVertices, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        Vec3f colour = VoxelHelperUtils.ColourForMinLeafSize(clipmapNodeSize/meshGen.clipmapLeafSize);

        long k_GenerateMeshVertexBufferKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "GenerateMeshVertexBuffer", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(k_GenerateMeshVertexBufferKernel, 0, octree.getVertexPositionsBuf());
        clSetKernelArg1p(k_GenerateMeshVertexBufferKernel, 1, octree.getVertexNormalsBuf());
        clSetKernelArg1p(k_GenerateMeshVertexBufferKernel, 2, octree.getNodeMaterialsBuf());
        clSetKernelArg4f(k_GenerateMeshVertexBufferKernel, 3, colour.X, colour.Y, colour.Z, 0.f);
        clSetKernelArg1p(k_GenerateMeshVertexBufferKernel, 4, d_vertexBuffer);

        PointerBuffer numVerticesWorkSize = BufferUtils.createPointerBuffer(1);
        numVerticesWorkSize.put(0, numVertices);
        int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), k_GenerateMeshVertexBufferKernel, 1, null, numVerticesWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);

        meshBufferGPU.setVertices(d_vertexBuffer);
        meshBufferGPU.setCountVertices(numVertices);

        int err = CL10.clReleaseKernel(k_GenerateMeshVertexBufferKernel);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(d_vertexBuffer);
        OCLUtils.checkCLError(err);

        return CL_SUCCESS;
    }

    public int exportMeshBuffer(MeshBufferGPU gpuBuffer, MeshBuffer cpuBuffer){
        //cpuBuffer.setNumVertices(gpuBuffer.getCountVertices());
        //cpuBuffer.setNumIndicates(gpuBuffer.getCountTriangles());

        MeshVertex[] meshVertices = OCLUtils.getVertexBuffer(gpuBuffer.getVertices(), gpuBuffer.getCountVertices());
        cpuBuffer.setVertices(BufferUtil.createDcFlippedBufferAOS(meshVertices));

        //Vec3i[] triangles = OCLUtils.getTriangles(gpuBuffer.getTriangles(), gpuBuffer.getCountTriangles());
        //cpuBuffer.setIndicates(BufferUtil.createDcFlippedBufferAOS(triangles));
        //cpuBuffer.setIndicates(OCLUtils.getTrianglesAsIntBuffer(meshBufferGPU.getTriangles(), meshBufferGPU.getCountTriangles()));
        return CL_SUCCESS;
    }

    public int findSeamNodesKernel(KernelsHolder kernels) {
        d_isSeamNode = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, Integer.BYTES * octree.getNumNodes(), ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        d_isSeamNodeScan = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, Integer.BYTES * octree.getNumNodes(), ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        long k_FindSeamNodesKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "FindSeamNodes", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(k_FindSeamNodesKernel, 0, octree.getNodeCodesBuf());
        clSetKernelArg1p(k_FindSeamNodesKernel, 1, d_isSeamNode);

        PointerBuffer numVerticesWorkSize = BufferUtils.createPointerBuffer(1);
        numVerticesWorkSize.put(0, octree.getNumNodes());
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), k_FindSeamNodesKernel, 1, null, numVerticesWorkSize, null, null, null);
        OCLUtils.checkCLError(err);

        int numSeamNodes = scanService.exclusiveScan(d_isSeamNode, d_isSeamNodeScan, octree.getNumNodes());
        if (numSeamNodes <= 0) {
            return numSeamNodes;
        }
        //OCLUtils.getIntBuffer(d_isSeamNode, isSeamNode);
        err = CL10.clReleaseKernel(k_FindSeamNodesKernel);
        OCLUtils.checkCLError(err);
        return numSeamNodes;
    }

    public int gatherSeamNodesFromOctree(KernelsHolder kernels, Vec3i chunkMin, int chunkSize, List<PointerBasedOctreeNode> seamNodes, int numSeamNodes) {
        long d_seamNodeInfo = CL10.clCreateBuffer(ctx.getClContext(), CL10.CL_MEM_READ_WRITE, Float.BYTES * 4 * 3 * numSeamNodes, ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());

        long k_ExtractSeamNodeInfoKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "ExtractSeamNodeInfo", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 0, d_isSeamNode);
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 1, d_isSeamNodeScan);
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 2, octree.getNodeCodesBuf());
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 3, octree.getNodeMaterialsBuf());
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 4, octree.getVertexPositionsBuf());
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 5, octree.getVertexNormalsBuf());
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 6, d_seamNodeInfo);

        PointerBuffer extractSeamNodeWorkSize = BufferUtils.createPointerBuffer(1);
        extractSeamNodeWorkSize.put(0, octree.getNumNodes());
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), k_ExtractSeamNodeInfoKernel, 1, null, extractSeamNodeWorkSize, null, null, null);
        OCLUtils.checkCLError(err);

        OCLUtils.getListSeamNodesTriangles(d_seamNodeInfo, numSeamNodes, chunkMin,
                VoxelHelperUtils.ColourForMinLeafSize(chunkSize), //Constants.Yellow
                chunkSize, seamNodes);

        err = CL10.clReleaseMemObject(d_seamNodeInfo);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseKernel(k_ExtractSeamNodeInfoKernel);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(d_isSeamNode);
        OCLUtils.checkCLError(err);
        err = CL10.clReleaseMemObject(d_isSeamNodeScan);
        OCLUtils.checkCLError(err);
        return CL_SUCCESS;
    }
}
