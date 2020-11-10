package dc.impl.opencl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.utils.BufferUtil;
import dc.PointerBasedOctreeNode;
import dc.entities.MeshBuffer;
import dc.entities.MeshVertex;
import dc.impl.GPUDensityField;
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
    private BufferGpu d_indexBuffer;
    private BufferGpu d_trianglesValid, d_trianglesScan, d_isSeamNode, d_isSeamNodeScan;
    private int numVertices, indexBufferSize, trianglesValidSize;
    private final GPUDensityField field;
    private final BufferGpuService bufferGpuService;

    public GenerateMeshFromOctreeService(ComputeContext ctx, MeshGenerationContext meshGen, ScanOpenCLService scanService,
                                         GpuOctree octree, MeshBufferGPU meshBufferGPU, GPUDensityField field, BufferGpuService bufferGpuService) {
        this.meshGen = meshGen;
        this.ctx = ctx;
        this.octree = octree;
        this.scanService = scanService;
        this.meshBufferGPU = meshBufferGPU;
        this.field = field;
        this.bufferGpuService = bufferGpuService;

        numVertices = octree.getNumNodes();
        indexBufferSize = numVertices * 6 * 3;
        trianglesValidSize = numVertices * 3;
    }

    public int generateMeshKernel(KernelsHolder kernels, int[] meshIndex, int[] trianglesValid) {
        d_indexBuffer = bufferGpuService.create("d_indexBuffer", indexBufferSize * Integer.BYTES, CL10.CL_MEM_READ_WRITE);
        d_trianglesValid = bufferGpuService.create("d_trianglesValid", trianglesValidSize * Integer.BYTES, CL10.CL_MEM_READ_WRITE);

        CuckooHashOpenCLService octreeHashTable = new CuckooHashOpenCLService(ctx, meshGen, scanService, kernels, octree.getNumNodes(), bufferGpuService);
        octreeHashTable.insertKeys(octree.getNodeCodesBuf(), octree.getNumNodes());

        long k_GenerateMeshKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "GenerateMesh", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(k_GenerateMeshKernel, 0, octree.getNodeCodesBuf().getMem());
        clSetKernelArg1p(k_GenerateMeshKernel, 1, octree.getNodeMaterialsBuf().getMem());
        clSetKernelArg1p(k_GenerateMeshKernel, 2, d_indexBuffer.getMem());
        clSetKernelArg1p(k_GenerateMeshKernel, 3, d_trianglesValid.getMem());

        clSetKernelArg1p(k_GenerateMeshKernel, 4, octreeHashTable.getTable().getMem());
        clSetKernelArg1p(k_GenerateMeshKernel, 5, octreeHashTable.getStash().getMem());
        clSetKernelArg1i(k_GenerateMeshKernel, 6, octreeHashTable.getPrime());
        clSetKernelArg1p(k_GenerateMeshKernel, 7, octreeHashTable.getHashParams().getMem());
        clSetKernelArg1i(k_GenerateMeshKernel, 8, octreeHashTable.getStashUsed());

        PointerBuffer createLeafNodesWorkSize = BufferUtils.createPointerBuffer(1);
        createLeafNodesWorkSize.put(0, octree.getNumNodes());
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), k_GenerateMeshKernel, 1, null, createLeafNodesWorkSize, null, null, null);
        OCLUtils.checkCLError(err);

        OCLUtils.getIntBuffer(d_indexBuffer, meshIndex);
        OCLUtils.getIntBuffer(d_trianglesValid, trianglesValid);

        err = CL10.clReleaseKernel(k_GenerateMeshKernel);
        OCLUtils.checkCLError(err);
        octreeHashTable.destroy();
        d_trianglesScan = bufferGpuService.create("d_trianglesScan", trianglesValidSize * Integer.BYTES, CL10.CL_MEM_READ_WRITE);

        int numTriangles = scanService.exclusiveScan(d_trianglesValid, d_trianglesScan, trianglesValidSize);
        if (numTriangles <= 0) {
            bufferGpuService.releaseAll();
            return numTriangles; // < 0 is an error, so return that, 0 is ok just no tris to generate, return 0 which is CL_SUCCESS
        }
        numTriangles *= 2;
        OCLUtils.validateExpression(numTriangles < meshGen.MAX_MESH_TRIANGLES, true, "Mesh triangle count too high");
        OCLUtils.validateExpression( numVertices < meshGen.MAX_MESH_VERTICES, true, "Mesh vertex count too high");
        return numTriangles;
    }

    public void compactMeshTrianglesKernel(KernelsHolder kernels, int numTriangles, int[] compactIndexBuffer){
        BufferGpu d_compactIndexBuffer = bufferGpuService.create("d_compactIndexBuffer", Integer.BYTES * numTriangles * 3, CL_MEM_READ_WRITE);

        long k_CompactMeshTrianglesKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "CompactMeshTriangles", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(k_CompactMeshTrianglesKernel, 0, d_trianglesValid.getMem());
        clSetKernelArg1p(k_CompactMeshTrianglesKernel, 1, d_trianglesScan.getMem());
        clSetKernelArg1p(k_CompactMeshTrianglesKernel, 2, d_indexBuffer.getMem());
        clSetKernelArg1p(k_CompactMeshTrianglesKernel, 3, d_compactIndexBuffer.getMem());

        PointerBuffer trianglesValidSizeWorkSize = BufferUtils.createPointerBuffer(1);
        trianglesValidSizeWorkSize.put(0, trianglesValidSize);
        int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), k_CompactMeshTrianglesKernel, 1, null, trianglesValidSizeWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);

        //meshBufferGPU.setTriangles(d_compactIndexBuffer);
        meshBufferGPU.setCountTriangles(numTriangles);

        OCLUtils.getIntBuffer(d_compactIndexBuffer, compactIndexBuffer);

        int err = CL10.clReleaseKernel(k_CompactMeshTrianglesKernel);
        OCLUtils.checkCLError(err);

        bufferGpuService.release(d_compactIndexBuffer);
        bufferGpuService.release(d_indexBuffer);
        bufferGpuService.release(d_trianglesValid);
        bufferGpuService.release(d_trianglesScan);
    }

    public int run(KernelsHolder kernels, int clipmapNodeSize) {
        BufferGpu d_vertexBuffer = bufferGpuService.create("d_vertexBuffer", Float.BYTES * 4 * 3 * numVertices, CL10.CL_MEM_READ_WRITE);
        Vec3f colour = VoxelHelperUtils.ColourForMinLeafSize(clipmapNodeSize/meshGen.clipmapLeafSize);

        long k_GenerateMeshVertexBufferKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "GenerateMeshVertexBuffer", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(k_GenerateMeshVertexBufferKernel, 0, octree.getVertexPositionsBuf().getMem());
        clSetKernelArg1p(k_GenerateMeshVertexBufferKernel, 1, octree.getVertexNormalsBuf().getMem());
        clSetKernelArg1p(k_GenerateMeshVertexBufferKernel, 2, octree.getNodeMaterialsBuf().getMem());
        clSetKernelArg4f(k_GenerateMeshVertexBufferKernel, 3, colour.X, colour.Y, colour.Z, 0.f);
        clSetKernelArg1p(k_GenerateMeshVertexBufferKernel, 4, d_vertexBuffer.getMem());

        PointerBuffer numVerticesWorkSize = BufferUtils.createPointerBuffer(1);
        numVerticesWorkSize.put(0, numVertices);
        int errcode = clEnqueueNDRangeKernel(ctx.getClQueue(), k_GenerateMeshVertexBufferKernel, 1, null, numVerticesWorkSize, null, null, null);
        OCLUtils.checkCLError(errcode);

        meshBufferGPU.setVertices(d_vertexBuffer);
        meshBufferGPU.setCountVertices(numVertices);

        int err = CL10.clReleaseKernel(k_GenerateMeshVertexBufferKernel);
        OCLUtils.checkCLError(err);

        return CL_SUCCESS;
    }

    public int exportMeshBuffer(MeshBufferGPU gpuBuffer, MeshBuffer cpuBuffer){
        //cpuBuffer.setNumVertices(gpuBuffer.getCountVertices());
        //cpuBuffer.setNumIndicates(gpuBuffer.getCountTriangles());

        MeshVertex[] meshVertices = OCLUtils.getVertexBuffer(gpuBuffer.getVertices(), gpuBuffer.getCountVertices());
        bufferGpuService.release(gpuBuffer.getVertices());

        cpuBuffer.setVertices(BufferUtil.createDcFlippedBufferAOS(meshVertices));

        //Vec3i[] triangles = OCLUtils.getTriangles(gpuBuffer.getTriangles(), gpuBuffer.getCountTriangles());
        //cpuBuffer.setIndicates(BufferUtil.createDcFlippedBufferAOS(triangles));
        //cpuBuffer.setIndicates(OCLUtils.getTrianglesAsIntBuffer(meshBufferGPU.getTriangles(), meshBufferGPU.getCountTriangles()));
        return CL_SUCCESS;
    }

    public int findSeamNodesKernel(KernelsHolder kernels, int[] isSeamNode) {
        d_isSeamNode = bufferGpuService.create("d_isSeamNode", Integer.BYTES * octree.getNumNodes(), CL10.CL_MEM_READ_WRITE);
        d_isSeamNodeScan = bufferGpuService.create("d_isSeamNodeScan", Integer.BYTES * octree.getNumNodes(), CL10.CL_MEM_READ_WRITE);

        long k_FindSeamNodesKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "FindSeamNodes", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(k_FindSeamNodesKernel, 0, octree.getNodeCodesBuf().getMem());
        clSetKernelArg1p(k_FindSeamNodesKernel, 1, d_isSeamNode.getMem());

        PointerBuffer numVerticesWorkSize = BufferUtils.createPointerBuffer(1);
        numVerticesWorkSize.put(0, octree.getNumNodes());
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), k_FindSeamNodesKernel, 1, null, numVerticesWorkSize, null, null, null);
        OCLUtils.checkCLError(err);

        int numSeamNodes = scanService.exclusiveScan(d_isSeamNode, d_isSeamNodeScan, octree.getNumNodes());
        if (numSeamNodes <= 0) {
            bufferGpuService.releaseAll();
            err = CL10.clReleaseKernel(k_FindSeamNodesKernel);
            OCLUtils.checkCLError(err);
            return numSeamNodes;
        }
        OCLUtils.getIntBuffer(d_isSeamNode, isSeamNode);
        err = CL10.clReleaseKernel(k_FindSeamNodesKernel);
        OCLUtils.checkCLError(err);
        return numSeamNodes;
    }

    public int gatherSeamNodesFromOctree(KernelsHolder kernels, Vec3i chunkMin, int chunkSize, List<PointerBasedOctreeNode> seamNodes, int numSeamNodes) {
        BufferGpu d_seamNodeInfo = bufferGpuService.create("d_seamNodeInfo", Float.BYTES * 4 * 3 * numSeamNodes, CL10.CL_MEM_READ_WRITE);

        long k_ExtractSeamNodeInfoKernel = clCreateKernel(kernels.getKernel(KernelNames.OCTREE), "ExtractSeamNodeInfo", ctx.getErrcode_ret());
        OCLUtils.checkCLError(ctx.getErrcode_ret());
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 0, d_isSeamNode.getMem());
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 1, d_isSeamNodeScan.getMem());
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 2, octree.getNodeCodesBuf().getMem());
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 3, octree.getNodeMaterialsBuf().getMem());
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 4, octree.getVertexPositionsBuf().getMem());
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 5, octree.getVertexNormalsBuf().getMem());
        clSetKernelArg1p(k_ExtractSeamNodeInfoKernel, 6, d_seamNodeInfo.getMem());

        PointerBuffer extractSeamNodeWorkSize = BufferUtils.createPointerBuffer(1);
        extractSeamNodeWorkSize.put(0, octree.getNumNodes());
        int err = clEnqueueNDRangeKernel(ctx.getClQueue(), k_ExtractSeamNodeInfoKernel, 1, null, extractSeamNodeWorkSize, null, null, null);
        OCLUtils.checkCLError(err);

        OCLUtils.getListSeamNodesTriangles(d_seamNodeInfo, numSeamNodes, chunkMin,
                VoxelHelperUtils.ColourForMinLeafSize(chunkSize), //Constants.Yellow
                chunkSize, seamNodes);

        err = CL10.clReleaseKernel(k_ExtractSeamNodeInfoKernel);
        OCLUtils.checkCLError(err);
        bufferGpuService.release(d_seamNodeInfo);
        bufferGpuService.release(d_isSeamNode);
        bufferGpuService.release(d_isSeamNodeScan);
        bufferGpuService.release(octree.getVertexPositionsBuf());
        bufferGpuService.release(octree.getVertexNormalsBuf());
        bufferGpuService.release(octree.getNodeMaterialsBuf());
        bufferGpuService.release(octree.getNodeCodesBuf());
        return CL_SUCCESS;
    }
}
