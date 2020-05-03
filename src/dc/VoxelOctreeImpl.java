package dc;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.utils.Constants;
import dc.utils.*;
import dc.svd.QefSolver;
import dc.svd.SVD;

import java.util.*;

import static dc.OctreeNodeType.Node_Internal;
import static dc.OctreeNodeType.Node_Leaf;

public class VoxelOctreeImpl implements VoxelOctree{

    DualContouring dualContouring;

    public VoxelOctreeImpl(DualContouring dualContouring) {
        this.dualContouring = dualContouring;
    }

    private boolean nodeIsSeam(int zi, int yi, int xi, Vec3i leafMin, int voxelPerChunk) {
        //return (leafMin.x==1016 || leafMin.x==1024) && (leafMin.z < -395 && leafMin.z >-433); //show path of seams to debug
        return xi==0||xi==voxelPerChunk-1 || yi==0||yi==voxelPerChunk-1 || zi==0||zi==voxelPerChunk-1;
    }

    @Override
    public EnumMap<VoxelTypes, List<OctreeNode>> createLeafVoxelNodes(ChunkNode chunk, Vec4f[] frustumPlanes,
                                                                      int voxelsPerChunk,
                                                                      RenderDebugCmdBuffer debugRenderBuf,
                                                                      int clipmapLeafSize, int leafSizeScale) {
        List<OctreeNode> voxels = new ArrayList<>();
        List<OctreeNode> seamNodes = new ArrayList<>();
        for (int zi = 0; zi < voxelsPerChunk; zi++) {
            for (int yi = 0; yi < voxelsPerChunk; yi++) {
                for (int xi = 0; xi < voxelsPerChunk; xi++) {
                    int leafSize = (chunk.size / clipmapLeafSize) * leafSizeScale;
                    Vec3i leafMin = new Vec3i(xi, yi, zi).mul(leafSize).add(chunk.min);
                    if (Frustum.cubeInFrustum(frustumPlanes, leafMin.x, leafMin.y, leafMin.z, leafSize)) {
                        OctreeNode leaf = ConstructLeaf(new OctreeNode(leafMin, leafSize, chunk.min, chunk.size));
                        if (leaf != null) {
                            voxels.add(leaf);
                            if(nodeIsSeam(zi, yi, xi, leafMin, voxelsPerChunk)) {
                                seamNodes.add(leaf);
                                if (debugRenderBuf!=null) {
                                    debugRenderBuf.addCube(Constants.White, 0.2f, leafMin, leafSize);
                                }
                            }
                        }
                    }
                }
            }
        }
        EnumMap<VoxelTypes, List<OctreeNode>> res = new EnumMap<>(VoxelTypes.class);
        res.put(VoxelTypes.NODES, voxels);
        res.put(VoxelTypes.SEAMS, seamNodes);
        return res;
    }

    private OctreeNode ConstructLeaf(OctreeNode leaf) {
        if (leaf == null) {
            return null;
        }
        int corners = 0;
        for (int i = 0; i < 8; i++) {
            Vec3f cornerPos = leaf.min.add(CHILD_MIN_OFFSETS[i].mul(leaf.size)).toVec3f();
            float density = Density.Density_Func(cornerPos);
		    int material = density < 0.f ? MATERIAL_SOLID : MATERIAL_AIR;
            corners |= (material << i);
        }
        if (corners == 0 || corners == 255) {
            return null;    // voxel is full inside or outside the volume
        }

        // otherwise the voxel contains the surface, so find the edge intersections
	    int MAX_CROSSINGS = 6;
        int edgeCount = 0;
        Vec3f averageNormal = new Vec3f(0.f);
        QefSolver qef = new QefSolver();
        for (int i = 0; i < 12 && edgeCount < MAX_CROSSINGS; i++) {
		    int c1 = edgevmap[i][0];
		    int c2 = edgevmap[i][1];
		    int m1 = (corners >> c1) & 1;
		    int m2 = (corners >> c2) & 1;
            if ((m1 == MATERIAL_AIR && m2 == MATERIAL_AIR) || (m1 == MATERIAL_SOLID && m2 == MATERIAL_SOLID)) {
                continue; // no zero crossing on this edge
            }
            Vec3f p1 = leaf.min.add(CHILD_MIN_OFFSETS[c1].mul(leaf.size)).toVec3f();
            Vec3f p2 = leaf.min.add(CHILD_MIN_OFFSETS[c2].mul(leaf.size)).toVec3f();
            Vec3f p = ApproximateZeroCrossingPosition(p1, p2);
            Vec3f n = CalculateSurfaceNormal(p);
            qef.add(p, n);
            averageNormal = averageNormal.add(n);
            edgeCount++;
        }

        Vec3f qefPosition = new Vec3f(qef.getMassPoint());
        qef.solve(qefPosition, QEF_ERROR, QEF_SWEEPS, QEF_ERROR);

        OctreeDrawInfo drawInfo = new OctreeDrawInfo();
        drawInfo.position = contains(qefPosition, leaf.min.toVec3f(), leaf.size) ? qefPosition : qef.getMassPoint();
        drawInfo.color = Constants.Red;//isSeamNode(drawInfo.position, leaf.rootMin, leaf.chunkSize, leaf.min, leaf.size); //
        drawInfo.qef = qef.getData();
        drawInfo.averageNormal = averageNormal.div((float)edgeCount);//.normalize();
        SVD.normalize(drawInfo.averageNormal);
        drawInfo.corners = corners;

        leaf.Type = Node_Leaf;
        leaf.drawInfo = drawInfo;
        return leaf;
    }

    private Vec3f isSeamNode(Vec3f pos, Vec3i chunkMin, int chunkSize){
        //pos.Z == chunkMin.z+(chunkSize/100)
        if ((pos.X == chunkMin.x || pos.X == chunkMin.x + chunkSize) || (pos.Z == chunkMin.z || pos.Z == chunkMin.z + chunkSize))
            return new Vec3f(0.f, 0.7f, 0.f);
        else
            return new Vec3f(0.7f, 0.f, 0.f);
    }

    private boolean isOutFromBounds(Vec3f p, Vec3f min, int size) {
        Vec3f max = min.add(size);
        return (p.X < min.X || p.X > max.X ||
                p.Y < min.Y || p.Y > max.Y ||
                p.Z < min.Z || p.Z > max.Z);
    }

    private boolean contains(Vec3f p, Vec3f min, int size) {
        return (p.X >= min.X - BIAS &&
                p.Y >= min.Y - BIAS &&
                p.Z >= min.Z - BIAS &&
                p.X <= min.X + size + BIAS &&
                p.Y <= min.Y + size + BIAS &&
                p.Z <= min.Z + size + BIAS);
    }

    private List<OctreeNode> constructParents(List<OctreeNode> nodes, Vec3i rootMin, int parentSize, int chunkSize) {
        Map<Vec3i, OctreeNode> parentsHash = new HashMap<>();
        for (OctreeNode node : nodes) {
            Vec3i localPos = node.min.sub(rootMin);
            Vec3i parentPos = node.min.sub(new Vec3i(localPos.x % parentSize, localPos.y % parentSize, localPos.z % parentSize));
            OctreeNode parent = parentsHash.get(parentPos);
            if (parent == null) {
                parent = new OctreeNode();
                parent.min = parentPos;
                parent.size = parentSize;
                parent.Type = OctreeNodeType.Node_Internal;
                parent.chunkSize = chunkSize;
                parentsHash.put(parentPos, parent);
            }
            for (int j = 0; j < 8; j++) {
                //Vec3i childMin = parentPos.add(Octree.CHILD_MIN_OFFSETS[j].mul(node.size));
                Vec3i childMin = parentPos.add(VoxelOctree.CHILD_MIN_OFFSETS[j].mul(parentSize / 2));
                if (childMin.equals(node.min)) {
                    parent.children[j] = node;
                    break;
                }
            }
        }
        return new ArrayList<>(parentsHash.values());
    }

    @Override
    public OctreeNode constructTreeUpwards(List<OctreeNode> inputNodes, Vec3i rootMin, int rootNodeSize) {
        List<OctreeNode> sortedNodes = new ArrayList<>(inputNodes);
        sortedNodes.sort(Comparator.comparingInt((OctreeNode lhs) -> lhs.size));
        while (sortedNodes.get(0).size != sortedNodes.get(sortedNodes.size() - 1).size) {
            int iter = 0;
            int size = sortedNodes.get(iter).size;
            do {
                ++iter;
            } while (sortedNodes.get(iter).size == size);

            List<OctreeNode> newNodes = constructParents(sortedNodes.subList(0, iter), rootMin, size * 2, rootNodeSize);
            newNodes.addAll(sortedNodes.subList(iter, sortedNodes.size()));
            sortedNodes.clear();
            sortedNodes.addAll(newNodes);
            newNodes.clear();
        }

        int parentSize = (sortedNodes.get(0).size) * 2;
        while (parentSize <= rootNodeSize) {
            sortedNodes = constructParents(sortedNodes, rootMin, parentSize, rootNodeSize);
            parentSize *= 2;
        }
        if (sortedNodes.size()!=1){
            throw new IllegalStateException("Incorrect octree!");
        }
        if (!(rootMin.x==sortedNodes.get(0).min.x) || !(rootMin.y==sortedNodes.get(0).min.y)|| !(rootMin.z==sortedNodes.get(0).min.z)){
            throw new IllegalStateException("returned root not equal to input root!");
        }
        int octreeCounts = VoxelHelperUtils.countLeafNodes(sortedNodes.get(0));
        if (octreeCounts!=inputNodes.size()){
            throw new IllegalStateException("Octree leafs is not equal to octree counts!");
        }
        return sortedNodes.get(0);
    }

    private Vec3f ApproximateZeroCrossingPosition(Vec3f p0, Vec3f p1) {
        // approximate the zero crossing by finding the min value along the edge
        float minValue = 100000.f;
        float t = 0.f;
        float currentT = 0.f;
        int steps = 8;
        float increment = 1.f / (float)steps;
        while (currentT <= 1.f) {
            Vec3f p = p0.add(p1.sub(p0).mul(currentT)); // p = p0 + ((p1 - p0) * currentT);
            float density = Math.abs(Density.Density_Func(p));
            if (density < minValue) {
                minValue = density;
                t = currentT;
            }
            currentT += increment;
        }
        return p0.add((p1.sub(p0)).mul(t)); // p0 + ((p1 - p0) * t);
    }

    private Vec3f CalculateSurfaceNormal(Vec3f p) {
	    float H = 0.001f;
	    float dx = Density.Density_Func(p.add(new Vec3f(H, 0.f, 0.f))) - Density.Density_Func(p.sub(new Vec3f(H, 0.f, 0.f)));
	    float dy = Density.Density_Func(p.add(new Vec3f(0.f, H, 0.f))) - Density.Density_Func(p.sub(new Vec3f(0.f, H, 0.f)));
	    float dz = Density.Density_Func(p.add(new Vec3f(0.f, 0.f, H))) - Density.Density_Func(p.sub(new Vec3f(0.f, 0.f, H)));
        Vec3f v = new Vec3f(dx, dy, dz);
        SVD.normalize(v);
        return v;
    }

    private void GenerateVertexIndices(OctreeNode node, List<MeshVertex> vertexBuffer) {
        if (node == null) {
            return;
        }
        if (node.Type != Node_Leaf) {
            for (int i = 0; i < 8; i++) {
                GenerateVertexIndices(node.children[i], vertexBuffer);
            }
        }
        if (node.Type != Node_Internal) {
            node.drawInfo.index = vertexBuffer.size();
            vertexBuffer.add(new MeshVertex(node.drawInfo.position, node.drawInfo.averageNormal, node.drawInfo.color));
        }
    }

    @Override
    public void GenerateMeshFromOctree(OctreeNode node, List<MeshVertex> vertexBuffer, List<Integer> indexBuffer, boolean isSeam) {
        if (node == null) {
            return;
        }
        vertexBuffer.clear();
        indexBuffer.clear();
        GenerateVertexIndices(node, vertexBuffer);
        dualContouring.ContourCellProc(node, indexBuffer, isSeam);
    }

    public void DestroyOctree(OctreeNode node) {
        if (node == null) {
            return;
        }
        for (int i = 0; i < 8; i++) {
            DestroyOctree(node.children[i]);
        }

        if (node.drawInfo != null) {
            node.drawInfo = null;
        }
    }
}