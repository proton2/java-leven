package dc;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import svd.QefSolver;
import svd.SVD;
import utils.Frustum;

import java.util.List;

import static dc.OctreeNodeType.Node_Internal;
import static dc.OctreeNodeType.Node_Leaf;
import static dc.OctreeNodeType.Node_Psuedo;

public class Octree {
    public static final int MATERIAL_AIR = 0;
    public static final int MATERIAL_SOLID = 1;
    private static final double BIAS = 1e-1;

    public static final float QEF_ERROR = 1e-6f;
    public static final int QEF_SWEEPS = 4;
    private static Vec4f[] frustumPlanes;

    public static final Vec3i[] CHILD_MIN_OFFSETS = {
            // needs to match the vertMap from Dual Contouring impl
            new Vec3i( 0, 0, 0 ),
            new Vec3i( 0, 0, 1 ),
            new Vec3i( 0, 1, 0 ),
            new Vec3i( 0, 1, 1 ),
            new Vec3i( 1, 0, 0 ),
            new Vec3i( 1, 0, 1 ),
            new Vec3i( 1, 1, 0 ),
            new Vec3i( 1, 1, 1 ),
    };

    public static final int[][] edgevmap = {
        {0,4},{1,5},{2,6},{3,7},	// x-axis
        {0,2},{1,3},{4,6},{5,7},	// y-axis
        {0,1},{2,3},{4,5},{6,7}		// z-axis
    };

    // -------------------------------------------------------------------------------
    public static OctreeNode ConstructLeaf(OctreeNode leaf) {
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
        drawInfo.color = new Vec3f(0.7f, 0.f, 0.f);//isSeamNode(drawInfo.position, leaf.rootMin, leaf.chunkSize);
        drawInfo.qef = qef.getData();
        drawInfo.averageNormal = averageNormal.div((float)edgeCount);//.normalize();
        SVD.normalize(drawInfo.averageNormal);
        drawInfo.corners = corners;

        leaf.Type = Node_Leaf;
        leaf.drawInfo = drawInfo;
        return leaf;
    }

//    private static Vec3f isSeamNode(Vec3f pos, Vec3i min, int size){
//        if (    (pos.X <= min.x || pos.X >= min.x + size-1) ||
//                //(pos.Y <= min.y || pos.Y >= min.y + size-1) ||
//                (pos.Z <= min.z || pos.Z >= min.z + size-1))
//            return new Vec3f(0.f, 0.7f, 0.f);
//        else
//            return new Vec3f(0.7f, 0.f, 0.f);
//    }

    private static Vec3f isSeamNode(Vec3f pos, Vec3i min, int size){
        if (    (pos.X <= min.x+(size/100) || pos.X >= min.x + size) ||
                //(pos.Y == min.y || pos.Y >= min.y + size) ||
                (pos.Z <= min.z+(size/100) || pos.Z >= min.z + size))
            return new Vec3f(0.f, 0.7f, 0.f);
        else
            return new Vec3f(0.7f, 0.f, 0.f);
    }

    private static boolean isOutFromBounds(Vec3f p, Vec3f min, int size) {
        Vec3f max = min.add(size);
        return (p.X < min.X || p.X > max.X ||
                p.Y < min.Y || p.Y > max.Y ||
                p.Z < min.Z || p.Z > max.Z);
    }

    private static boolean contains(Vec3f p, Vec3f min, int size) {
        return (p.X >= min.X - BIAS &&
                p.Y >= min.Y - BIAS &&
                p.Z >= min.Z - BIAS &&
                p.X <= min.X + size + BIAS &&
                p.Y <= min.Y + size + BIAS &&
                p.Z <= min.Z + size + BIAS);
    }

    private static OctreeNode SimplifyOctree(OctreeNode node, float threshold) {
        if (node == null) {
            return null;
        }
        if (node.Type != Node_Internal) {    // can't simplify!
            return node;
        }

        QefSolver qef = new QefSolver();
        int[] signs = { -1, -1, -1, -1, -1, -1, -1, -1 };
        int midsign = -1;
        boolean isCollapsible = true;

        for (int i = 0; i < 8; i++) {
            node.children[i] = SimplifyOctree(node.children[i], threshold);
            if (node.children[i] != null) {
                OctreeNode child = node.children[i];
                if (child.Type == Node_Internal) {
                    isCollapsible = false;
                }
                else {
                    qef.add(child.drawInfo.qef);
                    midsign = (child.drawInfo.corners >> (7 - i)) & 1;
                    signs[i] = (child.drawInfo.corners >> i) & 1;
                }
            }
        }

        if (!isCollapsible) {
            return node;    // at least one child is an internal node, can't collapse
        }

        Vec3f position = new Vec3f();
        float error = qef.solve(position, QEF_ERROR, QEF_SWEEPS, QEF_ERROR);
        //float error = qef.getError();

        // at this point the masspoint will actually be a sum, so divide to make it the average
        if (error > threshold) {
            return node;    // this collapse breaches the threshold
        }

        // change the node from an internal node to a 'psuedo leaf' node
        OctreeDrawInfo drawInfo = new OctreeDrawInfo();

        for (int i = 0; i < 8; i++) {
            if (signs[i] == -1) {
                drawInfo.corners |= (midsign << i); // Undetermined, use centre sign instead
            }
            else {
                drawInfo.corners |= (signs[i] << i);
            }
        }

        drawInfo.averageNormal = new Vec3f(0);
        for (int i = 0; i < 8; i++) {
            if (node.children[i] != null) {
                OctreeNode child = node.children[i];
                if (child.Type == Node_Psuedo || child.Type == Node_Leaf) {
                    drawInfo.averageNormal = drawInfo.averageNormal.add(child.drawInfo.averageNormal);
                }
            }
        }

        SVD.normalize(drawInfo.averageNormal);
        drawInfo.position = contains(position, node.min.toVec3f(), node.size) ? position : qef.getMassPoint();
        drawInfo.color = isSeamNode(drawInfo.position, node.min, node.size);
        drawInfo.qef = qef.getData();

        for (int i = 0; i < 8; i++) {
            DestroyOctree(node.children[i]);
            node.children[i] = null;
        }

        node.Type = Node_Psuedo;
        node.drawInfo = drawInfo;
        return node;
    }

    public static Vec3f ApproximateZeroCrossingPosition(Vec3f p0, Vec3f p1) {
        // approximate the zero crossing by finding the min value along the edge
        float minValue = 100000.f;
        float t = 0.f;
        float currentT = 0.f;
        int steps = 8;
        float increment = 1.f / (float)steps;
        while (currentT <= 1.f)
        {
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

    public static Vec3f CalculateSurfaceNormal(Vec3f p) {
	    float H = 0.001f;
	    float dx = Density.Density_Func(p.add(new Vec3f(H, 0.f, 0.f))) - Density.Density_Func(p.sub(new Vec3f(H, 0.f, 0.f)));
	    float dy = Density.Density_Func(p.add(new Vec3f(0.f, H, 0.f))) - Density.Density_Func(p.sub(new Vec3f(0.f, H, 0.f)));
	    float dz = Density.Density_Func(p.add(new Vec3f(0.f, 0.f, H))) - Density.Density_Func(p.sub(new Vec3f(0.f, 0.f, H)));
        Vec3f v = new Vec3f(dx, dy, dz);
        SVD.normalize(v);
        return v;
    }

    private static void GenerateVertexIndices(OctreeNode node, List<MeshVertex> vertexBuffer) {
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

    private static OctreeNode ConstructOctreeNodes(OctreeNode node)
    {
        if (node == null) {
            return null;
        }

        if (!Frustum.cubeInFrustum(frustumPlanes, node.min.x, node.min.y, node.min.z, node.size)){
            return null;
        }

        if (node.size == 1) {
            return ConstructLeaf(node);
        }

	    int childSize = node.size / 2;
        boolean hasChildren = false;

        for (int i = 0; i < 8; i++) {
            OctreeNode child = new OctreeNode();
            child.size = childSize;
            child.min = node.min.add(CHILD_MIN_OFFSETS[i].mul(childSize));
            child.Type = Node_Internal;
            child.rootMin = node.rootMin;
            child.chunkSize = node.chunkSize;

            node.children[i] = ConstructOctreeNodes(child);
            hasChildren |= (node.children[i] != null);
        }

        if (!hasChildren) {
            return null;
        }

        return node;
    }

    public static OctreeNode BuildOctree(Vec4f[] frustPlanes, Vec3i min, int size, float threshold) {
        frustumPlanes = frustPlanes;
        OctreeNode root = new OctreeNode();
        root.min = min;
        root.size = size;
        root.Type = Node_Internal;

        root.rootMin = min;
        root.chunkSize = size;

        root = ConstructOctreeNodes(root);
        root = SimplifyOctree(root, threshold);
        return root;
    }

    public static void GenerateMeshFromOctree(OctreeNode node, List<MeshVertex> vertexBuffer, List<Integer> indexBuffer) {
        if (node == null) {
            return;
        }

        vertexBuffer.clear();
        indexBuffer.clear();

        GenerateVertexIndices(node, vertexBuffer);
        Dc.ContourCellProc(node, indexBuffer);
    }

    public static void DestroyOctree(OctreeNode node) {
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