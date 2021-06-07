package dc.impl.simplifier;

import core.math.Vec3f;
import core.math.Vec4f;
import core.math.Vec4i;
import core.utils.BufferUtil;
import dc.entities.MeshBuffer;
import dc.entities.MeshVertex;
import dc.solver.LevenQefSolver;
import dc.solver.QEFData;

import java.util.*;

import static java.lang.Math.max;
import static java.lang.Math.min;

public class NgMeshSimplify {

    private static final int COLLAPSE_MAX_DEGREE = 16;
    private static final int MAX_TRIANGLES_PER_VERTEX = COLLAPSE_MAX_DEGREE;
    private static final int QEF_MAX_INPUT_COUNT = 12;

    static QEFData qef = new QEFData(new LevenQefSolver());

    public void ngMeshSimplifier(MeshBuffer mesh, Vec4i worldSpaceOffset, MeshSimplificationOptions options) {
        if (mesh.getNumIndicates()/3 < 100 || mesh.getNumVertices() < 100) {
            return;
        }

        ArrayList<MeshVertex> vertices = new ArrayList<>(mesh.getNumVertices());
        for (int i = 0; i < mesh.getNumVertices(); i++) {
            int index = i * 9;
            float x = mesh.getVertices().get(index + 0);
            float y = mesh.getVertices().get(index + 1);
            float z = mesh.getVertices().get(index + 2);

            x -= worldSpaceOffset.x;
            y -= worldSpaceOffset.y;
            z -= worldSpaceOffset.z;

            MeshVertex meshVertex = new MeshVertex();
            meshVertex.setPos(new Vec3f(x, y, z));
            vertices.add(meshVertex);
        }
        mesh.getVertices().flip();

        ArrayList<MeshTriangle> triangles = new ArrayList<>(mesh.getNumIndicates()/3);
        for (int i = 0; i < mesh.getNumIndicates(); i++) {
            int index = i * 3;
            int i1 = mesh.getIndicates().get(index + 0);
            int i2 = mesh.getIndicates().get(index + 1);
            int i3 = mesh.getIndicates().get(index + 2);
            MeshTriangle meshTriangle = new MeshTriangle(i1, i2, i3);
            triangles.add(meshTriangle);
        }
        mesh.getIndicates().flip();

        mesh.setNumVertices(0);
        mesh.setNumIndicates(0);

        ArrayList<Edge> edges = new ArrayList<>(triangles.size() * 3);
        BuildCandidateEdges(vertices, triangles, edges);

        ArrayList<Vec3f> collapsePosition = new ArrayList<>(edges.size());
        ArrayList<Vec3f> collapseNormal = new ArrayList<>(edges.size());
        ArrayList<Integer> collapseValid = new ArrayList<>(edges.size());
        ArrayList<Integer> collapseEdgeID = new ArrayList<>(vertices.size());
        ArrayList<Integer> collapseTarget = new ArrayList<>(vertices.size());

        ArrayList<Edge> edgeBuffer = new ArrayList<>(edges.size());
        ArrayList<MeshTriangle> triBuffer = new ArrayList<>(triangles.size());

        // per vertex
        ArrayList<Integer> vertexTriangleCounts = new ArrayList<>(vertices.size());
        for (MeshTriangle triangle : triangles) {
            int[] indices = triangle.indices;
            for (int index = 0; index < 3; index++) {
                vertexTriangleCounts.set(indices[index], vertexTriangleCounts.get(indices[index]) + 1);
            }
        }

        int targetTriangleCount = (int) (triangles.size() * options.targetPercentage);
        int iterations = 0;

        while (triangles.size() > targetTriangleCount && iterations++ < options.maxIterations) {
            for(int i=0; i<vertices.size(); i++){
                collapseEdgeID.set(i, -1);
            }
            for(int i=0; i<vertices.size(); i++){
                collapseTarget.set(i, -1);
            }

            collapseValid.clear();
		    int countValidCollapse = FindValidCollapses(
                options,
                edges, vertices, triangles, vertexTriangleCounts, collapseValid,
                collapseEdgeID, collapsePosition, collapseNormal);
            if (countValidCollapse == 0) {
                break;
            }

            CollapseEdges(collapseValid, edges,
                    collapseEdgeID, collapsePosition, collapseNormal, vertices,
                    collapseTarget);

            RemoveTriangles(vertices, collapseTarget, triangles, triBuffer, vertexTriangleCounts);
            RemoveEdges(collapseTarget, edges, edgeBuffer);
        }

        mesh.setNumIndicates(0);
//        for (int i = 0; i < triangles.size(); i++) {
//            mesh->triangles[mesh->numTriangles].indices_[0] = triangles[i].indices_[0];
//            mesh->triangles[mesh->numTriangles].indices_[1] = triangles[i].indices_[1];
//            mesh->triangles[mesh->numTriangles].indices_[2] = triangles[i].indices_[2];
//            mesh->numTriangles++;
//        }
        for (int i = 0; i < triangles.size(); i++) {
            int index = i * 3;
            mesh.getIndicates().put(index + 0, triangles.get(index + 0).indices[0]);
            mesh.getIndicates().put(index + 1, triangles.get(index + 1).indices[1]);
            mesh.getIndicates().put(index + 2, triangles.get(index + 2).indices[2]);
        }
        mesh.setNumIndicates(triangles.size());
        CompactVertices(vertices, mesh);

        mesh.setNumVertices(vertices.size());
        mesh.setVertices(BufferUtil.createDcFlippedBufferAOS(vertices, worldSpaceOffset));
    }

    static void BuildCandidateEdges(ArrayList<MeshVertex> vertices, ArrayList<MeshTriangle> triangles, ArrayList<Edge> edges) {
        for (MeshTriangle triangle : triangles) {
            int[] indices = triangle.indices;
            edges.add(new Edge(min(indices[0], indices[1]), max(indices[0], indices[1])));
            edges.add(new Edge(min(indices[1], indices[2]), max(indices[1], indices[2])));
            edges.add(new Edge(min(indices[0], indices[2]), max(indices[0], indices[2])));
        }
        edges.sort(Comparator.comparingLong(lhs -> lhs.idx));

        ArrayList<Edge> filteredEdges = new ArrayList<>(edges.size());
        ArrayList<Boolean> boundaryVerts = new ArrayList<>(vertices.size());

        Edge prev = edges.get(0);
        int count = 1;
        for (int idx = 1; idx < edges.size(); idx++) {
            Edge curr = edges.get(idx);
            if (curr.idx != prev.idx) {
                if (count == 1) {
                    boundaryVerts.set(prev.min, true);
                    boundaryVerts.set(prev.max, true);
                }
                else {
                    filteredEdges.add(prev);
                }
                count = 1;
            }
            else {
                count++;
            }
            prev = curr;
        }
        edges.clear();
        for (Edge edge: filteredEdges) {
            if (!boundaryVerts.get(edge.min) && !boundaryVerts.get(edge.max)) {
                edges.add(edge);
            }
        }
    }

    private static int RemoveTriangles(ArrayList<MeshVertex> vertices,
                                       ArrayList<Integer> collapseTarget,
                                       ArrayList<MeshTriangle> tris,
                                       ArrayList<MeshTriangle> triBuffer,
                                       ArrayList<Integer> vertexTriangleCounts) {
        int removedCount = 0;
        vertexTriangleCounts.clear();
        triBuffer.clear();

        for (MeshTriangle tri: tris) {
            for (int j = 0; j < 3; j++) {
			    int t = collapseTarget.get(tri.indices[j]);
                if (t != -1) {
                    tri.indices[j] = t;
                }
            }
            if (tri.indices[0] == tri.indices[1] ||
                    tri.indices[0] == tri.indices[2] ||
                    tri.indices[1] == tri.indices[2]) {
                removedCount++;
                continue;
            }
		    int[] indices = tri.indices;
            for (int index = 0; index < 3; index++) {
                vertexTriangleCounts.set(indices[index], vertexTriangleCounts.get(indices[index]) +1);
            }
            triBuffer.add(tri);
        }
        swap(tris, triBuffer);
        return removedCount;
    }

    public static <T>void swap(List<T> list1, List<T> list2){
        List<T> tmpList = new ArrayList<>(list1);
        list1.clear();
        list1.addAll(list2);
        list2.clear();
        list2.addAll(tmpList);
    }

    private static void RemoveEdges(ArrayList<Integer> collapseTarget, ArrayList<Edge> edges, ArrayList<Edge> edgeBuffer) {
        edgeBuffer.clear();
        for (Edge edge: edges) {
            int t = collapseTarget.get(edge.min);
            if (t != -1) {
                edge.min = t;
            }

            t = collapseTarget.get(edge.max);
            if (t != -1) {
                edge.max = t;
            }

            if (edge.min != edge.max)
            {
                edgeBuffer.add(edge);
            }
        }
        swap(edges, edgeBuffer);
    }

    static void CollapseEdges(
            ArrayList<Integer> collapseValid,
            ArrayList<Edge> edges,
            ArrayList<Integer> collapseEdgeID,
            ArrayList<Vec3f> collapsePositions,
            ArrayList<Vec3f> collapseNormal,
            ArrayList<MeshVertex> vertices,
            ArrayList<Integer> collapseTarget)
    {
        int countCollapsed = 0, countCandidates = 0;
        for (int i: collapseValid) {
            countCandidates++;
		    Edge edge = edges.get(i);
            if (collapseEdgeID.get(edge.min) == i && collapseEdgeID.get(edge.max) == i) {
                countCollapsed++;
                collapseTarget.set(edge.max, edge.min);
                vertices.get(edge.min).setPos(collapsePositions.get(i));
                vertices.get(edge.min).setNormal(collapseNormal.get(i));
            }
        }
    }

    public static int randomGet(Random rnd, int min, int max) // get random number from
    // min to max (not max-1 !)
    {
        return min + (int) Math.floor(rnd.nextDouble() * (max - min + 1));
    }

    static int FindValidCollapses(
	    MeshSimplificationOptions options,
        ArrayList<Edge> edges,
        ArrayList<MeshVertex> vertices,
        ArrayList<MeshTriangle> tris,
        ArrayList<Integer> vertexTriangleCounts,
        ArrayList<Integer> collapseValid,
        ArrayList<Integer> collapseEdgeID,
        ArrayList<Vec3f> collapsePosition,
        ArrayList<Vec3f> collapseNormal)
    {
        int validCollapses = 0;
        int numRandomEdges = edges.size();
        Random r = new Random(42);
        ArrayList<Integer> randomEdges = new ArrayList<>(numRandomEdges);
        for (int i = 0; i < numRandomEdges; i++) {
		    int randomIdx = randomGet(r, 0, (edges.size() - 1));
            randomEdges.add(randomIdx);
        }
        Collections.sort(randomEdges);

        ArrayList<Float> minEdgeCost = new ArrayList<>(vertices.size());
        for (int i=0; i<vertices.size(); i++){
            minEdgeCost.set(i, Float.MAX_VALUE);
        }

        for (int i: randomEdges) {
		    Edge edge = edges.get(i);
		    MeshVertex vMin = vertices.get(edge.min);
            MeshVertex vMax = vertices.get(edge.max);

            // prevent collapses along edges
            float cosAngle = vMin.getNormal().dot(vMax.getNormal());
            if (cosAngle < options.minAngleCosine) {
                continue;
            }

		    float edgeSize = vMax.getPos().sub(vMin.getPos()).lengthSquared();
            if (edgeSize > (options.maxEdgeSize * options.maxEdgeSize)) {
                continue;
            }

            if (Math.abs(vMin.getColor().Z - vMax.getColor().getZ()) > 1e-3) { // [ToDo] color.w ???
                continue;
            }

		    int degree = vertexTriangleCounts.get(edge.min) + vertexTriangleCounts.get(edge.max);
            if (degree > COLLAPSE_MAX_DEGREE) {
                continue;
            }

            Vec4f[] positions = {new Vec4f(vMin.getPos()), new Vec4f(vMax.getPos())};
            Vec4f[] normals = {new Vec4f(vMin.getNormal()), new Vec4f(vMax.getNormal())};
            qef.qef_create_from_points(positions, normals, 2);
            Vec4f solvedPos = qef.solve();
            float[] pos = solvedPos.to1dArray();
            float error = qef.getError();
            if (error > 0.f) {
                error = 1.f / error;
            }

            // avoid vertices becoming a 'hub' for lots of edges by penalising collapses
            // which will lead to a vertex with degree > 10
		    int penalty = Math.max(0, degree - 10);
            error += penalty * (options.maxError * 0.1f);
            if (error > options.maxError) {
                continue;
            }

            collapseValid.add(i);

            collapseNormal.set(i, vMin.getNormal().sub(vMax.getNormal())).mul(0.5f);
            collapsePosition.set(i, new Vec3f(pos[0], pos[1], pos[2]));

            if (error < minEdgeCost.get(edge.min)) {
                minEdgeCost.set(edge.min, error);
                collapseEdgeID.set(edge.min, i);
            }

            if (error < minEdgeCost.get(edge.max)) {
                minEdgeCost.set(edge.max, error);
                collapseEdgeID.set(edge.max, i);
            }
            validCollapses++;
        }
        return validCollapses;
    }

    static void CompactVertices(ArrayList<MeshVertex> vertices, MeshBuffer meshBuffer) {
        ArrayList<Boolean> vertexUsed = new ArrayList<>(vertices.size());

//        for (int i = 0; i < meshBuffer->numTriangles; i++) {
//            MeshTriangle tri = meshBuffer->triangles[i];
//            vertexUsed.set(tri.indices[0], true);
//            vertexUsed.set(tri.indices[1], true);
//            vertexUsed.set(tri.indices[2], true);
//        }

        for (int i = 0; i < meshBuffer.getNumIndicates(); i++) {
            int tri = meshBuffer.getIndicates().get(i);
            vertexUsed.set(tri, true);
        }

        ArrayList<MeshVertex> compactVertices = new ArrayList<>(vertices.size());
        ArrayList<Integer> remappedVertexIndices = new ArrayList<>(vertices.size());
        for(int i=0; i<vertices.size(); i++){
            remappedVertexIndices.set(i, -1);
        }

        for (int i = 0; i < vertices.size(); i++) {
            if (vertexUsed.get(i)) {
                remappedVertexIndices.set(i, compactVertices.size());
                //vertices[i].normal = vertices.get(i).getNormal();
                compactVertices.add(vertices.get(i));
            }
        }

//        for (int i = 0; i < meshBuffer->numTriangles; i++) {
//            MeshTriangle tri = meshBuffer.triangles[i];
//            for (int j = 0; j < 3; j++) {
//                tri.indices[j] = remappedVertexIndices.get(tri.indices[j]);
//            }
//        }

        for (int i = 0; i < meshBuffer.getNumIndicates(); i++) {
            meshBuffer.getIndicates().put(i, remappedVertexIndices.get(meshBuffer.getIndicates().get(i)));
        }

        swap(vertices, compactVertices);
    }
}
