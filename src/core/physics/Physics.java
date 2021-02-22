package core.physics;

import core.math.Vec3f;
import dc.ChunkNode;

public interface Physics {
    Vec3f getCollisionNorm();
    Vec3f getCollisionPos();
    void Physics_UpdateWorldNodeMainMesh(boolean updateMain, ChunkNode chunkNode);
    void Physics_CastRay(Vec3f start, Vec3f end);
    void Physics_Shutdown();
    void RemoveMeshData(PhysicsMeshData meshData);
}
