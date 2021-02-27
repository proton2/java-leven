package core.physics;

import core.math.Vec3f;
import dc.ChunkNode;

public interface Physics {
    float PLAYER_WIDTH = 48.f;
    float PLAYER_HEIGHT = 120.f - PLAYER_WIDTH;   // the capsule height is (2 * radius + height)

    void Physics_TogglePlayerNoClip();
    void Physics_PlayerJump();
    Vec3f getCollisionNorm();
    Vec3f getCollisionPos();
    void Physics_SetPlayerVelocity(Vec3f velocity);
    void Physics_UpdateWorldNodeMainMesh(boolean updateMain, ChunkNode chunkNode);
    void Physics_CastRay(Vec3f start, Vec3f end);
    void Physics_Shutdown();
    void RemoveMeshData(PhysicsMeshData meshData);
    Vec3f Physics_GetPlayerPosition();
    void Physics_SpawnPlayer(Vec3f origin);
}
