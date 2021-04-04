package core.physics;

//import com.badlogic.gdx.physics.bullet.collision.btBvhTriangleMeshShape;
//import com.badlogic.gdx.physics.bullet.collision.btCollisionObject;
//import com.badlogic.gdx.physics.bullet.collision.btTriangleIndexVertexArray;
import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.TriangleIndexVertexArray;
import com.bulletphysics.dynamics.RigidBody;
//import com.jme3.bullet.collision.shapes.MeshCollisionShape;
//import com.jme3.bullet.collision.shapes.infos.CompoundMesh;
//import com.jme3.bullet.objects.PhysicsRigidBody;

public class PhysicsMeshData{
    public TriangleIndexVertexArray		buffer;
    public BvhTriangleMeshShape			shape;
    public RigidBody					body;

//    public CompoundMesh         nativeBuffer;
//    public MeshCollisionShape   nativeShape;
//    public PhysicsRigidBody     nativeBody;
//
//    public btTriangleIndexVertexArray gdxbuffer;
//    public btBvhTriangleMeshShape gdxShape;
//    public btCollisionObject gdxBody;
}
