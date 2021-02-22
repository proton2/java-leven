package core.physics;

import com.bulletphysics.collision.dispatch.PairCachingGhostObject;
import com.bulletphysics.dynamics.RigidBody;
import core.math.Vec3f;

public class Player {
    public RigidBody              body;
    public PairCachingGhostObject ghost;
    public Vec3f velocity = new Vec3f();
    public boolean                      falling = true;
    public boolean                      noclip = false;
    public boolean                      jump = false;
}
