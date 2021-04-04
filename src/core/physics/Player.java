package core.physics;

import core.math.Vec3f;

public class Player {
    public RigidBodyCustom              body;
    public PairCachingGhostObjectCustom ghost;
    public Vec3f velocity = new Vec3f();
    public boolean                      falling = true; // падение
    public boolean                      noclip = true; // без столкновений
    public boolean                      jump = false;   // прыжок
}
