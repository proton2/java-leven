package core.physics;

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.Transform;

public class RigidBodyCustom extends RigidBody {
    public RigidBodyCustom(RigidBodyConstructionInfo constructionInfo) {
        super(constructionInfo);
    }

    public Transform getWorldTransform(){
        return worldTransform;
    }
}
