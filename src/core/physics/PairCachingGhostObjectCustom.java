package core.physics;

import com.bulletphysics.collision.dispatch.PairCachingGhostObject;
import com.bulletphysics.linearmath.Transform;

public class PairCachingGhostObjectCustom extends PairCachingGhostObject {
    public Transform getWorldTransform(){
        return worldTransform;
    }
}
