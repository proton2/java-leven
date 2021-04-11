package dc.entities;

import core.buffers.VBO;
import core.math.Transform;
import core.math.Vec3f;

public class ModelEntity {
    private VBO vbo;
    private Transform transform;

    public ModelEntity(VBO vbo) {
        this.vbo = vbo;
        transform = new Transform();
    }

    public void setTranslation(Vec3f translation) {
        transform.setTranslation(translation);
    }

    public void setRotation(Vec3f rotation) {
        transform.setRotation(rotation);
    }

    public void setScaling(Vec3f scaling) {
        transform.setScaling(scaling);
    }

    public VBO getVbo() {
        return vbo;
    }

    public Transform getTransform() {
        return transform;
    }
}
