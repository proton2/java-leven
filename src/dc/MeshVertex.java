package dc;

import core.math.Vec2f;
import core.math.Vec3f;
import core.model.Vertex;

public class MeshVertex extends Vertex {
    public MeshVertex(Vec3f pos, Vec3f norm, Vec3f color)
    {
        super();
        this.setPos(pos);
        this.setNormal(norm);
        this.setColor(color);
    }
}
