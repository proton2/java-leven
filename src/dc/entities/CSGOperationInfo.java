package dc.entities;

import core.math.Vec3f;
import core.math.Vec4f;
import dc.utils.RenderShape;

public class CSGOperationInfo {
    private int				type = 0;
    private RenderShape		brushShape = RenderShape.RenderShape_Cube;
    private int				material = 0;
    private float			rotateY = 0.f;
    private Vec4f           origin;
    private Vec4f   		dimensions;

    public CSGOperationInfo(){ }

    public CSGOperationInfo(Vec4f origin, Vec4f dimensions) {
        this.origin = origin;
        this.dimensions = dimensions;
    }

    public int getType() {
        return type;
    }

    public void setType(int type) {
        this.type = type;
    }

    public RenderShape getBrushShape() {
        return brushShape;
    }

    public void setBrushShape(RenderShape brushShape) {
        this.brushShape = brushShape;
    }

    public int getMaterial() {
        return material;
    }

    public void setMaterial(int material) {
        this.material = material;
    }

    public float getRotateY() {
        return rotateY;
    }

    public void setRotateY(float rotateY) {
        this.rotateY = rotateY;
    }

    public Vec4f getOrigin() {
        return origin;
    }

    public void setOrigin(Vec4f origin) {
        this.origin = origin;
    }

    public Vec4f getDimensions() {
        return dimensions;
    }

    public void setDimensions(Vec4f dimensions) {
        this.dimensions = dimensions;
    }
}
