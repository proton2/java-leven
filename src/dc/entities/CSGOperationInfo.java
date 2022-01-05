package dc.entities;

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

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        CSGOperationInfo that = (CSGOperationInfo) o;

        if (type != that.type) return false;
        if (material != that.material) return false;
        if (Float.compare(that.rotateY, rotateY) != 0) return false;
        if (brushShape != that.brushShape) return false;
        if (origin != null ? !origin.equals(that.origin) : that.origin != null) return false;
        return dimensions != null ? dimensions.equals(that.dimensions) : that.dimensions == null;
    }

    @Override
    public int hashCode() {
        int result = type;
        result = 31 * result + (brushShape != null ? brushShape.hashCode() : 0);
        result = 31 * result + material;
        result = 31 * result + (rotateY != +0.0f ? Float.floatToIntBits(rotateY) : 0);
        result = 31 * result + (origin != null ? origin.hashCode() : 0);
        result = 31 * result + (dimensions != null ? dimensions.hashCode() : 0);
        return result;
    }
}
