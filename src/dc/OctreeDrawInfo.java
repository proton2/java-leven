package dc;

import core.math.Vec3f;

public class OctreeDrawInfo {
    public Vec3f position;
    public Vec3f color;
    public Vec3f averageNormal;

    public Vec3f getPosition() {
        return position;
    }

    public void setPosition(Vec3f position) {
        this.position = position;
    }

    public Vec3f getAverageNormal() {
        return averageNormal;
    }

    public void setAverageNormal(Vec3f averageNormal) {
        this.averageNormal = averageNormal;
    }

    public Vec3f getColor() {
        return color;
    }

    public void setColor(Vec3f color) {
        this.color = color;
    }
}
