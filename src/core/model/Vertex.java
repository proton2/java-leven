package core.model;

import core.math.Vec3f;

public class Vertex {
	public static final int FLOATS = 12;

	private Vec3f pos;
	private Vec3f normal;
	private Vec3f normal2;

	public Vertex(Vec3f pos, Vec3f color, Vec3f normal, Vec3f normal2) {
		this.pos = pos;
		this.normal = normal;
		this.normal2 = normal2;
		this.color = color;
	}

	public Vertex(Vec3f pos, Vec3f color) {
		this.pos = pos;
		this.color = color;
	}

	private Vec3f color;
	
	public Vertex(){	
	}
	
	public Vertex(Vec3f pos) {
		this.setPos(pos);
		this.setNormal(new Vec3f(0,0,0));
		this.setColor(new Vec3f(0,0,0));
	}

	public Vec3f getNormal2() {
		return normal2;
	}

	public void setNormal2(Vec3f normal2) {
		this.normal2 = normal2;
	}

	public Vec3f getPos() {
		return pos;
	}

	public void setPos(Vec3f pos) {
		this.pos = pos;
	}

	public Vec3f getNormal() {
		return normal;
	}

	public void setNormal(Vec3f normal) {
		this.normal = normal;
	}

	public Vec3f getColor() {
		return color;
	}

	public void setColor(Vec3f color) {
		this.color = color;
	}
}
