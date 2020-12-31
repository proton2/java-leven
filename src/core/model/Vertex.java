package core.model;

import core.math.Vec2f;
import core.math.Vec3f;

public class Vertex {
	public static final int FLOATS = 11;
	private Vec3f pos;
	private Vec3f normal;
	private Vec3f color;
	private Vec2f textureCoord;

	public Vertex(Vec3f pos, Vec3f color, Vec3f normal) {
		this.pos = pos;
		this.normal = normal;
		this.color = color;
	}

	public Vertex(Vec3f pos, Vec3f color) {
		this.pos = pos;
		this.color = color;
	}
	
	public Vertex(){	
	}
	
	public Vertex(Vec3f pos) {
		this.setPos(pos);
		this.setNormal(new Vec3f(0,0,0));
		this.setColor(new Vec3f(0,0,0));
	}

	public Vec2f getTextureCoord() {
		return textureCoord;
	}

	public void setTextureCoord(Vec2f textureCoord) {
		this.textureCoord = textureCoord;
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
