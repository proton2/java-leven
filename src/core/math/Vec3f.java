package core.math;

import org.joml.Vector3f;
import org.joml.Vector3fc;

public class Vec3f {
	
	public float X;
	public float Y;
	public float Z;
	
	public Vec3f()
	{
		this.setX(0);
		this.setY(0);
		this.setZ(0);
	}
	
	public Vec3f(float x, float y, float z)
	{
		this.setX(x);
		this.setY(y);
		this.setZ(z);
	}

	public Vec3f(float t)
	{
		this.setX(t);
		this.setY(t);
		this.setZ(t);
	}

	public float[] convTo1dFloat(){
		return new float [] {X,Y,Z};
	}

	public Vec3f(Vec3f v)
	{
		this.X = v.getX();
		this.Y = v.getY();
		this.Z = v.getZ();
	}

	public Vec3f(Vec3i v)
	{
		this.X = v.x;
		this.Y = v.y;
		this.Z = v.z;
	}

	public void set(Vec3f v)
	{
		this.X = v.getX();
		this.Y = v.getY();
		this.Z = v.getZ();
	}

	public void set(float x, float y, float z)
	{
		this.setX(x);
		this.setY(y);
		this.setZ(z);
	}

	public void set(float v) {
		this.X = v; this.Y = v; this.Z = v;
	}
	
	public float length()
	{
		return (float) Math.sqrt(X*X + Y*Y + Z*Z);
	}

	public float lengthSquared() {
		return X * X + Y * Y + Z * Z;
	}
	
	public float dot(Vec3f r)
	{
		return X * r.getX() + Y * r.getY() + Z * r.getZ();
	}
	
	public Vec3f cross(Vec3f r)
	{
		float x = Y * r.getZ() - Z * r.getY();
		float y = Z * r.getX() - X * r.getZ();
		float z = X * r.getY() - Y * r.getX();
		
		return new Vec3f(x,y,z);
	}

	public Vec3f neg() {
		return new Vec3f(-X, Y, Z);
	}
	
	public Vec3f normalize()
	{
		float length = this.length();
		
		X /= length;
		Y /= length;
		Z /= length;
		
		return this;
	}

	public Vec3f scale(float s) {
		X *= s;
		Y *= s;
		Z *= s;
		return this;
	}
	
	public Vec3f rotate(float angle, Vec3f axis)
	{
		float sinHalfAngle = (float)Math.sin(Math.toRadians(angle / 2));
		float cosHalfAngle = (float)Math.cos(Math.toRadians(angle / 2));
		
		float rX = axis.getX() * sinHalfAngle;
		float rY = axis.getY() * sinHalfAngle;
		float rZ = axis.getZ() * sinHalfAngle;
		float rW = cosHalfAngle;
		
		Quaternion rotation = new Quaternion(rX, rY, rZ, rW);
		Quaternion conjugate = rotation.conjugate();
		
		Quaternion w = rotation.mul(this).mul(conjugate);
		
		X = w.getX();
		Y = w.getY();
		Z = w.getZ();
		
		return this;
	}
	
	public Vec3f add(Vec3f r)
	{
		return new Vec3f(this.X + r.getX(), this.Y + r.getY(), this.Z + r.getZ());
	}

	public Vec3i add(Vec3i r)
	{
		return new Vec3i(this.X + r.x, this.Y + r.y, this.Z + r.z);
	}
	
	public Vec3f add(float r)
	{
		return new Vec3f(this.X + r, this.Y + r, this.Z + r);
	}
	
	public Vec3f sub(Vec3f r)
	{
		return new Vec3f(this.X - r.getX(), this.Y - r.getY(), this.Z - r.getZ());
	}
	
	public Vec3f sub(float r)
	{
		return new Vec3f(this.X - r, this.Y - r, this.Z - r);
	}
	
	public Vec3f mul(Vec3f r)
	{
		return new Vec3f(this.X * r.getX(), this.Y * r.getY(), this.Z * r.getZ());
	}
	
	public Vec3f mul(float x, float y, float z)
	{
		return new Vec3f(this.X * x, this.Y * y, this.Z * z);
	}
	
	public Vec3f mul(float r)
	{
		return new Vec3f(this.X * r, this.Y * r, this.Z * r);
	}

	public Vec3i mul3i(float r)
	{
		return new Vec3i(this.X * r, this.Y * r, this.Z * r);
	}
	
	public Vec3f div(Vec3f r)
	{
		return new Vec3f(this.X / r.getX(), this.Y / r.getY(), this.getZ() / r.getZ());
	}
	
	public Vec3f div(float r)
	{
		return new Vec3f(this.X / r, this.Y / r, this.Z / r);
	}
	
	public Vec3f abs()
	{
		return new Vec3f(Math.abs(X), Math.abs(Y), Math.abs(Z));
	}

	public Vec3f getNormalDominantAxis() {
		// use the dominant axis of the node
		Vec3f dir = new Vec3f();
		Vec3f absNormal = this.abs();
		if (absNormal.Y >= absNormal.X) {
			if (absNormal.Y >= absNormal.Z) {
				dir.Y = this.Y;
			}
			else {
				dir.Z = this.Z;
			}
		}
		else {
			if (absNormal.X >= absNormal.Z) {
				dir.X = this.X;
			}
			else {
				dir.Z = this.Z;
			}
		}
		return dir.normalize();
	}
	
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		Vec3f other = (Vec3f) obj;
		if (X != other.getX()) return false;
		if (Y != other.getY()) return false;
		if (Z != other.getZ()) return false;
		return true;
	}

	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + Float.floatToIntBits(X);
		result = prime * result + Float.floatToIntBits(Y);
		result = prime * result + Float.floatToIntBits(Z);
		return result;
	}
	
	public String toString()
	{
		return "[" + this.X + "," + this.Y + "," + this.Z + "]";
	}

	public float getX() {
		return X;
	}

	public void setX(float x) {
		X = x;
	}

	public float getY() {
		return Y;
	}

	public void setY(float y) {
		Y = y;
	}

	public float getZ() {
		return Z;
	}

	public void setZ(float z) {
		Z = z;
	}

	public static Vec3f max(Vec3f v, Vec3f dest) {
		return new Vec3f(Math.max(dest.X, v.X), Math.max(dest.Y, v.Y), Math.max(dest.Z, v.Z));
	}

	public float[] to1dArray(){
		return new float[] {X, Y, Z};
	}

	public Vec3i getVec3i(){
		return new Vec3i(this.X, this.Y, this.Z);
	}

	public Vec3f scaleAdd(float scalar, Vec3f add) {
		Vec3f res = new Vec3f();
		res.X = X * scalar + add.X;
		res.Y = Y * scalar + add.Y;
		res.Z = Z * scalar + add.Z;
		return res;
	}

	public javax.vecmath.Vector3f convert() {
		javax.vecmath.Vector3f newVec = new javax.vecmath.Vector3f();
		convert(this, newVec);
		return newVec;
	}

	public static javax.vecmath.Vector3f convert(Vec3f oldVec, javax.vecmath.Vector3f newVec) {
		newVec.x = oldVec.X;
		newVec.y = oldVec.Y;
		newVec.z = oldVec.Z;
		return newVec;
	}

	public void set(javax.vecmath.Vector3f v){
		this.X = v.x;
		this.Y = v.y;
		this.Z = v.z;
	}
}