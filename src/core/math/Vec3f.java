package core.math;

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
}