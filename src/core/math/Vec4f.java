package core.math;


public class Vec4f {

	public float x;
	public float y;
	public float z;
	public float w;

	public Vec4f(){
	}

	public Vec4f(float x, float y, float z, float w){
		this.setX(x);
		this.setY(y);
		this.setZ(z);
		this.setW(w);
	}

	public Vec4f(float x, float y, float z){
		this.setX(x);
		this.setY(y);
		this.setZ(z);
		this.setW(w);
	}

	public Vec4f(Vec3i v){
		this.setX(v.x);
		this.setY(v.y);
		this.setZ(v.z);
	}

	public Vec4f(Vec3f v){
		this.setX(v.X);
		this.setY(v.Y);
		this.setZ(v.Z);
	}

	public Vec4f(Vec4f f) {
		this.setX(f.x);
		this.setY(f.y);
		this.setZ(f.z);
		this.setW(f.w);
	}

	public Vec3f getVec3f(){
		return new Vec3f(this.x, this.y, this.z);
	}

	public Vec3i getVec3i(){
		return new Vec3i(this.x, this.y, this.z);
	}
	
	public Vec4f(Vec3f v, float w){
		this.setX(v.getX());
		this.setY(v.getY());
		this.setZ(v.getZ());
		this.setW(w);
	}
	
	public float length()
	{
		return (float) Math.sqrt(x*x + y*y + z*z + w*w);
	}
	
	public Vec4f normalize()
	{
		float length = length();
		
		x /= length;
		y /= length;
		z /= length;
		w /= length;
		
		return this;
	}
	
	public Vec4f conjugate()
	{
		return new Vec4f (-x, -y, -z, w);
	}
	
	public Vec4f mul(Vec4f r)
	{
		float w_ = w * r.getW() - x * r.getX() - y * r.getY() - z * r.getZ();
		float x_ = x * r.getW() + w * r.getX() + y * r.getZ() - z * r.getY();
		float y_ = y * r.getW() + w * r.getY() + z * r.getX() - x * r.getZ();
		float z_ = z * r.getW() + w * r.getZ() + x * r.getY() - y * r.getX();

		return new Vec4f(x_, y_, z_, w_);
	}

	public Vec4f mul(Vec3f r)
	{
		float w_ = -x * r.getX() - y * r.getY() - z * r.getZ();
		float x_ =  w * r.getX() + y * r.getZ() - z * r.getY();
		float y_ =  w * r.getY() + z * r.getX() - x * r.getZ();
		float z_ =  w * r.getZ() + x * r.getY() - y * r.getX();

		return new Vec4f(x_, y_, z_, w_);
	}
	
	public Vec4f div(float r)
	{
		float w_ = w/r;
		float x_ = x/r;
		float y_ = y/r;
		float z_ = z/r;
		return new Vec4f(x_, y_, z_, w_);
	}
	
	public Vec4f mul(float r)
	{
		float w_ = w*r;
		float x_ = x*r;
		float y_ = y*r;
		float z_ = z*r;
		return new Vec4f(x_, y_, z_, w_);
	}

	public Vec3f mul3f(float r) {
		float x_ = x*r;
		float y_ = y*r;
		float z_ = z*r;
		return new Vec3f(x_, y_, z_);
	}
	
	public Vec4f sub(Vec4f r)
	{
		float w_ = w - r.getW();
		float x_ = x - r.getX();
		float y_ = y - r.getY();
		float z_ = z - r.getZ();
		return new Vec4f(x_, y_, z_, w_);
	}

	public Vec3f sub(Vec3f r) {
		float x_ = x - r.getX();
		float y_ = y - r.getY();
		float z_ = z - r.getZ();
		return new Vec3f(x_, y_, z_);
	}
	
	public Vec4f add(Vec4f r)
	{
		float w_ = w + r.getW();
		float x_ = x + r.getX();
		float y_ = y + r.getY();
		float z_ = z + r.getZ();
		return new Vec4f(x_, y_, z_, w_);
	}

	public Vec4f add(Vec3f r, float w) {
		return new Vec4f(this.x += r.getX(), this.y += r.getY(), this.z += r.getZ(), this.w += w);
	}

	public Vec4f add(Vec3i r)
	{
		float x_ = x + r.x;
		float y_ = y + r.y;
		float z_ = z + r.z;
		return new Vec4f(x_, y_, z_, w);
	}

	public float dot(Vec4f r)
	{
		return x * r.getX() + y * r.getY() + z * r.getZ() + w * r.getW();
	}

	public Vec4f vmul(float[] mat3x3_tri_ATA) {
		Vec4f o = new Vec4f();
		o.x = (mat3x3_tri_ATA[0] * this.x) + (mat3x3_tri_ATA[1] * this.y) + (mat3x3_tri_ATA[2] * this.z);
		o.y = (mat3x3_tri_ATA[1] * this.x) + (mat3x3_tri_ATA[3] * this.y) + (mat3x3_tri_ATA[4] * this.z);
		o.z = (mat3x3_tri_ATA[2] * this.x) + (mat3x3_tri_ATA[4] * this.y) + (mat3x3_tri_ATA[5] * this.z);
		o.w = 0;
		return o;
	}
	
	public Vec3f xyz(){
		return new Vec3f(x,y,z);
	}
	
	public String toString()
	{
		return "[" + this.x + "," + this.y + "," + this.z + "," + this.w + "]";
	}

	public float getX() {
		return x;
	}

	public void setX(float x) {
		this.x = x;
	}

	public void set(Vec4f that){
		this.setX(that.x);
		this.setY(that.y);
		this.setZ(that.z);
		this.setW(that.w);
	}

	public float getY() {
		return y;
	}

	public void setY(float y) {
		this.y = y;
	}

	public float getZ() {
		return z;
	}

	public void setZ(float z) {
		this.z = z;
	}

	public float getW() {
		return w;
	}

	public void setW(float w) {
		this.w = w;
	}

	public void set(float x, float y, float z, float v) {
		this.setX(x);
		this.setY(y);
		this.setZ(z);
		this.setW(w);
	}

	public Vec4f mul(Matrix4f m) {
		Vec4f res = new Vec4f(0,0,0,0);
		res.setX(m.get(0,0) * getX() + m.get(0,1) * getY() + m.get(0,2) * getZ() + m.get(0,3) * getW());
		res.setY(m.get(1,0) * getX() + m.get(1,1) * getY() + m.get(1,2) * getZ() + m.get(1,3) * getW());
		res.setZ(m.get(2,0) * getX() + m.get(2,1) * getY() + m.get(2,2) * getZ() + m.get(2,3) * getW());
		res.setW(m.get(3,0) * getX() + m.get(3,1) * getY() + m.get(3,2) * getZ() + m.get(3,3) * getW());
		return res;
	}
}
