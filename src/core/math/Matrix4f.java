package core.math;

import core.kernel.Window;

import java.nio.FloatBuffer;


public class Matrix4f {

	public float[][] m;
	
	public Matrix4f()
	{
		setM(new float[4][4]);
		Identity();
	}

	public Matrix4f(Matrix4f v) {
		this();
		m[0][0] = v.m[0][0];
		m[0][1] = v.m[0][1];
		m[0][2] = v.m[0][2];
		m[0][3] = v.m[0][3];
		m[1][0] = v.m[1][0];
		m[1][1] = v.m[1][1];
		m[1][2] = v.m[1][2];
		m[1][3] = v.m[1][3];
		m[2][0] = v.m[2][0];
		m[2][1] = v.m[2][1];
		m[2][2] = v.m[2][2];
		m[2][3] = v.m[2][3];
		m[3][0] = v.m[3][0];
		m[3][1] = v.m[3][1];
		m[3][2] = v.m[3][2];
		m[3][3] = v.m[3][3];
	}

	public Matrix4f(float m00, float m01, float m02, float m03, float m10, float m11, float m12, float m13,
					float m20, float m21, float m22, float m23, float m30, float m31, float m32, float m33) {
		this();
		this.m[0][0] = m00;
		this.m[0][1] = m01;
		this.m[0][2] = m02;
		this.m[0][3] = m03;
		this.m[1][0] = m10;
		this.m[1][1] = m11;
		this.m[1][2] = m12;
		this.m[1][3] = m13;
		this.m[2][0] = m20;
		this.m[2][1] = m21;
		this.m[2][2] = m22;
		this.m[2][3] = m23;
		this.m[3][0] = m30;
		this.m[3][1] = m31;
		this.m[3][2] = m32;
		this.m[3][3] = m33;
	}

	public Matrix4f Zero()
	{
		m[0][0] = 0; m[0][1] = 0; m[0][2] = 0; m[0][3] = 0;
		m[1][0] = 0; m[1][1] = 0; m[1][2] = 0; m[1][3] = 0;
		m[2][0] = 0; m[2][1] = 0; m[2][2] = 0; m[2][3] = 0;
		m[3][0] = 0; m[3][1] = 0; m[3][2] = 0; m[3][3] = 0;
	
		return this;
	}

	public float[] contTo1dFloat(){
		float[] mat = new float[16];
		mat[0] = m[0][0];
		mat[1] = m[0][1];
		mat[2] = m[0][2];
		mat[3] = m[0][3];
		mat[4] = m[1][0];
		mat[5] = m[1][1];
		mat[6] = m[1][2];
		mat[7] = m[1][3];
		mat[8] = m[2][0];
		mat[9] = m[2][1];
		mat[10] = m[2][2];
		mat[11] = m[2][3];
		mat[12] = m[3][0];
		mat[13] = m[3][1];
		mat[14] = m[3][2];
		mat[15] = m[3][3];
		return mat;
	}

	public FloatBuffer convToFloatBuff(){
		return FloatBuffer.wrap(contTo1dFloat());
	}
	
	public Matrix4f Identity()
	{
		m[0][0] = 1; m[0][1] = 0; m[0][2] = 0; m[0][3] = 0;
		m[1][0] = 0; m[1][1] = 1; m[1][2] = 0; m[1][3] = 0;
		m[2][0] = 0; m[2][1] = 0; m[2][2] = 1; m[2][3] = 0;
		m[3][0] = 0; m[3][1] = 0; m[3][2] = 0; m[3][3] = 1;
	
		return this;
	}
	
	public Matrix4f Orthographic2D(int width, int height)
	{
		m[0][0] = 2f/(float)width; 	m[0][1] = 0; 			    m[0][2] = 0; m[0][3] = -1;
		m[1][0] = 0;		 		m[1][1] = 2f/(float)height; m[1][2] = 0; m[1][3] = -1;
		m[2][0] = 0; 				m[2][1] = 0; 				m[2][2] = 1; m[2][3] =  0;
		m[3][0] = 0; 				m[3][1] = 0; 				m[3][2] = 0; m[3][3] =  1;
		
		return this;
	}
	
	public Matrix4f Orthographic2D()
	{
		//Z-Value 1: depth of orthographic OOB between 0 and -1
		
		m[0][0] = 2f/(float)Window.getInstance().getWidth();m[0][1] = 0; 								 								 m[0][2] = 0; m[0][3] = -1;
		m[1][0] = 0;		 														m[1][1] = 2f/(float)Window.getInstance().getHeight();m[1][2] = 0; m[1][3] = -1;
		m[2][0] = 0; 																m[2][1] = 0; 								 								 m[2][2] = 1; m[2][3] =  0;
		m[3][0] = 0; 																m[3][1] = 0; 								 								 m[3][2] = 0; m[3][3] =  1;
		
		return this;
	}
	
	public Matrix4f Translation(Vec3f translation) {
		m[0][0] = 1;
		m[0][1] = 0;
		m[0][2] = 0;
		m[0][3] = translation.getX();
		m[1][0] = 0;
		m[1][1] = 1;
		m[1][2] = 0;
		m[1][3] = translation.getY();
		m[2][0] = 0;
		m[2][1] = 0;
		m[2][2] = 1;
		m[2][3] = translation.getZ();
		m[3][0] = 0;
		m[3][1] = 0;
		m[3][2] = 0;
		m[3][3] = 1;
	
		return this;
	}
	
	public Matrix4f Rotation(Vec3f rotation)
	{
		Matrix4f rx = new Matrix4f();
		Matrix4f ry = new Matrix4f();
		Matrix4f rz = new Matrix4f();
		
		float x = (float)Math.toRadians(rotation.getX());
		float y = (float)Math.toRadians(rotation.getY());
		float z = (float)Math.toRadians(rotation.getZ());
		
		rz.m[0][0] = (float)Math.cos(z); rz.m[0][1] = -(float)Math.sin(z); 	 rz.m[0][2] = 0; 				   rz.m[0][3] = 0;
		rz.m[1][0] = (float)Math.sin(z); rz.m[1][1] = (float)Math.cos(z);  	 rz.m[1][2] = 0; 				   rz.m[1][3] = 0;
		rz.m[2][0] = 0; 				 rz.m[2][1] = 0; 				   	 rz.m[2][2] = 1; 				   rz.m[2][3] = 0;
		rz.m[3][0] = 0; 				 rz.m[3][1] = 0; 				   	 rz.m[3][2] = 0; 				   rz.m[3][3] = 1;
		
		rx.m[0][0] = 1; 				 rx.m[0][1] = 0;					 rx.m[0][2] = 0; 				   rx.m[0][3] = 0;
		rx.m[1][0] = 0; 				 rx.m[1][1] = (float)Math.cos(x); 	 rx.m[1][2] = -(float)Math.sin(x); rx.m[1][3] = 0;
		rx.m[2][0] = 0; 				 rx.m[2][1] = (float)Math.sin(x); 	 rx.m[2][2] = (float)Math.cos(x);  rx.m[2][3] = 0;
		rx.m[3][0] = 0; 				 rx.m[3][1] = 0; 				 	 rx.m[3][2] = 0;				   rx.m[3][3] = 1;
		
		ry.m[0][0] = (float)Math.cos(y); ry.m[0][1] = 0; 					 ry.m[0][2] = (float)Math.sin(y);  ry.m[0][3] = 0;
		ry.m[1][0] = 0; 				 ry.m[1][1] = 1; 				 	 ry.m[1][2] = 0; 				   ry.m[1][3] = 0;
		ry.m[2][0] = -(float)Math.sin(y);ry.m[2][1] = 0;					 ry.m[2][2] = (float)Math.cos(y);  ry.m[2][3] = 0;
		ry.m[3][0] = 0; 				 ry.m[3][1] = 0; 					 ry.m[3][2] = 0; 				   ry.m[3][3] = 1;
	
		m =  rz.mul(ry.mul(rx)).getM();
		
		return this;
	}
	
	public Matrix4f Scaling(Vec3f scaling)
	{
		m[0][0] = scaling.getX(); 	m[0][1] = 0; 				m[0][2] = 0; 				m[0][3] = 0;
		m[1][0] = 0; 			 	m[1][1] = scaling.getY();	m[1][2] = 0; 				m[1][3] = 0;
		m[2][0] = 0; 				m[2][1] = 0; 				m[2][2] = scaling.getZ(); 	m[2][3] = 0;
		m[3][0] = 0; 				m[3][1] = 0; 				m[3][2] = 0; 				m[3][3] = 1;
	
		return this;
	}
	
	public Matrix4f OrthographicProjection(float l, float r, float b, float t, float n, float f){
		
		m[0][0] = 2.0f/(r-l); 	m[0][1] = 0; 			m[0][2] = 0; 			m[0][3] = -(r+l)/(r-l);
		m[1][0] = 0;			m[1][1] = 2.0f/(t-b); 	m[1][2] = 0; 			m[1][3] = -(t+b)/(t-b);
		m[2][0] = 0; 			m[2][1] = 0; 			m[2][2] = 2.0f/(f-n); 	m[2][3] = -(f+n)/(f-n);
		m[3][0] = 0; 			m[3][1] = 0; 			m[3][2] = 0; 			m[3][3] = 1;
	
		return this;
	}

	public Matrix4f PerspectiveProjection(float fovY, float width, float height, float zNear, float zFar)
	{
		float tanFOV = (float) Math.tan(Math.toRadians(fovY/2));
		float aspectRatio = width/height;

		m[0][0] = 1/(tanFOV*aspectRatio); m[0][1] = 0; 		 	   m[0][2] = 0; 				m[0][3] = 0;
		m[1][0] = 0; 					  m[1][1] = 1/tanFOV; 	   m[1][2] = 0; 			 	m[1][3] = 0;
		m[2][0] = 0; 				 	  m[2][1] = 0; 		 	   m[2][2] = zFar/(zFar-zNear);	m[2][3] = zFar*zNear /(zFar-zNear);
		m[3][0] = 0; 				 	  m[3][1] = 0; 		 	   m[3][2] = 1; 				m[3][3] = 1;

		return this;
	}

//	public Matrix4f PerspectiveProjection(float fov, float width, float height, float near, float far)
//	{
//		float tanFOV = (float) Math.tan(Math.toRadians(fov / 2));
//		float aspect = width/height;
//		float range = far - near;
//
//		m[0][0] = 1.0f / (aspect * tanFOV); m[0][1] = 0; 			m[0][2] = 0; 							m[0][3] = 0;
//		m[1][0] = 0; 						m[1][1] = 1.0f / tanFOV;m[1][2] = 0; 							m[1][3] = 0;
//		m[2][0] = 0; 				 		m[2][1] = 0; 			m[2][2] = -((far + near) / range);		m[2][3] = -1.0f;
//		m[3][0] = 0; 				 		m[3][1] = 0; 			m[3][2] = -((2 * far * near) / range); 	m[3][3] = 0;
//
//		return this;
//	}
	
	public Matrix4f View(Vec3f forward, Vec3f up) {
		Vec3f right = up.cross(forward);
		
		m[0][0] = right.getX();
		m[0][1] = right.getY();
		m[0][2] = right.getZ();
		m[0][3] = 0;

		m[1][0] = up.getX();
		m[1][1] = up.getY();
		m[1][2] = up.getZ();
		m[1][3] = 0;

		m[2][0] = forward.getX();
		m[2][1] = forward.getY();
		m[2][2] = forward.getZ();
		m[2][3] = 0;

		m[3][0] = 0;
		m[3][1] = 0;
		m[3][2] = 0;
		m[3][3] = 1;
	
		return this;
	}

	public Matrix4f View1(Vec3f forward, Vec3f up) {
		Vec3f directional = forward.normalize();
		Vec3f right = forward.cross(up).normalize();
		Vec3f upVec = right.cross(directional).normalize();

		m[0][0] = -right.getX();
		m[0][1] = -right.getY();
		m[0][2] = -right.getZ();
		m[0][3] = 0;

		m[1][0] = upVec.getX();
		m[1][1] = upVec.getY();
		m[1][2] = upVec.getZ();
		m[1][3] = 0;

		m[2][0] = directional.getX();
		m[2][1] = directional.getY();
		m[2][2] = directional.getZ();
		m[2][3] = 0;

		m[3][0] = 0;
		m[3][1] = 0;
		m[3][2] = 0;
		m[3][3] = 1;

		return this;
	}
	
	public Matrix4f mul(Matrix4f r){
		
		Matrix4f res = new Matrix4f();
		
		for (int i=0; i<4; i++)
		{
			for (int j=0; j<4; j++)
			{
				res.set(i, j, m[i][0] * r.get(0, j) + 
							  m[i][1] * r.get(1, j) +
							  m[i][2] * r.get(2, j) +
							  m[i][3] * r.get(3, j));
			}
		}
		
		return res;
	}
	
	public Quaternion mul(Quaternion v)
	{
		Quaternion res = new Quaternion(0,0,0,0);
		
		res.setX(m[0][0] * v.getX() + m[0][1] * v.getY() + m[0][2] * v.getZ() + m[0][3] * v.getW());
		res.setY(m[1][0] * v.getX() + m[1][1] * v.getY() + m[1][2] * v.getZ() + m[1][3] * v.getW());
		res.setZ(m[2][0] * v.getX() + m[2][1] * v.getY() + m[2][2] * v.getZ() + m[2][3] * v.getW());
		res.setW(m[3][0] * v.getX() + m[3][1] * v.getY() + m[3][2] * v.getZ() + m[3][3] * v.getW());
		
		return res;
	}

	public Vec4f mul(Vec4f v)
	{
		Vec4f res = new Vec4f(0,0,0,0);

		res.setX(m[0][0] * v.getX() + m[0][1] * v.getY() + m[0][2] * v.getZ() + m[0][3] * v.getW());
		res.setY(m[1][0] * v.getX() + m[1][1] * v.getY() + m[1][2] * v.getZ() + m[1][3] * v.getW());
		res.setZ(m[2][0] * v.getX() + m[2][1] * v.getY() + m[2][2] * v.getZ() + m[2][3] * v.getW());
		res.setW(m[3][0] * v.getX() + m[3][1] * v.getY() + m[3][2] * v.getZ() + m[3][3] * v.getW());

		return res;
	}
	
	public Matrix4f transpose()
	{
		Matrix4f result = new Matrix4f();
		
		for(int i=0; i<4; i++){
			for(int j=0; j<4; j++){
				result.set(i, j, get(j,i));
			}
		}
		return result;
	}
	
	public Matrix4f invert()
	{
		float s0 = get(0, 0) * get(1, 1) - get(1, 0) * get(0, 1);
		float s1 = get(0, 0) * get(1, 2) - get(1, 0) * get(0, 2);
		float s2 = get(0, 0) * get(1, 3) - get(1, 0) * get(0, 3);
		float s3 = get(0, 1) * get(1, 2) - get(1, 1) * get(0, 2);
		float s4 = get(0, 1) * get(1, 3) - get(1, 1) * get(0, 3);
		float s5 = get(0, 2) * get(1, 3) - get(1, 2) * get(0, 3);

		float c5 = get(2, 2) * get(3, 3) - get(3, 2) * get(2, 3);
		float c4 = get(2, 1) * get(3, 3) - get(3, 1) * get(2, 3);
		float c3 = get(2, 1) * get(3, 2) - get(3, 1) * get(2, 2);
		float c2 = get(2, 0) * get(3, 3) - get(3, 0) * get(2, 3);
		float c1 = get(2, 0) * get(3, 2) - get(3, 0) * get(2, 2);
		float c0 = get(2, 0) * get(3, 1) - get(3, 0) * get(2, 1);
		
		
		float div = (s0 * c5 - s1 * c4 + s2 * c3 + s3 * c2 - s4 * c1 + s5 * c0);
		if (div == 0) System.err.println("not invertible");
		
	    float invdet = 1.0f / div;
	    
	    Matrix4f invM = new Matrix4f();
	    
	    invM.set(0, 0, (get(1, 1) * c5 - get(1, 2) * c4 + get(1, 3) * c3) * invdet);
	    invM.set(0, 1, (-get(0, 1) * c5 + get(0, 2) * c4 - get(0, 3) * c3) * invdet);
	    invM.set(0, 2, (get(3, 1) * s5 - get(3, 2) * s4 + get(3, 3) * s3) * invdet);
	    invM.set(0, 3, (-get(2, 1) * s5 + get(2, 2) * s4 - get(2, 3) * s3) * invdet);

	    invM.set(1, 0, (-get(1, 0) * c5 + get(1, 2) * c2 - get(1, 3) * c1) * invdet);
	    invM.set(1, 1, (get(0, 0) * c5 - get(0, 2) * c2 + get(0, 3) * c1) * invdet);
	    invM.set(1, 2, (-get(3, 0) * s5 + get(3, 2) * s2 - get(3, 3) * s1) * invdet);
	    invM.set(1, 3, (get(2, 0) * s5 - get(2, 2) * s2 + get(2, 3) * s1) * invdet);

	    invM.set(2, 0, (get(1, 0) * c4 - get(1, 1) * c2 + get(1, 3) * c0) * invdet);
	    invM.set(2, 1, (-get(0, 0) * c4 + get(0, 1) * c2 - get(0, 3) * c0) * invdet);
	    invM.set(2, 2, (get(3, 0) * s4 - get(3, 1) * s2 + get(3, 3) * s0) * invdet);
	    invM.set(2, 3, (-get(2, 0) * s4 + get(2, 1) * s2 - get(2, 3) * s0) * invdet);

	    invM.set(3, 0, (-get(1, 0) * c3 + get(1, 1) * c1 - get(1, 2) * c0) * invdet);
	    invM.set(3, 1, (get(0, 0) * c3 - get(0, 1) * c1 + get(0, 2) * c0) * invdet);
	    invM.set(3, 2, (-get(3, 0) * s3 + get(3, 1) * s1 - get(3, 2) * s0) * invdet);
	    invM.set(3, 3, (get(2, 0) * s3 - get(2, 1) * s1 + get(2, 2) * s0) * invdet);
		
		return invM;
	}
	
	public boolean equals(Matrix4f m){
		return this.m[0][0] == m.getM()[0][0] && this.m[0][1] == m.getM()[0][1] &&
				this.m[0][2] == m.getM()[0][2] && this.m[0][3] == m.getM()[0][3] &&
				this.m[1][0] == m.getM()[1][0] && this.m[1][1] == m.getM()[1][1] &&
				this.m[1][2] == m.getM()[1][2] && this.m[1][3] == m.getM()[1][3] &&
				this.m[2][0] == m.getM()[2][0] && this.m[2][1] == m.getM()[2][1] &&
				this.m[2][2] == m.getM()[2][2] && this.m[2][3] == m.getM()[2][3] &&
				this.m[3][0] == m.getM()[3][0] && this.m[3][1] == m.getM()[3][1] &&
				this.m[3][2] == m.getM()[3][2] && this.m[3][3] == m.getM()[3][3];
	}
	
	public void set(int x, int y, float value)
	{
		this.m[x][y] = value;
	}

	public void set (Matrix4f that){
		this.setM(that.getM());
	}
	
	public float get(int x, int y)
	{
		return  this.m[x][y];
	}

	public float [][] getM() {
		return m;
	}

	public void setM(float [][] m) {
		this.m = m;
	}
	
	public String toString() {
		
		return 	"|" + m[0][0] + " " + m[0][1] + " " + m[0][2] + " " + m[0][3] + "|\n" +
				"|" + m[1][0] + " " + m[1][1] + " " + m[1][2] + " " + m[1][3] + "|\n" +
				"|" + m[2][0] + " " + m[2][1] + " " + m[2][2] + " " + m[2][3] + "|\n" +
				"|" + m[3][0] + " " + m[3][1] + " " + m[3][2] + " " + m[3][3] + "|";
	}

	public static Matrix4f rotate(Matrix4f m, float angle, Vec3f v) {
		//float a = (float)Math.toRadians(angle);
		float a = angle;
		float c = (float)Math.cos(a);
		float s = (float)Math.sin(a);

		Vec3f axis = v.normalize();
		Vec3f temp = axis.mul(1.0f - c);

		Matrix4f Rotate = new Matrix4f();
		Rotate.m[0][0] = c + temp.X * axis.X;
		Rotate.m[0][1] = 0 + temp.X * axis.Y + s * axis.Z;
		Rotate.m[0][2] = 0 + temp.X * axis.Z - s * axis.Y;

		Rotate.m[1][0] = 0 + temp.Y * axis.X - s * axis.Z;
		Rotate.m[1][1] = c + temp.Y * axis.Y;
		Rotate.m[1][2] = 0 + temp.Y * axis.Z + s * axis.X;

		Rotate.m[2][0] = 0 + temp.Z * axis.X + s * axis.Y;
		Rotate.m[2][1] = 0 + temp.Z * axis.Y - s * axis.X;
		Rotate.m[2][2] = c + temp.Z * axis.Z;

		Matrix4f Result = new Matrix4f();
		Result.m[0][0] = m.m[0][0] * Rotate.m[0][0] + m.m[1][0] * Rotate.m[0][1] + m.m[2][0] * Rotate.m[0][2];
		Result.m[0][1] = m.m[0][1] * Rotate.m[0][0] + m.m[1][1] * Rotate.m[0][1] + m.m[2][1] * Rotate.m[0][2];
		Result.m[0][2] = m.m[0][2] * Rotate.m[0][0] + m.m[1][2] * Rotate.m[0][1] + m.m[2][2] * Rotate.m[0][2];
		Result.m[0][3] = m.m[0][3] * Rotate.m[0][0] + m.m[1][3] * Rotate.m[0][1] + m.m[2][3] * Rotate.m[0][2];

		Result.m[1][0] = m.m[0][0] * Rotate.m[1][0] + m.m[1][0] * Rotate.m[1][1] + m.m[2][0] * Rotate.m[1][2];
		Result.m[1][1] = m.m[0][1] * Rotate.m[1][0] + m.m[1][1] * Rotate.m[1][1] + m.m[2][1] * Rotate.m[1][2];
		Result.m[1][2] = m.m[0][2] * Rotate.m[1][0] + m.m[1][2] * Rotate.m[1][1] + m.m[2][2] * Rotate.m[1][2];
		Result.m[1][3] = m.m[0][3] * Rotate.m[1][0] + m.m[1][3] * Rotate.m[1][1] + m.m[2][3] * Rotate.m[1][2];

		Result.m[2][0] = m.m[0][0] * Rotate.m[2][0] + m.m[1][0] * Rotate.m[2][1] + m.m[2][0] * Rotate.m[2][2];
		Result.m[2][1] = m.m[0][1] * Rotate.m[2][0] + m.m[1][1] * Rotate.m[2][1] + m.m[2][1] * Rotate.m[2][2];
		Result.m[2][2] = m.m[0][2] * Rotate.m[2][0] + m.m[1][2] * Rotate.m[2][1] + m.m[2][2] * Rotate.m[2][2];
		Result.m[2][3] = m.m[0][3] * Rotate.m[2][0] + m.m[1][3] * Rotate.m[2][1] + m.m[2][3] * Rotate.m[2][2];

		Result.m[3][0] = m.m[3][0];
		Result.m[3][1] = m.m[3][1];
		Result.m[3][2] = m.m[3][2];
		Result.m[3][3] = m.m[3][3];
		return Result;
	}

	public Matrix4f rotate(float angle, float x, float y, float z, Matrix4f res) {
		float s = (float) Math.sin(angle);
		float c = (float) Math.cos(angle);
		float C = 1.0f - c;
		// rotation matrix elements: m30, m31, m32, m03, m13, m23 = 0, m33 = 1
		float xx = x * x, xy = x * y, xz = x * z;
		float yy = y * y, yz = y * z;
		float zz = z * z;
		float rm00 = xx * C + c;
		float rm01 = xy * C + z * s;
		float rm02 = xz * C - y * s;
		float rm10 = xy * C - z * s;
		float rm11 = yy * C + c;
		float rm12 = yz * C + x * s;
		float rm20 = xz * C + y * s;
		float rm21 = yz * C - x * s;
		float rm22 = zz * C + c;
		// add temporaries for dependent values
		float nm00 = m[0][0] * rm00 + m[1][0] * rm01 + m[2][0] * rm02;
		float nm01 = m[0][1] * rm00 + m[1][1] * rm01 + m[2][1] * rm02;
		float nm02 = m[0][2] * rm00 + m[1][2] * rm01 + m[2][2] * rm02;
		float nm03 = m[0][3] * rm00 + m[1][3] * rm01 + m[2][3] * rm02;
		float nm10 = m[0][0] * rm10 + m[1][0] * rm11 + m[2][0] * rm12;
		float nm11 = m[0][1] * rm10 + m[1][1] * rm11 + m[2][1] * rm12;
		float nm12 = m[0][2] * rm10 + m[1][2] * rm11 + m[2][2] * rm12;
		float nm13 = m[0][3] * rm10 + m[1][3] * rm11 + m[2][3] * rm12;
		// set non-dependent values directly
		res.m[2][0] = m[0][0] * rm20 + m[1][0] * rm21 + m[2][0] * rm22;
		res.m[2][1] = m[0][1] * rm20 + m[1][1] * rm21 + m[2][1] * rm22;
		res.m[2][2] = m[0][2] * rm20 + m[1][2] * rm21 + m[2][2] * rm22;
		res.m[2][3] = m[0][3] * rm20 + m[1][3] * rm21 + m[2][3] * rm22;
		// set other values
		res.m[0][0] = nm00;
		res.m[0][1] = nm01;
		res.m[0][2] = nm02;
		res.m[0][3] = nm03;
		res.m[1][0] = nm10;
		res.m[1][1] = nm11;
		res.m[1][2] = nm12;
		res.m[1][3] = nm13;
		res.m[3][0] = m[3][0];
		res.m[3][1] = m[3][1];
		res.m[3][2] = m[3][2];
		res.m[3][3] = m[3][3];
		return res;
	}

	public static Matrix4f glmLookAt(Vec3f eye, Vec3f center, Vec3f upVec) {
		Vec3f forward = center.sub(eye).normalize();
		Vec3f up = upVec.normalize();
		Vec3f right = cross(forward, up).normalize();
		up = cross(right, forward);

		Matrix4f Result = new Matrix4f().Identity();
		Result.m[0][0] = right.X;
		Result.m[1][0] = right.Y;
		Result.m[2][0] = right.Z;
		Result.m[0][1] = up.X;
		Result.m[1][1] = up.Y;
		Result.m[2][1] = up.Z;
		Result.m[0][2] =-forward.X;
		Result.m[1][2] =-forward.Y;
		Result.m[2][2] =-forward.Z;

		return translate(Result, eye.mul(-1));
		//return Result.Translation(eye.mul(-1));
	}

	private static Vec3f cross(Vec3f x, Vec3f y){
		return new Vec3f (x.Y * y.Z - y.Y * x.Z, x.Z * y.X - y.Z * x.X, x.X * y.Y - y.X * x.Y);
	}

	private static Matrix4f translate(Matrix4f m, Vec3f v) {
		Matrix4f Result = new Matrix4f(m);
		//Result[3] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2] + m[3];
		Result.m[3][0] = m.m[0][0] * v.X + m.m[1][0] * v.Y + m.m[2][0] * v.Z + m.m[3][0];
		Result.m[3][1] = m.m[0][1] * v.X + m.m[1][1] * v.Y + m.m[2][1] * v.Z + m.m[3][1];
		Result.m[3][2] = m.m[0][2] * v.X + m.m[1][2] * v.Y + m.m[2][2] * v.Z + m.m[3][2];
		Result.m[3][3] = m.m[0][3] * v.X + m.m[1][3] * v.Y + m.m[2][3] * v.Z + m.m[3][3];
		return Result;
	}

	public static Matrix4f lookAtRH(Vec3f eye, Vec3f center, Vec3f up) {
		// f(normalize(center - eye))
		float fX = center.X - eye.X;
		float fY = center.Y - eye.Y;
		float fZ = center.Z - eye.Z;
		float inverseSqrt = 1f / (float) Math.sqrt(fX * fX + fY * fY + fZ * fZ);
		fX *= inverseSqrt;
		fY *= inverseSqrt;
		fZ *= inverseSqrt;
		// s(normalize(cross(f, up)))
		float sX = fY * up.Z - fZ * up.Y;
		float sY = fZ * up.X - fX * up.Z;
		float sZ = fX * up.Y - fY * up.X;
		inverseSqrt = 1.0f / (float) Math.sqrt(sX * sX + sY * sY + sZ * sZ);
		sX *= inverseSqrt;
		sY *= inverseSqrt;
		sZ *= inverseSqrt;
		// u(cross(s, f))
		float uX = sY * fZ - sZ * fY;
		float uY = sZ * fX - sX * fZ;
		float uZ = sX * fY - sY * fX;

		Matrix4f res = new Matrix4f().Identity();
		res.m[0][0] = sX;
		res.m[0][1] = uX;
		res.m[0][2] = -fX;
		res.m[0][3] = 0f;
		res.m[1][0] = sY;
		res.m[1][1] = uY;
		res.m[1][2] = -fY;
		res.m[1][3] = 0f;
		res.m[2][0] = sZ;
		res.m[2][1] = uZ;
		res.m[2][2] = -fZ;
		res.m[2][3] = 0f;
		res.m[3][0] = -sX * eye.X - sY * eye.Y - sZ * eye.Z;
		res.m[3][1] = -uX * eye.X - uY * eye.Y - uZ * eye.Z;
		res.m[3][2] = +fX * eye.X + fY * eye.Y + fZ * eye.Z;
		res.m[3][3] = 1f;
		return res;
	}

	public static void main(String[] args) {
		float angle = 1f;
//        float x = 2f;
//        float y = 3f;
//        float z = 4f;
		float x = 0.371390671f;
		float y = 0.557085991f;
		float z = 0.742781341f;
		Matrix4f dest = new Matrix4f().Identity();
		Matrix4f instance = new Matrix4f().Identity();
		Matrix4f expResult = new Matrix4f(0.603708863f, 0.720138788f, -0.341958523f, 0f,
				-0.529919028f, 0.682967067f, 0.502734184f, 0f,
				0.595584869f, -0.122294709f, 0.793928623f, 0f,
				0f, 0f, 0f, 1f);
		Matrix4f result = instance.rotate(angle, x, y, z, dest);
//		if(!result.equals(expResult)){
//			throw new RuntimeException("test failed");
//		}

		Matrix4f res2 = new Matrix4f().Identity().rotate(new Matrix4f().Identity(), angle, new Vec3f(x, y, z));
		if(!res2.equals(result)){
			throw new RuntimeException("test failed");
		}
	}
}
