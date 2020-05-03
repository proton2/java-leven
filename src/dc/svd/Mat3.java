package dc.svd;

import core.math.Vec3f;

/**
 * Created by proton2 on 31.12.2019.
 */
class Mat3
{
    public float m00, m01, m02, m10, m11, m12, m20, m21, m22;

    public Mat3()
    {
        this.clear();
    }

    public Mat3(float m00, float m01, float m02,
                float m10, float m11, float m12,
                float m20, float m21, float m22)
    {
        this.set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
    }

    public void clear()
    {
        this.set(0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    public void set(float m00, float m01, float m02,
                    float m10, float m11, float m12,
                    float m20, float m21, float m22)
    {
        this.m00 = m00;
        this.m01 = m01;
        this.m02 = m02;
        this.m10 = m10;
        this.m11 = m11;
        this.m12 = m12;
        this.m20 = m20;
        this.m21 = m21;
        this.m22 = m22;
    }

    public float[][] convTo2dFloat(){
        float[][] res = {{ m00, m01, m02 }, { m10, m11, m12 }, { m20, m21, m22 }};
        return res;
    }

    public Mat3 identity(){
        Mat3 m = new Mat3();
        m.set( 1, 0, 0, 0, 1, 0, 0, 0, 1 );
        return m;
    }


    public void set(Mat3 rhs)
    {
        this.set(rhs.m00, rhs.m01, rhs.m02, rhs.m10, rhs.m11, rhs.m12, rhs.m20,
                rhs.m21, rhs.m22);
    }

    public void setSymmetric(float a00, float a01, float a02,
                             float a11, float a12, float a22)
    {
        this.set(a00, a01, a02, a01, a11, a12, a02, a12, a22);
    }

    public void setSymmetric(SMat3 rhs)
    {
        this.setSymmetric(rhs.m00, rhs.m01, rhs.m02, rhs.m11, rhs.m12, rhs.m22);
    }

    private Mat3(Mat3 rhs)
    {
        this.set(rhs);
    }

    public Vec3f Vmul(Vec3f v)
    {
        Vec3f o = new Vec3f();
        o.X = (m00 * v.X) + (m01 * v.Y) + (m02 * v.Z);
        o.Y = (m10 * v.X) + (m11 * v.Y) + (m12 * v.Z);
        o.Z = (m20 * v.X) + (m21 * v.Y) + (m22 * v.Z);
        return o;
    }

    public void rot01_post(float c, float s)
    {
        float m00 = this.m00, m01 = this.m01, m10 = this.m10, m11 = this.m11, m20 = this.m20, m21 = this.m21;
        set(    c * m00 - s * m01, s * m00 + c * m01, this.m02,
                c * m10 - s * m11, s * m10 + c * m11, this.m12,
                c * m20 - s * m21, s * m20 + c * m21, this.m22);
    }

    public void rot02_post(float c, float s) {
        float m00 = this.m00, m02 = this.m02, m10 = this.m10, m12 = this.m12, m20 = this.m20, m22 = this.m22;
        set(    c * m00 - s * m02, this.m01, s * m00 + c * m02,
                c * m10 - s * m12, this.m11, s * m10 + c * m12,
                c * m20 - s * m22, this.m21, s * m20 + c * m22);
    }

    public void rot12_post(float c, float s)
    {
        float m01 = this.m01, m02 = this.m02, m11 = this.m11, m12 = this.m12, m21 = this.m21, m22 = this.m22;
        set(    this.m00,c * m01 - s * m02, s * m01 + c * m02,
                this.m10,c * m11 - s * m12, s * m11 + c * m12,
                this.m20,c * m21 - s * m22, s * m21 + c * m22);
    }
}
