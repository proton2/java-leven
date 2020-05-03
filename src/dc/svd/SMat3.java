package dc.svd;

import core.math.Vec3f;
import org.joml.Math;

/**
 * Created by proton2 on 31.12.2019.
 */
public class SMat3
{
    public float m00, m01, m02, m11, m12, m22;

    public SMat3()
    {
        this.clear();
    }

    public void set(float m00, float m01, float m02, float m11, float m12, float m22){
        this.m00 = m00;
        this.m01 = m01;
        this.m02 = m02;
        this.m11 = m11;
        this.m12 = m12;
        this.m22 = m22;
    }

    public void set(SMat3 ivtav){
        this.m00 = ivtav.m00;
        this.m01 = ivtav.m01;
        this.m02 = ivtav.m02;
        this.m11 = ivtav.m11;
        this.m12 = ivtav.m12;
        this.m22 = ivtav.m22;
    }

    public SMat3 copy(){
        return new SMat3(this);
    }

    public float[][] convTo2dFloat(){
        float[][] vtav = new float[3][3];
        vtav[0][0] = m00; vtav[0][1] = m01;   vtav[0][2] = m02;
        vtav[1][0] = 0.f; vtav[1][1] = m11;   vtav[1][2] = m12;
        vtav[2][0] = 0.f; vtav[2][1] = 0.f;   vtav[2][2] = m22;
        return vtav;
    }

    public static SMat3 createFromFloat2D(float[][] a){
        SMat3 s = new SMat3();
        s.m00 = a[0][0];
        s.m01 = a[0][1];
        s.m02 = a[0][2];
        s.m11 = a[1][1];
        s.m12 = a[1][2];
        s.m22 = a[2][2];
        return s;
    }

    public SMat3(float m00, float m01, float m02,
                 float m11, float m12, float m22)
    {
        this.setSymmetric(m00, m01, m02, m11, m12, m22);
    }

    public void clear()
    {
        this.setSymmetric(0, 0, 0, 0, 0, 0);
    }

    public void setSymmetric(float a00, float a01, float a02,
                             float a11, float a12, float a22)
    {
        this.m00 = a00;
        this.m01 = a01;
        this.m02 = a02;
        this.m11 = a11;
        this.m12 = a12;
        this.m22 = a22;
    }

    public void setSymmetric(SMat3 rhs)
    {
        this.setSymmetric(rhs.m00, rhs.m01, rhs.m02, rhs.m11, rhs.m12, rhs.m22);
    }

    private SMat3(SMat3 rhs)
    {
        this.setSymmetric(rhs);
    }

    public Vec3f Vmul(Vec3f v)
    {
        Vec3f o = new Vec3f();
        o.X = (m00 * v.X) + (m01 * v.Y) + (m02 * v.Z);
        o.Y = (m01 * v.X) + (m11 * v.Y) + (m12 * v.Z);
        o.Z = (m02 * v.X) + (m12 * v.Y) + (m22 * v.Z);
        return o;
    }

    public float Off()
    {
        return (float)Math.sqrt(2 * ((m01 * m01) + (m02 * m02) + (m12 * m12)));
    }

    public float Fnorm()
    {
        return (float)Math.sqrt((m00 * m00) + (m01 * m01) + (m02 * m02)
                + (m01 * m01) + (m11 * m11) + (m12 * m12)
                + (m02 * m02) + (m12 * m12) + (m22 * m22));
    }

    public FloatHolder rot01()
    {
        FloatHolder f = calcSymmetricGivensCoefficients(m00, m01, m11);
        float cc = f.c * f.c;
        float ss = f.s * f.s;
        float mix = 2.0f * f.c * f.s * m01;
        setSymmetric(cc * m00 - mix + ss * m11, 0, f.c * m02 - f.s * m12,
                ss * m00 + mix + cc * m11, f.s * m02 + f.c * m12, m22);
        return f;
    }

    public FloatHolder rot02()
    {
        FloatHolder f = calcSymmetricGivensCoefficients(m00, m02, m22);
        float cc = f.c * f.c;
        float ss = f.s * f.s;
        float mix = 2.0f * f.c * f.s * m02;
        setSymmetric(cc * m00 - mix + ss * m22, f.c * m01 - f.s * m12, 0,
                m11, f.s * m01 + f.c * m12, ss * m00 + mix + cc * m22);
        return f;
    }

    public FloatHolder rot12()
    {
        FloatHolder f = calcSymmetricGivensCoefficients(m11, m12, m22);
        float cc = f.c * f.c;
        float ss = f.s * f.s;
        float mix = 2.0f * f.c * f.s * m12;
        setSymmetric(m00, f.c * m01 - f.s * m02, f.s * m01 + f.c * m02,
                cc * m11 - mix + ss * m22, 0, ss * m11 + mix + cc * m22);
        return f;
    }

    private static FloatHolder calcSymmetricGivensCoefficients(final float a_pp, final float a_pq, final float a_qq) {
        FloatHolder f = new FloatHolder();
        if (a_pq == 0) {
            f.c = 1;
            f.s = 0;
            return f;
        }
        float tau = (a_qq - a_pp) / (2 * a_pq);
        float stt = (float) Math.sqrt(1.0f + tau * tau);
        float tan = 1.0f / ((tau >= 0) ? (tau + stt) : (tau - stt));
        f.c = (float) (1.0f / Math.sqrt (1.0f + tan * tan));
        f.s = tan * f.c;
        return f;
    }
}
