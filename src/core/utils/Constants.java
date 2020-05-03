package core.utils;

import core.math.Vec3f;

public class Constants {

	public static final long NANOSECOND = 1000000000;
	public static final float ZFAR = 10000.0f;
	public static final float ZNEAR = 0.1f;
	
	public static final String RENDERER_COMPONENT = "Renderer";

	public static Vec3f Yellow = new Vec3f(0.8f, 0.8f, 0.f);
	public static Vec3f Red = new Vec3f(0.7f, 0.f, 0.f);
	public static Vec3f Blue = new Vec3f(0.f, 0.f, 1.f);
	public static Vec3f Green = new Vec3f(0.f, 1.f, 0.f);
	public static Vec3f White = new Vec3f(1.f, 1.f, 1.f);
}
