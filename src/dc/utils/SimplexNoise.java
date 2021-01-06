package dc.utils;


/*
 * A speed-improved simplex noise algorithm for 2D, 3D and 4D in Java.
 *
 * Based on example code by Stefan Gustavson (stegu@itn.liu.se).
 * Optimisations by Peter Eastman (peastman@drizzle.stanford.edu).
 * Better rank ordering method by Stefan Gustavson in 2012.
 *
 * This could be speeded up even further, but it's useful as it is.
 *
 * Version 2012-03-09
 *
 * This code was placed in the public domain by its original author,
 * Stefan Gustavson. You may use it as you see fit, but
 * attribution is appreciated.
 *
 */

import core.math.Vec2f;
import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import core.utils.ImageLoader;
import org.joml.Vector3f;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

public class SimplexNoise { // Simplex noise in 2D, 3D and 4D
	static final float RIDGED_MULTI_H = 1.f;
	static final float NOISE_SCALE =  1.f;

	private static SimplexNoise instance = null;
	private static float[] densityField;
	public static int worldSizeXZ;
	public static SimplexNoise getInstance(String filename, int worldSizeXZ) {
		if(instance == null) {
			instance = new SimplexNoise(filename, worldSizeXZ);
		}
		return instance;
	}

	private SimplexNoise(String filename, int worldSizeXZ){
		densityField = prepareAndStoreDensity(filename, worldSizeXZ);
		SimplexNoise.worldSizeXZ = worldSizeXZ;
	}

	private float[] prepareAndStoreDensity(String filename, int rootSize){
		float[] densityField = new float[rootSize * rootSize];
		Path path = Paths.get(filename);
		if (Files.exists(path)) {
			ImageLoader.loadImageToFloatArray(densityField, filename);
		}
		if (Files.notExists(path)) {
			for (int z = 0; z < rootSize; z++) {
				for (int x = 0; x < rootSize; x++) {
					densityField[x + z * rootSize] = Terrain(new Vec2f((float)(x - (rootSize / 2)), (float)(z - (rootSize / 2))));
				}
			}
			ImageLoader.saveImageToFloat(densityField, filename);
		}
		return densityField;
	}

	public static float getNoise(Vec4f pos) {
		return getNoise(new Vec3i(pos.x, pos.y, pos.z));
	}

	public static float getNoise(Vec3f pos) {
		return getNoise(new Vec3i(pos.X, pos.Y, pos.Z));
	}

	public static float getNoise(Vec3i pos) {
		float MAX_TERRAIN_HEIGHT = 900.f;
		int x = (pos.x + worldSizeXZ /2) & worldSizeXZ -1;
		int z = (pos.z + worldSizeXZ /2) & worldSizeXZ -1;
		float height = densityField[z + x * worldSizeXZ];
		return pos.y - (MAX_TERRAIN_HEIGHT * height) + 800;
	}

	public static float Sample(Vec3f pos) {
		return pos.getY() - Noise(pos) * 8.0f - 8;
		//return Cuboid(pos);
		//return Sphere(pos);
	}

	private static float Noise(Vec3f pos) {
		float r = 0.05f;
		return (float) noise(pos.getX() * r, pos.getY() * r, pos.getZ() * r);
	}

	public static float Cuboid(Vec3f pos) {
		float radius = (float)64 / 8.0f;
		Vec3f local = pos.sub(new Vec3f(64 / 2, 64 / 2, 64 / 2));
		Vec3f d = new Vec3f(Math.abs(local.X), Math.abs(local.Y), Math.abs(local.Z)).sub(new Vec3f(radius, radius, radius));
		float m = Math.max(d.X, Math.max(d.Y, d.Z));
		Vec3f max = d;
		return Math.min(m, max.length());
	}

	public static float Sphere(Vec3f pos) {
		float radius = (float)64 / 2.0f - 2.0f;
		Vec3f origin = new Vec3f((64 - 2.0f) * 0.5f);
		return (pos.sub(origin)).lengthSquared() - radius * radius;
	}

	private static Grad grad3[] = { new Grad(1, 1, 0), new Grad(-1, 1, 0),
			new Grad(1, -1, 0), new Grad(-1, -1, 0), new Grad(1, 0, 1),
			new Grad(-1, 0, 1), new Grad(1, 0, -1), new Grad(-1, 0, -1),
			new Grad(0, 1, 1), new Grad(0, -1, 1), new Grad(0, 1, -1),
			new Grad(0, -1, -1) };

	private static Grad grad4[] = { new Grad(0, 1, 1, 1),
			new Grad(0, 1, 1, -1), new Grad(0, 1, -1, 1),
			new Grad(0, 1, -1, -1), new Grad(0, -1, 1, 1),
			new Grad(0, -1, 1, -1), new Grad(0, -1, -1, 1),
			new Grad(0, -1, -1, -1), new Grad(1, 0, 1, 1),
			new Grad(1, 0, 1, -1), new Grad(1, 0, -1, 1),
			new Grad(1, 0, -1, -1), new Grad(-1, 0, 1, 1),
			new Grad(-1, 0, 1, -1), new Grad(-1, 0, -1, 1),
			new Grad(-1, 0, -1, -1), new Grad(1, 1, 0, 1),
			new Grad(1, 1, 0, -1), new Grad(1, -1, 0, 1),
			new Grad(1, -1, 0, -1), new Grad(-1, 1, 0, 1),
			new Grad(-1, 1, 0, -1), new Grad(-1, -1, 0, 1),
			new Grad(-1, -1, 0, -1), new Grad(1, 1, 1, 0),
			new Grad(1, 1, -1, 0), new Grad(1, -1, 1, 0),
			new Grad(1, -1, -1, 0), new Grad(-1, 1, 1, 0),
			new Grad(-1, 1, -1, 0), new Grad(-1, -1, 1, 0),
			new Grad(-1, -1, -1, 0) };

	private static short p[] = { 151, 160, 137, 91, 90, 15, 131, 13, 201, 95,
			96, 53, 194, 233, 7, 225, 140, 36, 103, 30, 69, 142, 8, 99, 37,
			240, 21, 10, 23, 190, 6, 148, 247, 120, 234, 75, 0, 26, 197, 62,
			94, 252, 219, 203, 117, 35, 11, 32, 57, 177, 33, 88, 237, 149, 56,
			87, 174, 20, 125, 136, 171, 168, 68, 175, 74, 165, 71, 134, 139,
			48, 27, 166, 77, 146, 158, 231, 83, 111, 229, 122, 60, 211, 133,
			230, 220, 105, 92, 41, 55, 46, 245, 40, 244, 102, 143, 54, 65, 25,
			63, 161, 1, 216, 80, 73, 209, 76, 132, 187, 208, 89, 18, 169, 200,
			196, 135, 130, 116, 188, 159, 86, 164, 100, 109, 198, 173, 186, 3,
			64, 52, 217, 226, 250, 124, 123, 5, 202, 38, 147, 118, 126, 255,
			82, 85, 212, 207, 206, 59, 227, 47, 16, 58, 17, 182, 189, 28, 42,
			223, 183, 170, 213, 119, 248, 152, 2, 44, 154, 163, 70, 221, 153,
			101, 155, 167, 43, 172, 9, 129, 22, 39, 253, 19, 98, 108, 110, 79,
			113, 224, 232, 178, 185, 112, 104, 218, 246, 97, 228, 251, 34, 242,
			193, 238, 210, 144, 12, 191, 179, 162, 241, 81, 51, 145, 235, 249,
			14, 239, 107, 49, 192, 214, 31, 181, 199, 106, 157, 184, 84, 204,
			176, 115, 121, 50, 45, 127, 4, 150, 254, 138, 236, 205, 93, 222,
			114, 67, 29, 24, 72, 243, 141, 128, 195, 78, 66, 215, 61, 156, 180 };
	// To remove the need for index wrapping, double the permutation table
	// length
	private static short perm[] = new short[512];
	private static short permMod12[] = new short[512];
	static {
		for (int i = 0; i < 512; i++) {
			perm[i] = p[i & 255];
			permMod12[i] = (short) (perm[i] % 12);
		}
	}

	// Skewing and unskewing factors for 2, 3, and 4 dimensions
	private static final double F2 = 0.5 * (Math.sqrt(3.0) - 1.0);
	private static final double G2 = (3.0 - Math.sqrt(3.0)) / 6.0;
	private static final double F3 = 1.0 / 3.0;
	private static final double G3 = 1.0 / 6.0;
	private static final double F4 = (Math.sqrt(5.0) - 1.0) / 4.0;
	private static final double G4 = (5.0 - Math.sqrt(5.0)) / 20.0;

	// This method is a *lot* faster than using (int)Math.floor(x)
	private static int fastfloor(double x) {
		int xi = (int) x;
		return x < xi ? xi - 1 : xi;
	}

	private static double dot(Grad g, double x, double y) {
		return g.x * x + g.y * y;
	}

	private static double dot(Grad g, double x, double y, double z) {
		return g.x * x + g.y * y + g.z * z;
	}

	private static double dot(Grad g, double x, double y, double z, double w) {
		return g.x * x + g.y * y + g.z * z + g.w * w;
	}

	// 2D simplex noise
	private static double noise(double xin, double yin) {
		double n0, n1, n2; // Noise contributions from the three corners
		// Skew the input space to determine which simplex cell we're in
		double s = (xin + yin) * F2; // Hairy factor for 2D
		int i = fastfloor(xin + s);
		int j = fastfloor(yin + s);
		double t = (i + j) * G2;
		double X0 = i - t; // Unskew the cell origin back to (x,y) space
		double Y0 = j - t;
		double x0 = xin - X0; // The x,y distances from the cell origin
		double y0 = yin - Y0;
		// For the 2D case, the simplex shape is an equilateral triangle.
		// Determine which simplex we are in.
		int i1, j1; // Offsets for second (middle) corner of simplex in (i,j)
					// coords
		if (x0 > y0) {
			i1 = 1;
			j1 = 0;
		} // lower triangle, XY order: (0,0)->(1,0)->(1,1)
		else {
			i1 = 0;
			j1 = 1;
		} // upper triangle, YX order: (0,0)->(0,1)->(1,1)
		// A step of (1,0) in (i,j) means a step of (1-c,-c) in (x,y), and
		// a step of (0,1) in (i,j) means a step of (-c,1-c) in (x,y), where
		// c = (3-sqrt(3))/6
		double x1 = x0 - i1 + G2; // Offsets for middle corner in (x,y) unskewed
									// coords
		double y1 = y0 - j1 + G2;
		double x2 = x0 - 1.0 + 2.0 * G2; // Offsets for last corner in (x,y)
											// unskewed coords
		double y2 = y0 - 1.0 + 2.0 * G2;
		// Work out the hashed gradient indices of the three simplex corners
		int ii = i & 255;
		int jj = j & 255;
		int gi0 = permMod12[ii + perm[jj]];
		int gi1 = permMod12[ii + i1 + perm[jj + j1]];
		int gi2 = permMod12[ii + 1 + perm[jj + 1]];
		// Calculate the contribution from the three corners
		double t0 = 0.5 - x0 * x0 - y0 * y0;
		if (t0 < 0)
			n0 = 0.0;
		else {
			t0 *= t0;
			n0 = t0 * t0 * dot(grad3[gi0], x0, y0); // (x,y) of grad3 used for
													// 2D gradient
		}
		double t1 = 0.5 - x1 * x1 - y1 * y1;
		if (t1 < 0)
			n1 = 0.0;
		else {
			t1 *= t1;
			n1 = t1 * t1 * dot(grad3[gi1], x1, y1);
		}
		double t2 = 0.5 - x2 * x2 - y2 * y2;
		if (t2 < 0)
			n2 = 0.0;
		else {
			t2 *= t2;
			n2 = t2 * t2 * dot(grad3[gi2], x2, y2);
		}
		// Add contributions from each corner to get the final noise value.
		// The result is scaled to return values in the interval [-1,1].
		return 70.0 * (n0 + n1 + n2);
	}

	// 3D simplex noise
	public static double noise(double xin, double yin, double zin) {
		double n0, n1, n2, n3; // Noise contributions from the four corners
		// Skew the input space to determine which simplex cell we're in
		double s = (xin + yin + zin) * F3; // Very nice and simple skew factor
											// for 3D
		int i = fastfloor(xin + s);
		int j = fastfloor(yin + s);
		int k = fastfloor(zin + s);
		double t = (i + j + k) * G3;
		double X0 = i - t; // Unskew the cell origin back to (x,y,z) space
		double Y0 = j - t;
		double Z0 = k - t;
		double x0 = xin - X0; // The x,y,z distances from the cell origin
		double y0 = yin - Y0;
		double z0 = zin - Z0;
		// For the 3D case, the simplex shape is a slightly irregular
		// tetrahedron.
		// Determine which simplex we are in.
		int i1, j1, k1; // Offsets for second corner of simplex in (i,j,k)
						// coords
		int i2, j2, k2; // Offsets for third corner of simplex in (i,j,k) coords
		if (x0 >= y0) {
			if (y0 >= z0) {
				i1 = 1;
				j1 = 0;
				k1 = 0;
				i2 = 1;
				j2 = 1;
				k2 = 0;
			} // X Y Z order
			else if (x0 >= z0) {
				i1 = 1;
				j1 = 0;
				k1 = 0;
				i2 = 1;
				j2 = 0;
				k2 = 1;
			} // X Z Y order
			else {
				i1 = 0;
				j1 = 0;
				k1 = 1;
				i2 = 1;
				j2 = 0;
				k2 = 1;
			} // Z X Y order
		} else { // x0<y0
			if (y0 < z0) {
				i1 = 0;
				j1 = 0;
				k1 = 1;
				i2 = 0;
				j2 = 1;
				k2 = 1;
			} // Z Y X order
			else if (x0 < z0) {
				i1 = 0;
				j1 = 1;
				k1 = 0;
				i2 = 0;
				j2 = 1;
				k2 = 1;
			} // Y Z X order
			else {
				i1 = 0;
				j1 = 1;
				k1 = 0;
				i2 = 1;
				j2 = 1;
				k2 = 0;
			} // Y X Z order
		}
		// A step of (1,0,0) in (i,j,k) means a step of (1-c,-c,-c) in (x,y,z),
		// a step of (0,1,0) in (i,j,k) means a step of (-c,1-c,-c) in (x,y,z),
		// and
		// a step of (0,0,1) in (i,j,k) means a step of (-c,-c,1-c) in (x,y,z),
		// where
		// c = 1/6.
		double x1 = x0 - i1 + G3; // Offsets for second corner in (x,y,z) coords
		double y1 = y0 - j1 + G3;
		double z1 = z0 - k1 + G3;
		double x2 = x0 - i2 + 2.0 * G3; // Offsets for third corner in (x,y,z)
										// coords
		double y2 = y0 - j2 + 2.0 * G3;
		double z2 = z0 - k2 + 2.0 * G3;
		double x3 = x0 - 1.0 + 3.0 * G3; // Offsets for last corner in (x,y,z)
											// coords
		double y3 = y0 - 1.0 + 3.0 * G3;
		double z3 = z0 - 1.0 + 3.0 * G3;
		// Work out the hashed gradient indices of the four simplex corners
		int ii = i & 255;
		int jj = j & 255;
		int kk = k & 255;
		int gi0 = permMod12[ii + perm[jj + perm[kk]]];
		int gi1 = permMod12[ii + i1 + perm[jj + j1 + perm[kk + k1]]];
		int gi2 = permMod12[ii + i2 + perm[jj + j2 + perm[kk + k2]]];
		int gi3 = permMod12[ii + 1 + perm[jj + 1 + perm[kk + 1]]];
		// Calculate the contribution from the four corners
		double t0 = 0.6 - x0 * x0 - y0 * y0 - z0 * z0;
		if (t0 < 0)
			n0 = 0.0;
		else {
			t0 *= t0;
			n0 = t0 * t0 * dot(grad3[gi0], x0, y0, z0);
		}
		double t1 = 0.6 - x1 * x1 - y1 * y1 - z1 * z1;
		if (t1 < 0)
			n1 = 0.0;
		else {
			t1 *= t1;
			n1 = t1 * t1 * dot(grad3[gi1], x1, y1, z1);
		}
		double t2 = 0.6 - x2 * x2 - y2 * y2 - z2 * z2;
		if (t2 < 0)
			n2 = 0.0;
		else {
			t2 *= t2;
			n2 = t2 * t2 * dot(grad3[gi2], x2, y2, z2);
		}
		double t3 = 0.6 - x3 * x3 - y3 * y3 - z3 * z3;
		if (t3 < 0)
			n3 = 0.0;
		else {
			t3 *= t3;
			n3 = t3 * t3 * dot(grad3[gi3], x3, y3, z3);
		}
		// Add contributions from each corner to get the final noise value.
		// The result is scaled to stay just inside [-1,1]
		return 32.0 * (n0 + n1 + n2 + n3);
	}

	// 4D simplex noise, better simplex rank ordering method 2012-03-09
	public static double noise(double x, double y, double z, double w) {

		double n0, n1, n2, n3, n4; // Noise contributions from the five corners
		// Skew the (x,y,z,w) space to determine which cell of 24 simplices
		// we're in
		double s = (x + y + z + w) * F4; // Factor for 4D skewing
		int i = fastfloor(x + s);
		int j = fastfloor(y + s);
		int k = fastfloor(z + s);
		int l = fastfloor(w + s);
		double t = (i + j + k + l) * G4; // Factor for 4D unskewing
		double X0 = i - t; // Unskew the cell origin back to (x,y,z,w) space
		double Y0 = j - t;
		double Z0 = k - t;
		double W0 = l - t;
		double x0 = x - X0; // The x,y,z,w distances from the cell origin
		double y0 = y - Y0;
		double z0 = z - Z0;
		double w0 = w - W0;
		// For the 4D case, the simplex is a 4D shape I won't even try to
		// describe.
		// To find out which of the 24 possible simplices we're in, we need to
		// determine the magnitude ordering of x0, y0, z0 and w0.
		// Six pair-wise comparisons are performed between each possible pair
		// of the four coordinates, and the results are used to rank the
		// numbers.
		int rankx = 0;
		int ranky = 0;
		int rankz = 0;
		int rankw = 0;
		if (x0 > y0)
			rankx++;
		else
			ranky++;
		if (x0 > z0)
			rankx++;
		else
			rankz++;
		if (x0 > w0)
			rankx++;
		else
			rankw++;
		if (y0 > z0)
			ranky++;
		else
			rankz++;
		if (y0 > w0)
			ranky++;
		else
			rankw++;
		if (z0 > w0)
			rankz++;
		else
			rankw++;
		int i1, j1, k1, l1; // The integer offsets for the second simplex corner
		int i2, j2, k2, l2; // The integer offsets for the third simplex corner
		int i3, j3, k3, l3; // The integer offsets for the fourth simplex corner
		// simplex[c] is a 4-vector with the numbers 0, 1, 2 and 3 in some
		// order.
		// Many values of c will never occur, since e.g. x>y>z>w makes x<z, y<w
		// and x<w
		// impossible. Only the 24 indices which have non-zero entries make any
		// sense.
		// We use a thresholding to set the coordinates in turn from the largest
		// magnitude.
		// Rank 3 denotes the largest coordinate.
		i1 = rankx >= 3 ? 1 : 0;
		j1 = ranky >= 3 ? 1 : 0;
		k1 = rankz >= 3 ? 1 : 0;
		l1 = rankw >= 3 ? 1 : 0;
		// Rank 2 denotes the second largest coordinate.
		i2 = rankx >= 2 ? 1 : 0;
		j2 = ranky >= 2 ? 1 : 0;
		k2 = rankz >= 2 ? 1 : 0;
		l2 = rankw >= 2 ? 1 : 0;
		// Rank 1 denotes the second smallest coordinate.
		i3 = rankx >= 1 ? 1 : 0;
		j3 = ranky >= 1 ? 1 : 0;
		k3 = rankz >= 1 ? 1 : 0;
		l3 = rankw >= 1 ? 1 : 0;
		// The fifth corner has all coordinate offsets = 1, so no need to
		// compute that.
		double x1 = x0 - i1 + G4; // Offsets for second corner in (x,y,z,w)
									// coords
		double y1 = y0 - j1 + G4;
		double z1 = z0 - k1 + G4;
		double w1 = w0 - l1 + G4;
		double x2 = x0 - i2 + 2.0 * G4; // Offsets for third corner in (x,y,z,w)
										// coords
		double y2 = y0 - j2 + 2.0 * G4;
		double z2 = z0 - k2 + 2.0 * G4;
		double w2 = w0 - l2 + 2.0 * G4;
		double x3 = x0 - i3 + 3.0 * G4; // Offsets for fourth corner in
										// (x,y,z,w) coords
		double y3 = y0 - j3 + 3.0 * G4;
		double z3 = z0 - k3 + 3.0 * G4;
		double w3 = w0 - l3 + 3.0 * G4;
		double x4 = x0 - 1.0 + 4.0 * G4; // Offsets for last corner in (x,y,z,w)
											// coords
		double y4 = y0 - 1.0 + 4.0 * G4;
		double z4 = z0 - 1.0 + 4.0 * G4;
		double w4 = w0 - 1.0 + 4.0 * G4;
		// Work out the hashed gradient indices of the five simplex corners
		int ii = i & 255;
		int jj = j & 255;
		int kk = k & 255;
		int ll = l & 255;
		int gi0 = perm[ii + perm[jj + perm[kk + perm[ll]]]] % 32;
		int gi1 = perm[ii + i1 + perm[jj + j1 + perm[kk + k1 + perm[ll + l1]]]] % 32;
		int gi2 = perm[ii + i2 + perm[jj + j2 + perm[kk + k2 + perm[ll + l2]]]] % 32;
		int gi3 = perm[ii + i3 + perm[jj + j3 + perm[kk + k3 + perm[ll + l3]]]] % 32;
		int gi4 = perm[ii + 1 + perm[jj + 1 + perm[kk + 1 + perm[ll + 1]]]] % 32;
		// Calculate the contribution from the five corners
		double t0 = 0.6 - x0 * x0 - y0 * y0 - z0 * z0 - w0 * w0;
		if (t0 < 0)
			n0 = 0.0;
		else {
			t0 *= t0;
			n0 = t0 * t0 * dot(grad4[gi0], x0, y0, z0, w0);
		}
		double t1 = 0.6 - x1 * x1 - y1 * y1 - z1 * z1 - w1 * w1;
		if (t1 < 0)
			n1 = 0.0;
		else {
			t1 *= t1;
			n1 = t1 * t1 * dot(grad4[gi1], x1, y1, z1, w1);
		}
		double t2 = 0.6 - x2 * x2 - y2 * y2 - z2 * z2 - w2 * w2;
		if (t2 < 0)
			n2 = 0.0;
		else {
			t2 *= t2;
			n2 = t2 * t2 * dot(grad4[gi2], x2, y2, z2, w2);
		}
		double t3 = 0.6 - x3 * x3 - y3 * y3 - z3 * z3 - w3 * w3;
		if (t3 < 0)
			n3 = 0.0;
		else {
			t3 *= t3;
			n3 = t3 * t3 * dot(grad4[gi3], x3, y3, z3, w3);
		}
		double t4 = 0.6 - x4 * x4 - y4 * y4 - z4 * z4 - w4 * w4;
		if (t4 < 0)
			n4 = 0.0;
		else {
			t4 *= t4;
			n4 = t4 * t4 * dot(grad4[gi4], x4, y4, z4, w4);
		}
		// Sum up and scale the result to cover the range [-1,1]
		return 27.0 * (n0 + n1 + n2 + n3 + n4);
	}

	// Inner class to speed upp gradient computations
	// (array access is a lot slower than member access)
	private static class Grad {
		double x, y, z, w;

		Grad(double x, double y, double z) {
			this.x = x;
			this.y = y;
			this.z = z;
		}

		Grad(double x, double y, double z, double w) {
			this.x = x;
			this.y = y;
			this.z = z;
			this.w = w;
		}
	}

	public static float Terrain(Vec2f position) {
		Vec2f p = position.mul(1.f / 2000.f);

		float ridged = 0.8f * RidgedMultiFractal(7, 2.114352f, /*gain=*/1.5241f, /*offset=*/1.f, p);
		ridged = VoxelHelperUtils.clamp(ridged, 0.f, 1.f);

		float billow = 0.6f * BasicFractal(4, 0.24f, 1.8754f, 0.433f, p.mul(new Vec2f(-4.33f, 7.98f)));
		billow = (0.5f * billow) + 0.5f;

		float noise = billow * ridged;

		float b2 = 0.6f * BasicFractal(2, 0.63f, 2.2f, 0.15f, p);
		b2 = (b2 * 0.5f) + 0.5f;
		noise += b2;

		//	return 0.1;
		return noise;
	}

	public static float BasicFractal(int octaves, float frequency, float lacunarity, float persistence, Vec2f position) {
		float SCALE = 1.0f / 128.0f;
		//Vec2f p = position.mul(NOISE_SCALE);
		Vec2f p = position.mul(SCALE);
		float noise = 0.f;

		float amplitude = 1.f;
		p = p.mul(frequency);

		for (int i = 0; i < octaves; i++) {
			noise += noise(p.X, p.Y) * amplitude;
			p = p.mul(lacunarity);
			amplitude *= persistence;
		}

		// move into (0, 1) range
		//return noise;
  		return 0.5f + (0.5f * noise);
	}

	private static float RidgedMultiFractal(int octaves, float lacunarity, float gain, float offset, Vec2f position) {
		Vec2f p = position.mul(NOISE_SCALE);

		float signal = (float) noise(p.X, p.Y);
		signal = Math.abs(signal);
		signal = offset - signal;
		signal *= signal;

		float noise = signal;
		float weight = 1.f;
		float frequency = 1.f;

		for (int i = 0; i < octaves; i++) {
			p = p.mul(lacunarity);

			weight = signal * gain;
			weight = VoxelHelperUtils.clamp(weight, 0.f, 1.f);

			signal = (float) noise(p.X, p.Y);
			signal = Math.abs(signal);
			signal = offset - signal;
			signal *= weight;

  		 	float exponent = (float) Math.pow(frequency, -1.f * RIDGED_MULTI_H);
			frequency *= lacunarity;

			noise += signal * exponent;
		}

		noise *= (1.f / octaves);
		return noise;
	}

	/////////from opencl noise translated to java//////////////////////////////////////////////////////////////
	static char perm2d[] = {151,160,137,91,90,15,
			131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
			190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
			88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
			77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
			102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
			135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
			5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
			223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
			129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
			251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
			49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
			138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180,
			151,160,137,91,90,15,
			131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
			190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
			88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
			77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
			102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
			135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
			5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
			223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
			129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
			251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
			49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
			138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
	};

	static float grad( int hash, float x, float y ) {
		int h = hash & 7;      // Convert low 3 bits of hash code
		float u = h<4 ? x : y;  // into 8 simple gradient directions,
		float v = h<4 ? y : x;  // and compute the dot product with (x,y).
		return ((h&1)==1 ? -u : u) + ((h&2)==1? -2.0f*v : 2.0f*v);
	}

	static float noise2dFromOpencl(float x, float y)
	{
		float F2 = 0.366025403f; // F2 = 0.5*(sqrt(3.0)-1.0)
		float G2 = 0.211324865f; // G2 = (3.0-Math.sqrt(3.0))/6.0

		float n0, n1, n2; // Noise contributions from the three corners

		// Skew the input space to determine which simplex cell we're in
		float s = (x+y)*F2; // Hairy factor for 2D
		float xs = x + s;
		float ys = y + s;
		int i = fastfloor(xs);
		int j = fastfloor(ys);

		float t = (float)(i+j)*G2;
		float X0 = i-t; // Unskew the cell origin back to (x,y) space
		float Y0 = j-t;
		float x0 = x-X0; // The x,y distances from the cell origin
		float y0 = y-Y0;

		// For the 2D case, the simplex shape is an equilateral triangle.
		// Determine which simplex we are in.
		int i1, j1; // Offsets for second (middle) corner of simplex in (i,j) coords
		if(x0>y0) {i1=1; j1=0;} // lower triangle, XY order: (0,0)->(1,0)->(1,1)
		else {i1=0; j1=1;}      // upper triangle, YX order: (0,0)->(0,1)->(1,1)

		// A step of (1,0) in (i,j) means a step of (1-c,-c) in (x,y), and
		// a step of (0,1) in (i,j) means a step of (-c,1-c) in (x,y), where
		// c = (3-sqrt(3))/6

		float x1 = x0 - i1 + G2; // Offsets for middle corner in (x,y) unskewed coords
		float y1 = y0 - j1 + G2;
		float x2 = x0 - 1.0f + 2.0f * G2; // Offsets for last corner in (x,y) unskewed coords
		float y2 = y0 - 1.0f + 2.0f * G2;

		// Wrap the integer indices at 256, to avoid indexing perm2d[] out of bounds
		int ii = i & 0xff;
		int jj = j & 0xff;

		// Calculate the contribution from the three corners
		float t0 = 0.5f - x0*x0-y0*y0;
		if(t0 < 0.0f) n0 = 0.0f;
		else {
			t0 *= t0;
			n0 = t0 * t0 * grad(perm2d[ii+perm2d[jj]], x0, y0);
		}

		float t1 = 0.5f - x1*x1-y1*y1;
		if(t1 < 0.0f) n1 = 0.0f;
		else {
			t1 *= t1;
			n1 = t1 * t1 * grad(perm2d[ii+i1+perm2d[jj+j1]], x1, y1);
		}

		float t2 = 0.5f - x2*x2-y2*y2;
		if(t2 < 0.0f) n2 = 0.0f;
		else {
			t2 *= t2;
			n2 = t2 * t2 * grad(perm2d[ii+1+perm2d[jj+1]], x2, y2);
		}

		// Add contributions from each corner to get the final noise value.
		// The result is scaled to return values in the interval [-1,1].
		return 40.0f * (n0 + n1 + n2); // TODO: The scale factor is preliminary!
	}
	/////////from opencl noise translated to java//////////////////////////////////////////////////////////////
}
