#define ONE 0.00390625f
#define ONEHALF 0.001953125f
#define RIDGED_MULTI_H (1.f)

constant int MAX_TERRAIN_HEIGHT = 900.f;
__constant sampler_t permSampler = CLK_FILTER_NEAREST | CLK_ADDRESS_REPEAT | CLK_NORMALIZED_COORDS_TRUE;
constant int FIELD_DIM = 66;

int field_index(const int4 pos)
{
	return pos.x + (pos.y * FIELD_DIM) + (pos.z * FIELD_DIM * FIELD_DIM);
}

//kernel void GenerateDefaultFieldSimple(
//	const int4 offset,
//	const int sampleScale,
//	global int* field_materials)
//{
//	const int x = get_global_id(0);
//	const int y = get_global_id(1);
//	const int z = get_global_id(2);
//
//	const float4 world_pos = {
//	    (x * sampleScale) + offset.x,
//    	(y * sampleScale) + offset.y,
//    	(z * sampleScale) + offset.z,
//    	0
//    };
//
//    const float MAX_HEIGHT = 20.0f;
//    float noise = Noise_2d(world_pos.x, world_pos.z);
//    const float density = world_pos.y - noise * MAX_HEIGHT - 40;
//
//	const int4 local_pos = { x, y, z, 0 };
//	const int index = field_index(local_pos);
//
//	const int material = density < 0.f ?  1 : 0;
//	field_materials[index] = material;
//}



#define NOISE_SCALE (1.f)



float snoise2(const float2 P, read_only image2d_t permTexture)
{
// Skew and unskew factors are a bit hairy for 2D, so define them as constants
// This is (sqrt(3.f)-1.f)/2.f
#define F2 0.366025403784f
// This is (3.f-sqrt(3.f))/6.f
#define G2 0.211324865405f

	// Skew the (x,y) space to determine which cell of 2 simplices we're in
	float s = (P.x + P.y) * F2;	 // Hairy factor for 2D skewing
	float2 Pi = floor(P + s);
	float t = (Pi.x + Pi.y) * G2; // Hairy factor for unskewing
	float2 P0 = Pi - t; // Unskew the cell origin back to (x,y) space
	Pi = Pi * ONE + ONEHALF; // Integer part, scaled and offset for texture lookup

	float2 Pf0 = P - P0;	// The x,y distances from the cell origin

	// For the 2D case, the simplex shape is an equilateral triangle.
	// Find out whether we are above or below the x=y diagonal to
	// determine which of the two triangles we're in.
	float2 o1;
	if(Pf0.x > Pf0.y) o1 = (float2)(1.f, 0.f);	// +x, +y traversal order
	else o1 = (float2)(0.f, 1.f);					// +y, +x traversal order

	// Noise contribution from simplex origin
	float2 grad0 = read_imagef(permTexture, permSampler, Pi).xy * 4.f - 1.f;
	float t0 = 0.5 - dot(Pf0, Pf0);
	float n0;
	if (t0 < 0.f) n0 = 0.f;
	else {
		t0 *= t0;
		n0 = t0 * t0 * dot(grad0, Pf0);
	}

	// Noise contribution from middle corner
	float2 Pf1 = Pf0 - o1 + G2;
	float2 grad1 = read_imagef(permTexture, permSampler, Pi + o1*ONE).xy * 4.f - 1.f;
	float t1 = 0.5 - dot(Pf1, Pf1);
	float n1;
	if (t1 < 0.f) n1 = 0.f;
	else {
		t1 *= t1;
		n1 = t1 * t1 * dot(grad1, Pf1);
	}

	// Noise contribution from last corner
	float2 Pf2 = Pf0 - (float2)(1.f-2.f*G2);
	float2 grad2 = read_imagef(permTexture, permSampler, Pi + (float2)(ONE, ONE)).xy * 4.f - 1.f;
	float t2 = 0.5 - dot(Pf2, Pf2);
	float n2;
	if(t2 < 0.f) n2 = 0.f;
	else {
		t2 *= t2;
		n2 = t2 * t2 * dot(grad2, Pf2);
	}

	// Sum up and scale the result to cover the range [-1,1]
	return 70.f * (n0 + n1 + n2);
}

float BasicFractal(
	read_only image2d_t permTexture,
	const int octaves,
	const float frequency,
	const float lacunarity,
	const float persistence,
	float2 position)
{
	float2 p = position * NOISE_SCALE;
	float noise = 0.f;

	float amplitude = 1.f;
	p *= frequency;

	for (int i = 0; i < octaves; i++)
	{
		noise += snoise2(p, permTexture) * amplitude;
		p *= lacunarity;
		amplitude *= persistence;
	}

	// move into (0, 1) range
#if 1
	return noise;
#else
	const float remapped = 0.5f + (0.5f * noise);
	return remapped;
#endif
}

float RidgedMultiFractal(
	read_only image2d_t permTexture,
	const int octaves,
	const float lacunarity,
	const float gain,
	const float offset,
	float2 position)
{
	float2 p = position * NOISE_SCALE;

	float signal = snoise2(p, permTexture);
	signal = fabs(signal);
	signal = offset - signal;
	signal *= signal;

	float noise = signal;
	float weight = 1.f;
	float frequency = 1.f;

	for (int i = 0; i < octaves; i++)
	{
		p *= lacunarity;

		weight = signal * gain;
		weight = clamp(weight, 0.f, 1.f);

		signal = snoise2(p, permTexture);
		signal = fabs(signal);
		signal = offset - signal;
		signal *= weight;

		const float exponent = pow(frequency, -1.f * RIDGED_MULTI_H);
		frequency *= lacunarity;

		noise += signal * exponent;
	}

	noise *= (1.f / octaves);
	return noise;
}

float Terrain(const float4 position, read_only image2d_t permTexture)
{
	float2 p = position.xz * (1.f / 2000.f);

	float ridged = 0.8f * RidgedMultiFractal(permTexture, 7, 2.114352f, /*gain=*/1.5241f, /*offset=*/1.f, p.xy);
	ridged = clamp(ridged, 0.f, 1.f);

	float billow = 0.6f * BasicFractal(permTexture, 4, 0.24f, 1.8754f, 0.433f, (float2)(-4.33f, 7.98f) * p.xy);
	billow = (0.5f * billow) + 0.5f;

	float noise = billow * ridged;

	float b2 = 0.6f * BasicFractal(permTexture, 2, 0.63f, 2.2f, 0.15f, p.xy);
	b2 = (b2 * 0.5f) + 0.5f;
	noise += b2;

//	return 0.1;
	return noise;
}

float DensityFunc(const float4 position, read_only image2d_t permTexture){
	float noise = Terrain(position, permTexture);
	return position.y - (MAX_TERRAIN_HEIGHT * noise);
}

kernel void GenerateDefaultField(
	read_only image2d_t permTexture,
	const int4 offset,
	const int sampleScale,
	global int* field_materials)
{
	const int x = get_global_id(0);
	const int y = get_global_id(1);
	const int z = get_global_id(2);

	const float4 world_pos =
	{
		(x * sampleScale) + offset.x,
		(y * sampleScale) + offset.y,
		(z * sampleScale) + offset.z,
		0
	};

	const float density = DensityFunc(world_pos, permTexture);

	const int4 local_pos = { x, y, z, 0 };
	const int index = field_index(local_pos);
	const int material = density < 0.f ?  1 : 0;

	field_materials[index] = material;
}