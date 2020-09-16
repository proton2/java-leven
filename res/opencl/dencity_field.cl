constant int FIELD_DIM = 66;

int field_index(const int4 pos)
{
	return pos.x + (pos.y * FIELD_DIM) + (pos.z * FIELD_DIM * FIELD_DIM);
}

kernel void GenerateDefaultField(
	const int4 offset,
	const int sampleScale,
	global int* field_materials)
{
	const int x = get_global_id(0);
	const int y = get_global_id(1);
	const int z = get_global_id(2);

	const float4 world_pos = {
	    (x * sampleScale) + offset.x,
    	(y * sampleScale) + offset.y,
    	(z * sampleScale) + offset.z,
    	0
    };

    const float MAX_HEIGHT = 20.0f;
    float noise = Noise_2d(world_pos.x, world_pos.z);
    const float density = world_pos.y - noise * MAX_HEIGHT - 40;

	const int4 local_pos = { x, y, z, 0 };
	const int index = field_index(local_pos);

	const int material = density < 0.f ?  1 : 0;
	field_materials[index] = material;
}