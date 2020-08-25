package dc.impl;

import core.math.Vec2f;
import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import dc.utils.Density;
import modules.CalculateMaterialComputeShader;
import modules.ComputeBuffer;
import modules.DensityPrimitive;
import org.joml.Vector3f;

import java.util.Random;

import static dc.VoxelOctree.MATERIAL_AIR;
import static dc.VoxelOctree.MATERIAL_SOLID;
import static dc.impl.LevenLinearOctreeImpl.fieldSize;

public class CalculateMaterialService {
    CalculateMaterialComputeShader shader;
    private ComputeBuffer fieldMaterialsComputeBuffer, permComputeBuffer, densityPrimitiveComputeBuffer;

    static int[] permutations = new int[512];
    private DensityPrimitive[] primitiveMods;

    public CalculateMaterialService(){}

    public void calculate(Vec3i offset, int sampleScale, int[] fieldMaterials) {
        InitPermutations(1200, permutations);
        primitiveMods = new DensityPrimitive[]{
                new DensityPrimitive(new Vec4f(3.1f, 4.2f, 5.7f, 1), new Vec4f(6.4f, 7.1f, 9.6f, 1)),
                new DensityPrimitive(new Vec4f(5.8f, 3.7f, 8.1f, 1), new Vec4f(5.9f, 2.6f, 1.8f, 1))
        };

        densityPrimitiveComputeBuffer = new ComputeBuffer();
        densityPrimitiveComputeBuffer.setData(primitiveMods);

        permComputeBuffer = new ComputeBuffer();
        permComputeBuffer.setData(permutations);

        fieldMaterialsComputeBuffer = new ComputeBuffer();
        fieldMaterialsComputeBuffer.setData(fieldMaterials);

        shader = CalculateMaterialComputeShader.getInstance();
        shader.bind();


        permComputeBuffer.bindBufferBase(0);
        densityPrimitiveComputeBuffer.bindBufferBase(1);
        fieldMaterialsComputeBuffer.bindBufferBase(2);

        shader.updateSimpleScaleUniforms(sampleScale);
        shader.updateOffcetUniforms(offset.toVec3f());
        shader.dispatch(7,7,6);

        permComputeBuffer.getData(permutations);
        densityPrimitiveComputeBuffer.getData(primitiveMods);
        fieldMaterialsComputeBuffer.getData(fieldMaterials);

        //calculateTest(offset, sampleScale, fieldMaterials);
    }

    protected int field_index(Vec3i pos) {
        return pos.x + (pos.y * fieldSize) + (pos.z * fieldSize * fieldSize);
    }

    void calculateTest(Vec3i offset, int sampleScale, int[] fieldMaterials) {
        for (int z = 0; z < fieldSize; z++) {
            for (int y = 0; y < fieldSize; y++) {
                for (int x = 0; x < fieldSize; x++) {
                    Vec3i local_pos = new Vec3i(x, y, z);
                    Vec3f world_pos = local_pos.mul(sampleScale).add(offset).toVec3f();
                    float noise = Density_Func(new Vec2f(world_pos.X, world_pos.Z));
                    float MAX_HEIGHT = 20.0f;
                    float density = world_pos.Y - noise * MAX_HEIGHT - 40;
                    int material = density < 0.f ? MATERIAL_SOLID : MATERIAL_AIR;
                    int index = field_index(local_pos);
                    fieldMaterials[index] = material;
                }
            }
        }
    }

    public static void InitPermutations(int seed, int[] permutations) {
        Random random = new Random(seed);
        for (int i = 0; i < 256; i++)
            permutations[i] = (int) (256 * (random.nextInt(10000) / 10000.0f));

        for (int i = 256; i < 512; i++)
            permutations[i] = permutations[i - 256];
    }

    Vec3i Grad3[] = {
            new Vec3i(1,1,0), new Vec3i(-1,1,0), new Vec3i(1,-1,0), new Vec3i(-1,-1,0),
            new Vec3i(1,0,1), new Vec3i(-1,0,1), new Vec3i(1,0,-1), new Vec3i(-1,0,-1),
            new Vec3i(0,1,1), new Vec3i(0,-1,1), new Vec3i(0,1,-1), new Vec3i(0,-1,-1)
    };

    float Vec3Dot(Vec3i a, Vec3i b) {
        float res = (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
        return res;
    }

    float lerp(float a, float b, float t) {
        return (1 - t) * a + t * b;
    }

    float Perlin(float x, float y)
    {
        int i = x > 0 ? (int)x : (int)x - 1;
        int j = y > 0 ? (int)y : (int)y - 1;

        x = x - i;
        y = y - j;

        i = i & 255;
        j = j & 255;

        int gll = permutations[i + permutations[j]] % 12;
        int glh = permutations[i + permutations[j + 1]] % 12;
        int ghl = permutations[i + 1 + permutations[j]] % 12;
        int ghh = permutations[i + 1 + permutations[j + 1]] % 12;

        float nll = (Grad3[gll].x * x) + (Grad3[gll].y * y);
        float nlh = (Grad3[glh].x * x) + (Grad3[glh].y * (y - 1));
        float nhl = (Grad3[ghl].x * (x - 1)) + (Grad3[ghl].y * y);
        float nhh = (Grad3[ghh].x * (x - 1)) + (Grad3[ghh].y * (y - 1));

        float u = x * x * x * (x * (x * 6 - 15) + 10);
        float v = y * y * y * (y * (y * 6 - 15) + 10);

//float nyl = Mathf.Lerp(nll, nhl, u);
        float nyl = (1-u)*nll + u*nhl;
//float nyh = Mathf.Lerp(nlh, nhh, u);
        float nyh = (1-u)*nlh + u*nhh;

//float nxy = Mathf.Lerp(nyl, nyh, v);
        float nxy = (1-v)*nyl + v*nyh;

        return nxy;
    }

    float FractalNoise(int octaves, float frequency, float lacunarity, float persistence, Vec2f position)
    {
        float SCALE = 1.0f / 128.0f;
        Vec2f p = position.mul(SCALE);
        float nois = 0.0f;

        float amplitude = 1.0f;
        p = p.mul(frequency);

        for (int i = 0; i < octaves; i++)
        {
            nois += Perlin(p.X, p.Y) * amplitude;
            p = p.mul(lacunarity);
            amplitude *= persistence;
        }

        return 0.5f + (0.5f * nois);
    }

    float CalculateNoiseValue(Vec3f pos) {
        return FractalNoise(4, 0.5343f, 2.2324f, 0.68324f, new Vec2f(pos.X, pos.Z));
    }

    float CLerp(float a, float b, float t) {
        return (1 - t) * a + t * b;
    }

    public static float clamp(float val, float min, float max) {
        return Math.max(min, Math.min(max, val));
    }

    float Density_Func(Vec2f pos){
        return FractalNoise(4, 0.5343f, 2.2324f, 0.68324f, pos);
    }

    float Density_Func(Vec3f worldPosition) {
        float worldRadius = 200.0f;
        Vec3f world = worldPosition.sub(new Vec3f(0, -worldRadius, 0));
        float worldDist = -worldRadius + world.length();

        float flatlandNoiseScale = 1.0f;
        float flatlandLerpAmount = 0.07f;
        float flatlandYPercent = 1.2f;

        float rockyNoiseScale = 1.5f;
        float rockyLerpAmount = 0.05f;
        float rockyYPercent = 0.7f;

        float maxMountainMixLerpAmount = 0.075f;
        float minMountainMixLerpAmount = 1.0f;

        float mountainBlend;
        float rockyBlend = 1.0f;

        //mountainBlend = saturate(abs(FractalNoise(0.5343f, 2.2324f, 0.68324f, world * 0.11f)) * 4.0f);
        mountainBlend = clamp(Math.abs(FractalNoise(1,0.5343f, 2.2324f, 0.68324f, new Vec2f(worldPosition.X, worldPosition.Z))) * 4.0f, 0.0f, 1.0f);

        float mountain = CalculateNoiseValue(worldPosition);

        float blob = CalculateNoiseValue(worldPosition);
        blob = CLerp(blob, (worldDist) * (flatlandYPercent + ((rockyYPercent - flatlandYPercent) * rockyBlend)),
                flatlandLerpAmount + ((rockyLerpAmount - flatlandLerpAmount) * rockyBlend));

        float result = ((worldDist) / worldRadius) + CLerp(mountain, blob, minMountainMixLerpAmount + ((maxMountainMixLerpAmount - minMountainMixLerpAmount) * mountainBlend));
/*
        for (int i = 0; i < primitiveMods.length; i++){
            float primitive = 0;
            //type
            boolean primChosen = primitiveMods[i].position.w == 0 || primitiveMods[i].position.w == 1 || primitiveMods[i].position.w == 2;

            if (primChosen){
                if (primitiveMods[i].position.w == 0) {
                    primitive = Box(worldPosition, new Vec3f(primitiveMods[i].position.x, primitiveMods[i].position.y, primitiveMods[i].position.z),
                            new Vec3f(primitiveMods[i].size.x, primitiveMods[i].size.y, primitiveMods[i].size.z));
                }
                else if (primitiveMods[i].position.w == 1) {
                    primitive = Sphere(worldPosition, new Vec3f(primitiveMods[i].position.x, primitiveMods[i].position.y, primitiveMods[i].position.z),
                            primitiveMods[i].size.x);
                }
                else if (primitiveMods[i].position.w == 2) {
                    primitive = Cylinder(worldPosition, new Vec3f(primitiveMods[i].position.x, primitiveMods[i].position.y, primitiveMods[i].position.z),
                            new Vec3f(primitiveMods[i].size.x, primitiveMods[i].size.y, primitiveMods[i].size.z));
                }

                if (primitiveMods[i].size.w == 0) {
                    result = Math.max(-primitive, result);
                }
                else {
                    result = Math.min(primitive, result);
                }
            }
        }
*/
        return result;
    }
}
