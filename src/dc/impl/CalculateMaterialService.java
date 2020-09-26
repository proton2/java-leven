package dc.impl;

import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import dc.utils.VoxelHelperUtils;
import modules.CalculateMaterialComputeShader;
import modules.ComputeBuffer;
import modules.DensityPrimitive;

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
        shader.dispatch(1,1,1);

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
                    float density = Density_Func(world_pos);
                    int index = field_index(local_pos);
                    int material = density < 0.f ? MATERIAL_SOLID : MATERIAL_AIR;
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

    float Perlin(float x, float y, float z) {
        int X = x > 0 ? (int)x : (int)x - 1;
        int Y = y > 0 ? (int)y : (int)y - 1;
        int Z = z > 0 ? (int)z : (int)z - 1;

        x = x - X;
        y = y - Y;
        z = z - Z;

        X = X & 255;
        Y = Y & 255;
        Z = Z & 255;

        int gi000 = permutations[X + permutations[Y + permutations[Z]]] % 12;
        int gi001 = permutations[X + permutations[Y + permutations[Z + 1]]] % 12;
        int gi010 = permutations[X + permutations[Y + 1 + permutations[Z]]] % 12;
        int gi011 = permutations[X + permutations[Y + 1 + permutations[Z + 1]]] % 12;
        int gi100 = permutations[X + 1 + permutations[Y + permutations[Z]]] % 12;
        int gi101 = permutations[X + 1 + permutations[Y + permutations[Z + 1]]] % 12;
        int gi110 = permutations[X + 1 + permutations[Y + 1 + permutations[Z]]] % 12;
        int gi111 = permutations[X + 1 + permutations[Y + 1 + permutations[Z + 1]]] % 12;

        float n000 = Vec3Dot(Grad3[gi000], new Vec3i(x, y, z));
        float n100 = Vec3Dot(Grad3[gi100], new Vec3i(x - 1, y, z));
        float n010 = Vec3Dot(Grad3[gi010], new Vec3i(x, y - 1, z));
        float n110 = Vec3Dot(Grad3[gi110], new Vec3i(x - 1, y - 1, z));
        float n001 = Vec3Dot(Grad3[gi001], new Vec3i(x, y, z - 1));
        float n101 = Vec3Dot(Grad3[gi101], new Vec3i(x - 1, y, z - 1));
        float n011 = Vec3Dot(Grad3[gi011], new Vec3i(x, y - 1, z - 1));
        float n111 = Vec3Dot(Grad3[gi111], new Vec3i(x - 1, y - 1, z - 1));

        float u = (x * x * x * (x * (x * 6 - 15) + 10));
        float v = (y * y * y * (y * (y * 6 - 15) + 10));
        float w = (z * z * z * (z * (z * 6 - 15) + 10));

        float nx00 = lerp(n000, n100, u);
        float nx01 = lerp(n001, n101, u);
        float nx10 = lerp(n010, n110, u);
        float nx11 = lerp(n011, n111, u);

        float nxy0 = lerp(nx00, nx10, v);
        float nxy1 = lerp(nx01, nx11, v);

        float nxyz = lerp(nxy0, nxy1, w);

        return nxyz;
    }

    float FractalNoise(int octaves, float frequency, float lacunarity, float persistence, Vec3f position) {
        float SCALE = 1.0f / 128.0f;
        Vec3f p = position.mul(SCALE);
        float nois = 0.0f;

        float amplitude = 1.0f;
        p = p.mul(frequency);

        for (int i = 0; i < octaves; i++) {
            nois += Perlin(p.X, p.Y, p.Z) * amplitude;
            p = p.mul(frequency);
            amplitude *= persistence;
        }

        return nois;
    }

    float FractalNoise(float frequency, float lacunarity, float persistence, Vec3f position) {
        float SCALE = 1.0f / 128.0f;
        Vec3f p = position.mul(SCALE);
        float nois = 0.0f;

        float amplitude = 1.0f;
        p = p.mul(frequency);

        nois += Perlin(p.X, p.Y, p.Z) * amplitude;
        p = p.mul(frequency);
        amplitude *= persistence;

        return nois;
    }

    float CalculateNoiseValue(Vec3f pos, float scale) {
        return FractalNoise(4, 0.5343f, 2.2324f, 0.68324f, pos.mul(scale));
    }

    float CLerp(float a, float b, float t) {
        return (1 - t) * a + t * b;
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

        float mountainBlend = 0.0f;
        float rockyBlend = 1.0f;

        //mountainBlend = saturate(abs(FractalNoise(0.5343f, 2.2324f, 0.68324f, world * 0.11f)) * 4.0f);
        mountainBlend = VoxelHelperUtils.clamp(Math.abs(FractalNoise(0.5343f, 2.2324f, 0.68324f, world.mul(0.11f))) * 4.0f, 0.0f, 1.0f);

        float mountain = CalculateNoiseValue(world, 0.07f);

        float blob = CalculateNoiseValue(world, flatlandNoiseScale + ((rockyNoiseScale - flatlandNoiseScale) * rockyBlend));
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
