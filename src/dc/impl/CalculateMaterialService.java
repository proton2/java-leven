package dc.impl;

import core.math.Vec2f;
import core.math.Vec3f;
import core.math.Vec3i;
import core.math.Vec4f;
import modules.CalculateMaterialComputeShader;
import modules.ComputeBuffer;
import modules.DensityPrimitive;

import java.util.Random;

import static dc.VoxelOctree.MATERIAL_AIR;
import static dc.VoxelOctree.MATERIAL_SOLID;

public class CalculateMaterialService {
    CalculateMaterialComputeShader shader;
    private ComputeBuffer fieldMaterialsComputeBuffer, permComputeBuffer, densityPrimitiveComputeBuffer;
    private final int fieldSize;

    static int[] permutations = new int[512];
    private DensityPrimitive[] primitiveMods;

    public CalculateMaterialService(int fieldSize){
        this.fieldSize = fieldSize;
    }

    public static void InitPermutations(int seed, int[] permutations) {
        Random random = new Random(seed);
        for (int i = 0; i < 256; i++)
            permutations[i] = (int) (256 * (random.nextInt(10000) / 10000.0f));

        for (int i = 256; i < 512; i++)
            permutations[i] = permutations[i - 256];
    }

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
        shader.updateOffcetUniforms(offset);
        shader.dispatch(7,7,7);

        permComputeBuffer.getData(permutations);
        densityPrimitiveComputeBuffer.getData(primitiveMods);
        fieldMaterialsComputeBuffer.getData(fieldMaterials);

        //calculateTest(offset, sampleScale, fieldMaterials);
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

    protected int field_index(Vec3i pos) {
        return pos.x + (pos.y * fieldSize) + (pos.z * fieldSize * fieldSize);
    }

    Vec3i Grad3[] = {
            new Vec3i(1,1,0), new Vec3i(-1,1,0), new Vec3i(1,-1,0), new Vec3i(-1,-1,0),
            new Vec3i(1,0,1), new Vec3i(-1,0,1), new Vec3i(1,0,-1), new Vec3i(-1,0,-1),
            new Vec3i(0,1,1), new Vec3i(0,-1,1), new Vec3i(0,1,-1), new Vec3i(0,-1,-1)
    };

    float Density_Func(Vec2f pos){
        return FractalNoise(4, 0.5343f, 2.2324f, 0.68324f, pos);
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

    float Perlin(float x, float y) {
        int i = (int) (x > 0 ? x : x - 1);
        int j = (int) (y > 0 ? y : y - 1);

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

        float u = (x * x * x * (x * (x * 6 - 15) + 10));
        float v = (y * y * y * (y * (y * 6 - 15) + 10));

        //float nyl = Mathf.Lerp(nll, nhl, u);
        float nyl = (1-u)*nll + u*nhl;
        //float nyh = Mathf.Lerp(nlh, nhh, u);
        float nyh = (1-u)*nlh + u*nhh;

        //float nxy = Mathf.Lerp(nyl, nyh, v);
        float nxy = (1-v)*nyl + v*nyh;

        return nxy;
    }
}
