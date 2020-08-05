package modules;

import core.math.Vec3f;
import core.math.Vec4f;

import java.util.Random;

public class ComputeShaderTest {
	private int N;
	private ComputeShader shader;
	private ComputeBuffer computeBuffer, computeBufferPerm, densityPrimitiveComputeBuffer;
	private int DATA_SIZE = 64 + 1;
	private float[] floatArray;
	private int[] intArray = new int[DATA_SIZE * DATA_SIZE * DATA_SIZE];
	static int[] permutations = new int[512];
	private DensityPrimitive[] primitiveMods;

	public ComputeShaderTest(int N) {
		this.N = N;
		InitPermutations(1200);
		primitiveMods = new DensityPrimitive[]{
				new DensityPrimitive(new Vec4f(3.1f, 4.2f, 5.7f, 1), new Vec4f(6.4f, 7.1f, 9.6f, 1)),
				new DensityPrimitive(new Vec4f(5.8f, 3.7f, 8.1f, 1), new Vec4f(5.9f, 2.6f, 1.8f, 1))
		};

		densityPrimitiveComputeBuffer = new ComputeBuffer();
		densityPrimitiveComputeBuffer.setData(primitiveMods);

		computeBuffer = new ComputeBuffer();
		computeBuffer.setData(intArray);

		computeBufferPerm = new ComputeBuffer();
		computeBufferPerm.setData(permutations);

		shader = ComputeShader.getInstance();
	}
	
	public void render() {
		shader.bind();

		computeBuffer.bindBufferBase(0);
		computeBufferPerm.bindBufferBase(1);
		densityPrimitiveComputeBuffer.bindBufferBase(2);

		shader.updateUniforms(N);
		shader.dispatch(1,1,1);

		computeBuffer.getData(intArray);
		computeBufferPerm.getData(permutations);
		densityPrimitiveComputeBuffer.getData(primitiveMods);
	}

	private int[] initIntData(){
		int[] dataArray = new int[DATA_SIZE * DATA_SIZE * DATA_SIZE];
		for (int x=0; x<DATA_SIZE; x++){
			for (int y=0; y<DATA_SIZE; y++){
				for (int z=0; z<DATA_SIZE; z++){
					int index = x + (y * DATA_SIZE) + (z * DATA_SIZE * DATA_SIZE);
					dataArray[index] = 0;
				}
			}
		}
		return dataArray;
	}

	private float[] initFloatData(){
		float[] dataArray = new float[DATA_SIZE * DATA_SIZE * DATA_SIZE];
		for (int x=0; x<DATA_SIZE; x++){
			for (int y=0; y<DATA_SIZE; y++){
				for (int z=0; z<DATA_SIZE; z++){
					int index = x + (y * DATA_SIZE) + (z * DATA_SIZE * DATA_SIZE);
					dataArray[index] = 0;
				}
			}
		}
		return dataArray;
	}

	public static void InitPermutations(int seed)
	{
		Random random = new Random(seed);
		for (int i = 0; i < 256; i++)
			permutations[i] = (int) (256 * (random.nextInt(10000) / 10000.0f));

		for (int i = 256; i < 512; i++)
			permutations[i] = permutations[i - 256];
	}
}
