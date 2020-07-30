package modules;

import java.util.Random;

public class ComputeShaderTest {
	private int N;
	private ComputeShader shader;
	private ComputeBuffer computeBuffer, computeBufferPerm;
	private int DATA_SIZE = 64 + 1;
	private float[] floatArray;
	private int[] intArray = new int[DATA_SIZE * DATA_SIZE * DATA_SIZE];
	static int[] permutations = new int[512];

	public ComputeShaderTest(int N) {
		this.N = N;
		InitPermutations(1200);

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

		shader.updateUniforms(N);
		shader.dispatch(1,1,1);

		computeBuffer.getData(intArray);
		computeBufferPerm.getData(permutations);
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
