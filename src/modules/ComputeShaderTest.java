package modules;

import core.utils.BufferUtil;
import org.lwjgl.opengl.GL30;

import java.nio.*;

import static org.lwjgl.opengl.GL30.*;
import static org.lwjgl.opengl.GL43.GL_SHADER_STORAGE_BUFFER;
import static org.lwjgl.opengl.GL43.glDispatchCompute;

public class ComputeShaderTest {
	private int N;
	private ComputeShader shader;
	private ComputeBuffer computeBuffer;
	private int DATA_SIZE = 64 + 1;
	private float[] floatArray;
	private int[] intArray = new int[DATA_SIZE * DATA_SIZE * DATA_SIZE];

	public ComputeShaderTest(int N) {
		this.N = N;

		computeBuffer = new ComputeBuffer();
		computeBuffer.setData(intArray);
		shader = ComputeShader.getInstance();
	}
	
	public void render() {
		shader.bind();
		computeBuffer.bindBufferBase(0);
		shader.updateUniforms(N);
		shader.dispatch(1,1,1);
		computeBuffer.getData(intArray);

		computeBuffer.unmapBuffer();
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
}
