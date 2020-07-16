package modules;

import core.utils.BufferUtil;
import org.lwjgl.opengl.GL30;

import java.nio.IntBuffer;

import static org.lwjgl.opengl.GL30.GL_MAP_READ_BIT;
import static org.lwjgl.opengl.GL43.GL_SHADER_STORAGE_BUFFER;
import static org.lwjgl.opengl.GL43.glDispatchCompute;

public class ComputeShaderTest {
	private int N;
	private ComputeShader shader;
	private GLShaderStorageBuffer bitReversedSSBO;
	private int DATA_SIZE = 64;
	private int[] data;
	private IntBuffer intBuffer;
	
	public ComputeShaderTest(int N)
	{
		this.N = N;
		data = initData();
		intBuffer = BufferUtil.createFlippedBuffer(data);
		bitReversedSSBO = new GLShaderStorageBuffer();
		bitReversedSSBO.addChangedData(intBuffer);
		shader = ComputeShader.getInstance();
	}
	
	public void render() {
		shader.bind();
		bitReversedSSBO.bindBufferBase(0);
		shader.updateUniforms(N);
		glDispatchCompute(DATA_SIZE,1,1);

		IntBuffer byteBuffer = GL30.glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, 64, GL_MAP_READ_BIT).asIntBuffer();
		int t = 3;
	}

	private int[] initData(){
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
}
