package modules;

import core.utils.BufferUtil;

import java.nio.ByteBuffer;
import java.nio.IntBuffer;

import static org.lwjgl.opengl.GL15.*;
import static org.lwjgl.opengl.GL30.glBindBufferBase;
import static org.lwjgl.opengl.GL43.GL_SHADER_STORAGE_BUFFER;

/**
 * Shader Storage Buffer Object
 */

public class GLShaderStorageBuffer {
	
	private int ssbo;
	
	public GLShaderStorageBuffer()
	{
		ssbo = glGenBuffers();
	}
	
	public void addData(int[] data)
	{
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, BufferUtil.createFlippedBuffer(data), GL_STATIC_READ);
	}

	public void addChangedData(IntBuffer intBuffer)
	{
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, intBuffer, GL_DYNAMIC_COPY);
	}
	
	public void addData(ByteBuffer data)
	{
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, data, GL_STATIC_READ);
	}
	
	public void bindBufferBase(int index)
	{
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, index, ssbo);
	}

}
