package modules;

import core.utils.BufferUtil;
import org.lwjgl.opengl.GL30;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

import static org.lwjgl.opengl.GL15.*;
import static org.lwjgl.opengl.GL30.GL_MAP_READ_BIT;
import static org.lwjgl.opengl.GL30.glBindBufferBase;
import static org.lwjgl.opengl.GL43.GL_SHADER_STORAGE_BUFFER;

/**
 * Shader Storage Buffer Object
 */

public class ComputeBuffer {
	private int ssbo;
	private IntBuffer intBuffer;
	private FloatBuffer floatBuffer;
	private ByteBuffer byteBuffer;
	private int dataSize;
	
	public ComputeBuffer() {
		ssbo = glGenBuffers();
	}

	/**
	link layout binding index in shader program to ssbo buffer id
	 */
	public void bindBufferBase(int index) {
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, index, ssbo);
	}
	
	public void setData(int[] data) {
		intBuffer = BufferUtil.createFlippedBuffer(data);
		dataSize = data.length;
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, intBuffer, GL_DYNAMIC_COPY);
	}

	public void setData(float[] data) {
		floatBuffer = BufferUtil.createFlippedBuffer(data);
		dataSize = data.length;
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, floatBuffer, GL_DYNAMIC_COPY);
	}

	public void setData(byte[] data) {
		byteBuffer = BufferUtil.createFlippedBuffer(data);
		dataSize = data.length;
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, byteBuffer, GL_STATIC_READ);
	}

	public void getData(float[] data){
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		ByteBuffer buffer = GL30.glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, dataSize * 4, GL_MAP_READ_BIT);
		FloatBuffer fb = buffer.asFloatBuffer();
		fb.get(data);
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
	}

	public void getData(int[] data){
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		ByteBuffer buffer = GL30.glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, dataSize * 4, GL_MAP_READ_BIT);
		IntBuffer fb = buffer.asIntBuffer();
		fb.get(data);
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
	}

	public void getData(byte[] data){
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		ByteBuffer buffer = GL30.glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, dataSize, GL_MAP_READ_BIT);
		buffer.get(data);
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
	}
}
