package modules;

import core.utils.BufferUtil;
import org.lwjgl.opengl.GL30;

import java.nio.Buffer;
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

	public void unmapBuffer(){
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
	}
	
	public void setData(int[] data) {
		intBuffer = BufferUtil.createFlippedBuffer(data);
		dataSize = data.length;
		setData(intBuffer);
	}

	public void setData(float[] data) {
		floatBuffer = BufferUtil.createFlippedBuffer(data);
		dataSize = data.length;
		setData(floatBuffer);
	}

	public void setData(byte[] data) {
		byteBuffer = BufferUtil.createFlippedBuffer(data);
		dataSize = data.length;
		setData(byteBuffer);
	}

	private void setData(IntBuffer intBuffer) {
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, intBuffer, GL_DYNAMIC_COPY);
	}

	private void setData(FloatBuffer floatBuffer) {
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, floatBuffer, GL_DYNAMIC_COPY);
	}

	private void setData(ByteBuffer data) {
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, data, GL_STATIC_READ);
	}

	public void getData(float[] data){
		ByteBuffer buffer = GL30.glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, dataSize * 4, GL_MAP_READ_BIT);
		FloatBuffer fb = buffer.asFloatBuffer();
		fb.get(data);
	}

	public void getData(int[] data){
		ByteBuffer buffer = GL30.glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, dataSize * 4, GL_MAP_READ_BIT);
		IntBuffer fb = buffer.asIntBuffer();
		fb.get(data);
	}

	public void getData(byte[] data){
		ByteBuffer buffer = GL30.glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, dataSize, GL_MAP_READ_BIT);
		buffer.get(data);
	}
}
