package modules;

import core.utils.BufferUtil;
import org.lwjgl.BufferUtils;
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
	private final int ssbo;
	private IntBuffer intBuffer;
	private FloatBuffer floatBuffer;
	private ByteBuffer byteBuffer;
	private int dataSize;
	private Class<? extends Sizeable> clazz;
	private Sizeable[] sizeableData;
	
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

	public void setData(Sizeable[] data) {
		this.sizeableData = data;
		clazz = data[0].getClass();

		//dataSize = data.length * data[0].getSize();
		dataSize = data.length * 8;
		floatBuffer = BufferUtils.createFloatBuffer(dataSize);
		for(Sizeable item : data){
			item.toFloatBuffer(floatBuffer);
		}
		floatBuffer.rewind();

//		byte[] byteData = new byte[dataSize];
//		byteBuffer.get(byteData);
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, floatBuffer, GL_DYNAMIC_COPY);
	}

	public void getData(Sizeable[] data){
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		ByteBuffer buffer = GL30.glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, dataSize, GL_MAP_READ_BIT);
		byte[] byteData = new byte[dataSize];
		buffer.get(byteData);

		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
	}

	public void getData(DensityPrimitive[] data){
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);

		ByteBuffer buffer = GL30.glMapBufferRange(GL_SHADER_STORAGE_BUFFER, 0, dataSize * 4, GL_MAP_READ_BIT);
		FloatBuffer fb = buffer.asFloatBuffer();
		float[] floatData = new float[dataSize];
		fb.get(floatData);

		int index = 0;
		for(DensityPrimitive elem : data){
			elem.position.x = floatData[index++];
			elem.position.y = floatData[index++];
			elem.position.z = floatData[index++];
			elem.position.w = floatData[index++];
			elem.size.x = floatData[index++];
			elem.size.y = floatData[index++];
			elem.size.z = floatData[index++];
			elem.size.w = floatData[index++];
		}
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
	}

	public void updateData(int[] data) {
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		ByteBuffer mappedBuffer = GL30.glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_WRITE_ONLY);
		for(int i=0; i<data.length-1; i++) {
			mappedBuffer.putInt(data[i]);
		}
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
	}

	public void updateData(FloatBuffer buffer, int length){
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		ByteBuffer mappedBuffer = glMapBuffer(GL_SHADER_STORAGE_BUFFER, GL_READ_WRITE, length, null);
		mappedBuffer.clear();
		for (int i=0; i<length/Float.BYTES; i++){
			mappedBuffer.putFloat(buffer.get(i));
		}
		mappedBuffer.flip();
		glUnmapBuffer(GL_SHADER_STORAGE_BUFFER);
	}

	public void changeSSBO(int position, float[] data){
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);

		FloatBuffer ArrayData = BufferUtil.createFlippedBuffer(data);
		glBufferSubData(GL_SHADER_STORAGE_BUFFER,position * 4, ArrayData);

		glBindBuffer(GL_SHADER_STORAGE_BUFFER, 0);
	}
}
