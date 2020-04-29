package core.utils;

import core.math.*;
import core.model.Vertex;
import dc.DebugDrawBuffer;
import org.lwjgl.BufferUtils;

import java.nio.DoubleBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

public class BufferUtil {

	public static FloatBuffer createFloatBuffer(int size)
	{
		return BufferUtils.createFloatBuffer(size);
	}
	
	public static IntBuffer createIntBuffer(int size)
	{
		return BufferUtils.createIntBuffer(size);
	}

	public static IntBuffer createFlippedBuffer(int... values)
	{
		IntBuffer buffer = createIntBuffer(values.length);
		buffer.put(values);
		buffer.flip();
		
		return buffer;
	}

	public static FloatBuffer createFlippedBufferAOS(Vertex[] vertices) {
		FloatBuffer buffer = createFloatBuffer(vertices.length * 14);

		for(int i = 0; i < vertices.length; i++) {
			buffer.put(vertices[i].getPos().getX());
			buffer.put(vertices[i].getPos().getY());
			buffer.put(vertices[i].getPos().getZ());
			buffer.put(vertices[i].getNormal().getX());
			buffer.put(vertices[i].getNormal().getY());
			buffer.put(vertices[i].getNormal().getZ());
			buffer.put(vertices[i].getTextureCoord().getX());
			buffer.put(vertices[i].getTextureCoord().getY());

			if (vertices[i].getTangent() != null && vertices[i].getBitangent() != null){
				buffer.put(vertices[i].getTangent().getX());
				buffer.put(vertices[i].getTangent().getY());
				buffer.put(vertices[i].getTangent().getZ());
				buffer.put(vertices[i].getBitangent().getX());
				buffer.put(vertices[i].getBitangent().getY());
				buffer.put(vertices[i].getBitangent().getZ());
			}
		}
		buffer.flip();
		return buffer;
	}

	public static FloatBuffer createDcFlippedBufferAOS(Vertex[] vertices) {
		FloatBuffer buffer = createFloatBuffer(vertices.length * 9);
		for (Vertex vertice : vertices) {
			buffer.put(vertice.getPos().getX());
			buffer.put(vertice.getPos().getY());
			buffer.put(vertice.getPos().getZ());
			buffer.put(vertice.getNormal().getX());
			buffer.put(vertice.getNormal().getY());
			buffer.put(vertice.getNormal().getZ());
			buffer.put(vertice.getColor().getX());
			buffer.put(vertice.getColor().getY());
			buffer.put(vertice.getColor().getZ());
		}
		buffer.flip();
		return buffer;
	}

	public static FloatBuffer createDebugFlippedBufferAOS(DebugDrawBuffer buf) {
		FloatBuffer buffer = createFloatBuffer(buf.getVertexBuffer().length * 8);
		for (int i=0; i<buf.getVertexBuffer().length; i++) {
			buffer.put(buf.getVertexBuffer()[i].getX());
			buffer.put(buf.getVertexBuffer()[i].getY());
			buffer.put(buf.getVertexBuffer()[i].getZ());
			buffer.put(buf.getVertexBuffer()[i].getW());

			buffer.put(buf.getColourBuffer()[i].getX());
			buffer.put(buf.getColourBuffer()[i].getY());
			buffer.put(buf.getColourBuffer()[i].getZ());
			buffer.put(buf.getColourBuffer()[i].getW());
		}
		buffer.flip();
		return buffer;
	}
	
	public static FloatBuffer createFlippedBuffer(Matrix4f matrix)
	{
		FloatBuffer buffer = createFloatBuffer(4 * 4);
		
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 4; j++)
				buffer.put(matrix.get(i, j));
		
		buffer.flip();
		
		return buffer;
	}
}
