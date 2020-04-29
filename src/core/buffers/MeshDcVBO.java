package core.buffers;

import core.model.Vertex;
import core.utils.BufferUtil;

import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL15.*;
import static org.lwjgl.opengl.GL20.*;
import static org.lwjgl.opengl.GL30.glBindVertexArray;

public class MeshDcVBO extends MeshVBO{
    public MeshDcVBO() {
    }

    public void addData(Vertex[] vertArray, int[] indices) {
        size = indices.length;
        glBindVertexArray(vaoId);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, BufferUtil.createDcFlippedBufferAOS(vertArray), GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, BufferUtil.createFlippedBuffer(indices), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, false, Float.BYTES * 9, 0);
        glVertexAttribPointer(1, 3, GL_FLOAT, false, Float.BYTES * 9, Float.BYTES * 3);
        glVertexAttribPointer(2, 3, GL_FLOAT, false, Float.BYTES * 9, Float.BYTES * 6);

        glBindVertexArray(0);
    }
}
