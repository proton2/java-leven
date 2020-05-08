package core.buffers;

import core.utils.BufferUtil;
import dc.entities.DebugDrawBuffer;

import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.opengl.GL15.*;
import static org.lwjgl.opengl.GL20.*;
import static org.lwjgl.opengl.GL30.*;

public class DebugMeshVBO implements VBO{
    protected int vbo;
    protected int ibo;
    protected int vaoId;
    protected int size;

    public DebugMeshVBO(){
        vbo = glGenBuffers();
        ibo = glGenBuffers();
        vaoId = glGenVertexArrays();
        size = 0;
    }

    public void addData(DebugDrawBuffer buf){
        size = buf.getNumIndices();
        glBindVertexArray(vaoId);
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, BufferUtil.createDebugFlippedBufferAOS(buf), GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, BufferUtil.createFlippedBuffer(buf.getIndexBuffer()), GL_STATIC_DRAW);
        glVertexAttribPointer(0, 4, GL_FLOAT, false, Float.BYTES * 8, 0);
        glVertexAttribPointer(1, 4, GL_FLOAT, false, Float.BYTES * 8, Float.BYTES * 4);
        glBindVertexArray(0);
    }

    @Override
    public void draw(boolean wireframe) {
        glPolygonMode(GL_FRONT, GL_FILL);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glDisable(GL_TEXTURE_2D);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glBindVertexArray(vaoId);
        glEnableVertexAttribArray(0);
        glEnableVertexAttribArray(1);
        glDrawElements(GL_LINES, size, GL_UNSIGNED_INT, 0);
        glDisableVertexAttribArray(0);
        glDisableVertexAttribArray(1);
        glBindVertexArray(0);

        glDisable(GL_BLEND);
    }

    @Override
    public void delete() {
        glBindVertexArray(vaoId);
        glDeleteBuffers(vbo);
        glDeleteVertexArrays(vaoId);
        glBindVertexArray(0);
    }
}
