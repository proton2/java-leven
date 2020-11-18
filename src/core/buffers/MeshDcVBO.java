package core.buffers;

import dc.entities.MeshBuffer;

import static org.lwjgl.opengl.GL11.GL_FLOAT;
import static org.lwjgl.opengl.GL15.*;
import static org.lwjgl.opengl.GL20.glVertexAttribPointer;
import static org.lwjgl.opengl.GL30.glBindVertexArray;

public class MeshDcVBO extends MeshVBO{
    public MeshDcVBO(MeshBuffer meshBuffer) {
        addData(meshBuffer);
    }

    private void addData(MeshBuffer meshBuffer) {
        size = meshBuffer.getNumIndicates();
        glBindVertexArray(vaoId);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, meshBuffer.getVertices(), GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, meshBuffer.getIndicates(), GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, false, Float.BYTES * 9, 0);
        glVertexAttribPointer(1, 3, GL_FLOAT, false, Float.BYTES * 9, Float.BYTES * 3);
        glVertexAttribPointer(2, 3, GL_FLOAT, false, Float.BYTES * 9, Float.BYTES * 6);

        glBindVertexArray(0);
    }
}
