package dc;

import core.buffers.MeshDcVBO;
import core.configs.CW;
import core.math.Vec3i;
import core.renderer.RenderInfo;
import core.renderer.Renderer;
import dc.entities.MeshBuffer;
import dc.shaders.DcSimpleShader;

public class RenderMesh {
    public final Vec3i min;
    public final int size;
    public final Renderer render;

    public RenderMesh(Vec3i min, int size, MeshBuffer meshBuffer) {
        this.min = min;
        this.size = size;
        this.render = new Renderer(new MeshDcVBO(meshBuffer),
                new RenderInfo(new CW(), DcSimpleShader.getInstance()));
    }
}