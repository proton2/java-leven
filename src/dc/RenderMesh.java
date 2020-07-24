package dc;

import core.math.Vec3i;
import core.renderer.Renderer;
import dc.entities.MeshBuffer;

public class RenderMesh {
    public Vec3i min;
    public int size;
    MeshBuffer renderMesh, seamMesh;
    Renderer meshRender, seamRender;

    public RenderMesh(Vec3i min, int size, MeshBuffer renderMesh, MeshBuffer seamMesh) {
        this.min = min;
        this.size = size;
        this.renderMesh = renderMesh;
        this.seamMesh = seamMesh;
    }
}
