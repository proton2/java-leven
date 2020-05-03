package dc.shaders;

import core.scene.GameObject;
import core.shaders.Shader;
import core.utils.ResourceLoader;

public class RenderDebugShader extends Shader {
    private static RenderDebugShader instance = null;

    public static RenderDebugShader getInstance() {
        if(instance == null) {
            instance = new RenderDebugShader();
        }
        return instance;
    }

    private RenderDebugShader(){
        super();
        addVertexShader(ResourceLoader.loadShader("shaders/debug.vert"));
        addFragmentShader(ResourceLoader.loadShader("shaders/debug.frag"));
        compileShader();

        addUniform("MVP");
    }

    public void updateUniforms(GameObject object) {
        setUniform("MVP", object.getTransform().getModelViewProjectionMatrix());
    }
}
