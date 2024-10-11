package dc.impl.notused.gpu.computeshader;

import core.scene.GameObject;
import core.shaders.Shader;
import core.utils.ResourceLoader;
import dc.entities.ModelEntity;

public class CSGActorShader extends Shader {
    private static CSGActorShader instance = null;

    public static CSGActorShader getInstance() {
        if(instance == null) {
            instance = new CSGActorShader();
        }
        return instance;
    }

    private CSGActorShader(){
        super();
        addVertexShader(ResourceLoader.loadShader("shaders/csgAction.glsl"));
        addFragmentShader(ResourceLoader.loadShader("shaders/csgAction.frag"));
        compileShader();

        addUniform("worldMatrix");
        //addUniform("worldMatrix");
    }

    public void updateUniforms(GameObject object) {
        //setUniform("modelViewProjectionMatrix", object.getTransform().getModelViewProjectionMatrix());
    }

    public void updateTransform(ModelEntity modelEntity) {
        setUniform("worldMatrix", modelEntity.getTransform().getModelViewProjectionMatrix());
        //setUniform("worldMatrix", transform.getWorldMatrix());
    }
}