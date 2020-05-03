package dc.shaders;

import core.scene.GameObject;
import core.shaders.Shader;
import core.utils.ResourceLoader;

/**
 * Created by proton2 on 28.12.2019.
 */
public class DcSimpleShader extends Shader{
  private static DcSimpleShader instance = null;

  public static DcSimpleShader getInstance()
  {
    if(instance == null)
    {
      instance = new DcSimpleShader();
    }
    return instance;
  }

  protected DcSimpleShader()
  {
    super();
    addVertexShader(ResourceLoader.loadShader("shaders/dcVs.glsl"));
    addFragmentShader(ResourceLoader.loadShader("shaders/dcFs.glsl"));
    compileShader();

    addUniform("modelViewProjectionMatrix");
  }

  public void updateUniforms(GameObject object)
  {
    setUniform("modelViewProjectionMatrix", object.getTransform().getModelViewProjectionMatrix());
  }
}
