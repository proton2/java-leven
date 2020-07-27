package modules;

import core.shaders.Shader;
import core.utils.ResourceLoader;

import static org.lwjgl.opengl.GL43.glDispatchCompute;

public class ComputeShader extends Shader {

	private static ComputeShader instance = null;
	
	public static ComputeShader getInstance()
	{
	    if(instance == null) 
	    {
	    	instance = new ComputeShader();
	    }
	      return instance;
	}
	
	protected ComputeShader()
	{
		super();
		
		addComputeShader(ResourceLoader.loadShader("shaders/testComputeShader.comp"));
		compileShader();
		
		addUniform("N");
	}
	

	public void updateUniforms(int N)
	{
		setUniformi("N", N);
	}

	public void dispatch(int num_groups_x, int num_groups_y, int num_groups_z){
		glDispatchCompute(num_groups_x, num_groups_y, num_groups_z);
	}
}