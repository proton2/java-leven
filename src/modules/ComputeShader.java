package modules;

import core.shaders.Shader;
import core.utils.ResourceLoader;

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
}
