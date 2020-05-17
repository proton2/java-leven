package core.renderer;

import core.buffers.MeshDcVBO;
import core.buffers.VBO;
import core.scene.Component;

public class Renderer extends Component{
	
	private VBO vbo;
	private RenderInfo renderInfo;
	
	public Renderer(VBO vao) {
		this.vbo = vao;
	}
	
	public void render(){
		renderInfo.getConfig().enable();
		renderInfo.getShader().bind();			
		renderInfo.getShader().updateUniforms(getParent());
		getVbo().draw(true);
		renderInfo.getConfig().disable();
	};

	public VBO getVbo() {
		return vbo;
	}

	public void setVbo(VBO vbo) {
		this.vbo = vbo;
	}

	public RenderInfo getRenderInfo() {
		return renderInfo;
	}

	public void setRenderInfo(RenderInfo renderinfo) {
		this.renderInfo = renderinfo;
	}

	public void cleanMesh(){
		if (this.getVbo() instanceof MeshDcVBO && ((MeshDcVBO) this.getVbo()).getMeshBuffer()!=null){
			((MeshDcVBO) this.getVbo()).getMeshBuffer().getVertices().clear();
			((MeshDcVBO) this.getVbo()).getMeshBuffer().getIndicates().clear();
			((MeshDcVBO) this.getVbo()).getMeshBuffer().setNumVertices(0);
			((MeshDcVBO) this.getVbo()).getMeshBuffer().setNumVertices(0);
			((MeshDcVBO) this.getVbo()).setMeshBuffer(null);
		}
	}
}
