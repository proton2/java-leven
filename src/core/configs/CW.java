package core.configs;

import static org.lwjgl.opengl.GL11.GL_CCW;
import static org.lwjgl.opengl.GL11.GL_CW;
import static org.lwjgl.opengl.GL11.glFrontFace;

/**
 * Created by proton2 on 31.12.2019.
 */
public class CW implements RenderConfig{
    public void enable(){
        glFrontFace(GL_CW);
    }

    public void disable(){
        glFrontFace(GL_CCW);
    }
}
