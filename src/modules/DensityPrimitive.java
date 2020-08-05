package modules;

import core.math.Vec4f;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;

public class DensityPrimitive implements Sizeable{
    public Vec4f position;
    public Vec4f size;

    public DensityPrimitive(Vec4f position, Vec4f size) {
        this.position = position;
        this.size = size;
    }

    @Override
    public List<Class<?>> getFields() {
        ArrayList<Class<?>> fields = new ArrayList<>(2);
        fields.add(Vec4f.class);
        fields.add(Vec4f.class);
        return fields;
    }

    @Override
    public void toByteBuffer(ByteBuffer buf){
        buf.putFloat(position.x);
        buf.putFloat(position.y);
        buf.putFloat(position.z);
        buf.putFloat(position.w);
        buf.putFloat(size.x);
        buf.putFloat(size.y);
        buf.putFloat(size.z);
        buf.putFloat(size.w);
    }

    @Override
    public void toFloatBuffer(FloatBuffer buf){
        buf.put(position.x);
        buf.put(position.y);
        buf.put(position.z);
        buf.put(position.w);
        buf.put(size.x);
        buf.put(size.y);
        buf.put(size.z);
        buf.put(size.w);
    }
}
