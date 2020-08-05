package modules;

import core.math.Vec3f;
import core.math.Vec4f;

import java.io.UnsupportedEncodingException;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.util.List;

public interface Sizeable {
    default int sizeof(Class<?> dataType) {
        if (dataType == null) throw new NullPointerException();

        if (dataType == int.class    || dataType == Integer.class)   return 4;
        if (dataType == short.class  || dataType == Short.class)     return 2;
        if (dataType == byte.class   || dataType == Byte.class)      return 1;
        if (dataType == char.class   || dataType == Character.class) return 2;
        if (dataType == long.class   || dataType == Long.class)      return 8;
        if (dataType == float.class  || dataType == Float.class)     return 4;
        if (dataType == double.class || dataType == Double.class)    return 8;

        if (dataType == Vec3f.class)    return 4 * 3;
        if (dataType == Vec4f.class)    return 4 * 4;

        return 4; // 32-bit memory pointer...
        // (I'm not sure how this works on a 64-bit OS)
    }

    default byte[] intToByteArray(int value) {
        return new byte[] {
                (byte)(value >>> 24),
                (byte)(value >>> 16),
                (byte)(value >>> 8),
                (byte)value};
    }

    default byte[] floatToByteArray(float value) {
        int intBits = Float.floatToIntBits(value);
        return new byte[] {
                (byte) (intBits >> 24),
                (byte) (intBits >> 16),
                (byte) (intBits >> 8),
                (byte) (intBits)
        };
    }

    default byte[] stringToByteArray(String s){
        try {
            return s.getBytes("UTF-8");
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        }
        return new byte[0];
    }

    default byte[] enumToByteArray(Enum<?> e) {
        return e.name().getBytes();
    }

    default byte booleanToByte(boolean vIn){
        return (byte)(vIn?1:0);
    }

    default float convertToFloat(byte[] array) {
        ByteBuffer buffer = ByteBuffer.wrap(array);
        return buffer.getFloat();
    }

    List<Class<?>> getFields();

    default int getSize(){
        int size = 0;
        List<Class<?>> classes = getFields();
        for (Class<?> c : classes){
            size += sizeof(c);
        }
        return size;
    }

    void toByteBuffer(ByteBuffer buf);
    void toFloatBuffer(FloatBuffer buf);
}