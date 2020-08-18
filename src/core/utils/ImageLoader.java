package core.utils;

import org.lwjgl.BufferUtils;

import java.io.*;
import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.channels.Channels;
import java.nio.channels.FileChannel;
import java.nio.channels.ReadableByteChannel;
import java.nio.channels.SeekableByteChannel;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import static org.lwjgl.opengl.GL11.*;
import static org.lwjgl.stb.STBImage.*;
import static org.lwjgl.stb.STBImageWrite.stbi_write_tga;

public class ImageLoader {

public static int loadImage(String file) {
		
		ByteBuffer imageBuffer;
        try {
            imageBuffer = ioResourceToByteBuffer(file, 128 * 128);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        
        IntBuffer w    = BufferUtils.createIntBuffer(1);
        IntBuffer h    = BufferUtils.createIntBuffer(1);
        IntBuffer c = BufferUtils.createIntBuffer(1);

        // Use info to read image metadata without decoding the entire image.
        if (!stbi_info_from_memory(imageBuffer, w, h, c)) {
            throw new RuntimeException("Failed to read image information: " + stbi_failure_reason());
        }
  
        // Decode the image
        ByteBuffer image = stbi_load_from_memory(imageBuffer, w, h, c, 0);
        if (image == null) {
            throw new RuntimeException("Failed to load image: " + stbi_failure_reason());
        }
       
        int width = w.get(0);
        int height = h.get(0);
        int comp = c.get(0);
        
        int texId = glGenTextures();

        glBindTexture(GL_TEXTURE_2D, texId);
        
        if (comp == 3) {
            if ((width & 3) != 0) {
                glPixelStorei(GL_UNPACK_ALIGNMENT, 2 - (width & 1));
            }
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
        } else {
        	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image);
        }
        
        stbi_image_free(image);
        
		return texId;
	}

    public static ByteBuffer floatArray2ByteArray(float[] values){
        ByteBuffer buffer = ByteBuffer.allocate(4 * values.length);
        for (float value : values){
            buffer.putFloat(value);
        }
        buffer.flip();
        return buffer;
    }

	public static boolean saveImageToTga(String fileName, int width, int height, int comp, float[] data){
        ByteBuffer byteBuffer = floatArray2ByteArray(data);
        return stbi_write_tga(fileName, width, height, comp, byteBuffer);
        //return stbi_write_png(fileName, width, height, comp, floatArray2ByteArray(data), width*3);
    }

    public static void loadImageToFloatArray(float[] data, String fileName){
        try{
            RandomAccessFile rFile = new RandomAccessFile(fileName, "r");
            FileChannel inChannel = rFile.getChannel();
            ByteBuffer buf_in = ByteBuffer.allocate(4*data.length);
            buf_in.clear();
            inChannel.read(buf_in);
            buf_in.rewind();
            buf_in.asFloatBuffer().get(data);
            inChannel.close();
        } catch (IOException ex) {
            System.err.println(ex.getMessage());
        }
    }

    public static void saveImageToFloat(float[] data, String fileName) {
        DataOutputStream out = null;
        try {
            out = new DataOutputStream(new FileOutputStream(fileName));
            byte[] buf = new byte[4 * data.length];
            for (int i = 0; i < data.length; ++i) {
                int val = Float.floatToRawIntBits(data[i]);
                buf[4 * i] = (byte) (val >> 24);
                buf[4 * i + 1] = (byte) (val >> 16);
                buf[4 * i + 2] = (byte) (val >> 8);
                buf[4 * i + 3] = (byte) (val);
            }
            out.write(buf);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
	
	public static ByteBuffer loadImageToByteBuffer(String file){
		ByteBuffer imageBuffer;
        try {
            imageBuffer = ioResourceToByteBuffer(file, 128 * 128);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        
        IntBuffer w    = BufferUtils.createIntBuffer(1);
        IntBuffer h    = BufferUtils.createIntBuffer(1);
        IntBuffer c = BufferUtils.createIntBuffer(1);

        // Use info to read image metadata without decoding the entire image.
        if (!stbi_info_from_memory(imageBuffer, w, h, c)) {
            throw new RuntimeException("Failed to read image information: " + stbi_failure_reason());
        }
  
//        System.out.println("Image width: " + w.get(0));
//        System.out.println("Image height: " + h.get(0));
//        System.out.println("Image components: " + c.get(0));
//        System.out.println("Image HDR: " + stbi_is_hdr_from_memory(imageBuffer));

        // Decode the image
        ByteBuffer image = stbi_load_from_memory(imageBuffer, w, h, c, 0);
        if (image == null) {
            throw new RuntimeException("Failed to load image: " + stbi_failure_reason());
        }
        
        return image;
	}

    public static float[] loadImageToFloatArray(String file, int width, int height){
        ByteBuffer imageBuffer;
        try {
            imageBuffer = ioResourceToByteBuffer(file, width * height);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        IntBuffer w    = BufferUtils.createIntBuffer(1);
        IntBuffer h    = BufferUtils.createIntBuffer(1);
        IntBuffer c = BufferUtils.createIntBuffer(1);

        // Use info to read image metadata without decoding the entire image.
        if (!stbi_info_from_memory(imageBuffer, w, h, c)) {
            throw new RuntimeException("Failed to read image information: " + stbi_failure_reason());
        }

        // Decode the image
        ByteBuffer image = stbi_load_from_memory(imageBuffer, w, h, c, 0);
        if (image == null) {
            throw new RuntimeException("Failed to load image: " + stbi_failure_reason());
        }

        FloatBuffer fb = FloatBuffer.allocate(image.capacity());
        fb.put(image.asFloatBuffer());
        fb.rewind();
        return fb.array();
    }
	
	public static ByteBuffer ioResourceToByteBuffer(String resource, int bufferSize) throws IOException {
        ByteBuffer buffer;

        Path path = Paths.get(resource);
        if (Files.isReadable(path)) {
            try (SeekableByteChannel fc = Files.newByteChannel(path)) {
                buffer = BufferUtils.createByteBuffer((int)fc.size() + 1);
                while (fc.read(buffer) != -1) {
                    ;
                }
            }
        } else {
            try (
                    InputStream source = Thread.class.getClassLoader().getResourceAsStream(resource);
                    ReadableByteChannel rbc = Channels.newChannel(source)
                ) {
                    buffer = BufferUtils.createByteBuffer(bufferSize);

                    while (true) {
                        int bytes = rbc.read(buffer);
                        if (bytes == -1) {
                            break;
                        }
                        if (buffer.remaining() == 0) {
                            buffer = resizeBuffer(buffer, buffer.capacity() * 2);
                        }
                    }
                }
            }

            buffer.flip();
            return buffer;
    }
	
	private static ByteBuffer resizeBuffer(ByteBuffer buffer, int newCapacity) {
        ByteBuffer newBuffer = BufferUtils.createByteBuffer(newCapacity);
        buffer.flip();
        newBuffer.put(buffer);
        return newBuffer;
	}
}
