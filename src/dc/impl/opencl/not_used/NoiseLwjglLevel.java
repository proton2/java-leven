package dc.impl.opencl.not_used;

//import dc.impl.opencl.ComputeContext;
//import dc.impl.opencl.OCLUtils;
//import org.lwjgl.PointerBuffer;
//import org.lwjgl.opencl.CL10;
//import org.lwjgl.opencl.CL12;
//import org.lwjgl.opencl.CLImageFormat;
//
//import javax.imageio.ImageIO;
//import java.awt.image.BufferedImage;
//import java.io.ByteArrayInputStream;
//import java.io.IOException;
//import java.io.InputStream;
//import java.nio.ByteBuffer;
//import java.nio.CharBuffer;
//import java.nio.charset.Charset;
//import java.nio.charset.StandardCharsets;
//import java.util.Arrays;
//import java.util.Collections;
//import java.util.List;
//import java.util.Random;
//import java.util.stream.Collectors;
//
//import static org.lwjgl.opencl.CL10.*;

public class NoiseLwjglLevel {
    public NoiseLwjglLevel(){}
/*
    private int p[]= {151,160,137,91,90,15,
            131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
            190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
            88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
            77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
            102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
            135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
            5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
            223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
            129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
            251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
            49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
            138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180,
            131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
            190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
            88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
            77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
            102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
            135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
            5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
            223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
            129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
            251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
            49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
            138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180};

    private int perm[] = new int[512];
    {
        for (int i = 0; i < 512; i++) {
            perm[i] = p[i & 255];
        }
    }

//       These are Ken Perlin's proposed gradients for 3D noise. I kept them for
//       better consistency with the reference implementation, but there is really
//       no need to pad this to 16 gradients for this particular implementation.
//       If only the "proper" first 12 gradients are used, they can be extracted
//       from the grad4[][] array: grad3[i][j] == grad4[i*2][j], 0<=i<=11, j=0,1,2
    private int[][] grad3 = {{0,1,1},{0,1,-1},{0,-1,1},{0,-1,-1},
        {1,0,1},{1,0,-1},{-1,0,1},{-1,0,-1},
        {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0}, // 12 cube edges
        {1,0,-1},{-1,0,-1},{0,-1,1},{0,1,1}}; // 4 more to make 16

    private int NoiseHash(int x, int y, int seed) {
	    int key = ((x << 24) | (y << 16)) ^ seed;
        //unsigned char* keyBytes = (unsigned char*)&key;
        char[] keyBytes = String.valueOf(key).toCharArray();
        int hash = 0;
        for (int i = 0; i < 4; i++) {
            hash += keyBytes[i];
            hash += (hash << 10);
            hash ^= (hash >> 6);
        }
        hash += (hash << 3);
        hash ^= (hash >> 11);
        hash += (hash << 15);
        return hash;
    }

    public long CreateNoisePermutationLookupImage(int seed, ComputeContext computeContext) {
        char[] charBuffer = createPixelsArray(seed);
        long image = processImage2D(charBuffer, computeContext);
        //writeImage(computeContext.getClQueue(), byteBuffer, new long[]{0,0,0}, new long[]{256, 256, 1}, 0, 0);
        return image;
    }

    private char[] createPixelsArray(int seed){
        List<Integer> shuffledPerm = Arrays.stream(perm).boxed().collect(Collectors.toList());
        Collections.shuffle(shuffledPerm);
        seed = new Random().nextInt(seed);

        char[] pixels = new char[256 * 256 * 4];
        for (int i = 0; i < 256; i++) {
            for (int j = 0; j < 256; j++) {
                int offset = ((i * 256) + j) * 4;
                int value = shuffledPerm.get(NoiseHash(i, j, seed) & 0x1ff);
                pixels[offset + 0] = (char) (grad3[value & 0x0f][0] * 64 + 64);
                pixels[offset + 1] = (char) (grad3[value & 0x0f][1] * 64 + 64);
                pixels[offset + 2] = (char) (grad3[value & 0x0f][2] * 64 + 64);
                pixels[offset + 3] = (char) value;
            }
        }
        return pixels;
    }

    byte[] toBytes(char[] chars) {
        CharBuffer charBuffer = CharBuffer.wrap(chars);
        ByteBuffer byteBuffer = Charset.forName("UTF-8").encode(charBuffer);
        byte[] bytes = Arrays.copyOfRange(byteBuffer.array(), byteBuffer.position(), byteBuffer.limit());
        Arrays.fill(byteBuffer.array(), (byte) 0); // clear sensitive data
        return bytes;
    }

    BufferedImage getBufferedImage (byte[] imageInByte){
        InputStream in = new ByteArrayInputStream(imageInByte);
        BufferedImage bImageFromConvert = null;
        try {
            bImageFromConvert = ImageIO.read(in);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return bImageFromConvert;
    }

    private long processImage2D(char[] pixels, ComputeContext computeContext){
        CharBuffer charBuffer = CharBuffer.wrap(pixels);
        ByteBuffer encode = StandardCharsets.UTF_8.encode(charBuffer);
        CLImageFormat f = null;
        long result;
        try {
            f = CLImageFormat.malloc();
            f.image_channel_data_type(CL10.CL_UNORM_INT8);
            f.image_channel_order(CL10.CL_RGBA);
            result = CL12.clCreateImage2D(computeContext.getClContext(), CL_MEM_READ_ONLY| CL_MEM_USE_HOST_PTR, f,
                    256, 256, 256 * 4, encode, computeContext.getErrcode_ret());
            OCLUtils.checkCLError(computeContext.getErrcode_ret());
        } finally {
            if (f != null) {
                f.free();
            }
        }
        return result;
    }

    private long processEmptyImage(ComputeContext computeContext){
        CLImageFormat f = null;
        long image;
        try {
            f = CLImageFormat.malloc();
            f.image_channel_data_type(CL10.CL_UNORM_INT8);
            f.image_channel_order(CL10.CL_RGBA);
            ByteBuffer hostPtr = null;
            image = CL10.clCreateImage2D(computeContext.getClContext(), CL_MEM_READ_ONLY, f,
                    256, 256, 0, hostPtr, computeContext.getErrcode_ret());
            OCLUtils.checkCLError(computeContext.getErrcode_ret());
        } finally {
            if (f != null) {
                f.free();
            }
        }
        return image;
    }

    public void writeImage(long q, ByteBuffer dest, long sourceImage, long[] origin, long[] region, long rowPitch, long slicePitch) {
        if (origin.length!=3 || region.length!=3) {
            throw new IllegalArgumentException("origin and region must both be arrays of length 3");
        }
        PointerBuffer pointerBuffer1 = PointerBuffer.allocateDirect(4);
        PointerBuffer pointerBuffer2 = PointerBuffer.allocateDirect(4);
        pointerBuffer1.rewind();
        pointerBuffer2.rewind();
        pointerBuffer1.put(origin).position(0);
        pointerBuffer2.put(region).position(0);
        int ret = CL10.clEnqueueWriteImage(q, sourceImage, true,
                pointerBuffer1, pointerBuffer2,
                rowPitch, slicePitch, dest, null, null);
        OCLUtils.checkCLError(ret);
    }
 */
}
