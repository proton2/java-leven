package dc.impl;

import core.math.Vec3i;

// Morton codes for linear octrees.

/*
basics:
http://en.wikipedia.org/wiki/Z-order_curve
http://stackoverflow.com/questions/18529057/produce-interleaving-bit-patterns-morton-keys-for-32-bit-64-bit-and-128bit
http://stackoverflow.com/questions/1024754/how-to-compute-a-3d-morton-number-interleave-the-bits-of-3-ints
http://code.activestate.com/recipes/577558-interleave-bits-aka-morton-ize-aka-z-order-curve/
http://graphics.stanford.edu/~seander/bithacks.html#InterleaveBMN
http://dmytry.com/texts/collision_detection_using_z_order_curve_aka_Morton_order.html
http://www.forceflow.be/2013/10/07/morton-encodingdecoding-through-bit-interleaving-implementations/
https://www.fpcomplete.com/user/edwardk/revisiting-matrix-multiplication/part-1
https://www.fpcomplete.com/user/edwardk/revisiting-matrix-multiplication/part-2
Converting to and from Dilated Integers [2007]:
http://www.cs.indiana.edu/~dswise/Arcee/castingDilated-comb.pdf

a very good post:
http://asgerhoedt.dk/?p=276

It turns out we don't have to actually interleave the bits
if all we want is the ability to compare two keys as if they had been interleaved.
What we need to know is where the most significant difference between them occurs.
Given two halves of a key we can exclusive or them together to find the positions at which they differ.
*/

public class Morton3D {
    // from http://www.forceflow.be/2012/07/24/out-of-core-construction-of-sparse-voxel-octrees/
    // method to separate bits from a given integer 3 positions apart
    private static long Morton64_SpreadBits3(int n) {
        long x = n & 0x1fffff; // we only look at the first 21 bits
        x = (x | x << 32) & 0x1f00000000ffffL;  //         ___11111________________________________1111111111111111
        x = (x | x << 16) & 0x1f0000ff0000ffL;  //         ___11111________________11111111________________11111111
        x = (x | x << 8) & 0x100f00f00f00f00fL; // ___1________1111________1111________1111________1111____________
        x = (x | x << 4) & 0x10c30c30c30c30c3L; // ___1____11____11____11____11____11____11____11____11___1________
        x = (x | x << 2) & 0x1249249249249249L;
        return x;
    }

    /*
    https://fgiesen.wordpress.com/2009/12/13/decoding-morton-codes/
    x &= 0x55555555;                  // x = -f-e -d-c -b-a -9-8 -7-6 -5-4 -3-2 -1-0
    x = (x ^ (x >>  1)) & 0x33333333; // x = --fe --dc --ba --98 --76 --54 --32 --10
    x = (x ^ (x >>  2)) & 0x0f0f0f0f; // x = ---- fedc ---- ba98 ---- 7654 ---- 3210
    x = (x ^ (x >>  4)) & 0x00ff00ff; // x = ---- ---- fedc ba98 ---- ---- 7654 3210
    x = (x ^ (x >>  8)) & 0x0000ffff; // x = ---- ---- ---- ---- fedc ba98 7654 3210
    */
/// Contract bits along the 32-bit unsigned integer.
    private static int Morton32_CompactBits3(int x) {
        x &= 0x09249249; // 0b...01001001
        x = (x ^ (x >> 2)) & 0x030c30c3;
        x = (x ^ (x >> 4)) & 0x0300f00f;
        x = (x ^ (x >> 8)) & 0xff0000ff;
        x = (x ^ (x >> 16)) & 0x000003ff;
        return x;
    }//shrink/contract/compact

    private static int[] Morton32_Decode(int _code) {
        return new int[]{
                Morton32_CompactBits3(_code >> 0),
                Morton32_CompactBits3(_code >> 1),
                Morton32_CompactBits3(_code >> 2)
        };
    }

    // Interleaves coordinate bits to build a Morton code: spreads bits out to every-third bit.
    // Expands a 10-bit integer into 30 bits by inserting 2 zeros after each bit.
    // http://stackoverflow.com/a/18528775/1042102
    //......................9876543210
    //............98765..........43210
    //........987....56......432....10
    //......98..7..5..6....43..2..1..0
    //....9..8..7..5..6..4..3..2..1..0
    /// Dilate bits along the 32-bit unsigned integer.
    private static int Morton32_SpreadBits3(int x ) {
        x &= 0x3ff;	// zero out the upper 20 bits
        x = (x | x << 16) & 0x30000ff;	// 0b______11 ________ ________ 11111111
        x = (x | x << 8) & 0x300f00f;	// 0b______11 ________ 1111____ ____1111
        x = (x | x << 4) & 0x30c30c3;	// 0b______11 ____11__ __11____ 11____11
        x = (x | x << 2) & 0x9249249;	// 0b____1__1 __1__1__ 1__1__1_ _1__1__1
        return x;
    }

    public static long Morton32_Encode(int _x, int _y, int _z) {
        if ((_x > 1023) || ( _y > 1023 ) || ( _z > 1023)){
            throw new IllegalArgumentException("supports a maximum 3D resolution of 2^10 = 1024");
        }
        return (Morton32_SpreadBits3(_x) << 0)| (Morton32_SpreadBits3(_y) << 1)| (Morton32_SpreadBits3(_z) << 2);
    }

    private static long Morton64_Encode(int _x, int _y, int _z) {
        return (Morton64_SpreadBits3(_x) << 0) |
                (Morton64_SpreadBits3(_y) << 1) |
                (Morton64_SpreadBits3(_z) << 2);
    }

    private static int getNodeDepth(int size) {
        switch (size) {
            case 4096: return 0;
            case 2048: return 1;
            case 1024: return 2;
            case 512: return 3;
            case 256: return 4;
            case 128: return 5;
            case 64: return 6;
            case 32: return 7;
        }
        return 0;
    }

    public static long codeForPosition(Vec3i pos, int size, int halfWorldSize) {
        Vec3i localPos = pos.add(halfWorldSize).div(size);
        long M = Morton32_Encode(localPos.x, localPos.y, localPos.z);
        int depth = getNodeDepth(size);
        int indexOfDepthBit = depth * 3;
        long depthBitMask = (1L << indexOfDepthBit);
        return M | depthBitMask;
    }
}
