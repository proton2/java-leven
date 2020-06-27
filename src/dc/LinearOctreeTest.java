package dc;

import core.math.Vec3i;

import java.util.HashMap;
import java.util.Map;

import static dc.ChunkOctree.VOXELS_PER_CHUNK;
import static dc.ChunkOctree.log2;

public class LinearOctreeTest {
    static int MAX_OCTREE_DEPTH = log2(VOXELS_PER_CHUNK);
    static class LinearOctreeNode {
        private int nodeCode;
        private boolean childExists;

        public int getNodeCode() {
            return nodeCode;
        }

        public void setNodeCode(int nodeCode) {
            this.nodeCode = nodeCode;
        }

        public boolean isChildExists() {
            return childExists;
        }

        public void setChildExists(boolean childExists) {
            this.childExists = childExists;
        }
    }

    private Map<Integer, LinearOctreeNode> nodes;

    public LinearOctreeTest() {
        nodes = new HashMap<>();
    }

    public LinearOctreeNode getParent(int childCode){
        return nodes.get(childCode>>3);
    }

    public LinearOctreeNode getChild(int i, LinearOctreeNode node){
        int locCodeChild = getChildrenCode(i, node.getNodeCode());
        return getNode(locCodeChild);
    }

    public LinearOctreeNode getNode(int code){
        return nodes.get(code);
    }

    public static int getParentCode(int childCode){
        return childCode>>3;
    }

    public static int getChildrenCode(int i, int parentCode){
        return (parentCode<<3) | i;
    }

    public int getNodeTreeDepth(LinearOctreeNode node){
        return getMsb(node.getNodeCode())/3;
    }

    public void visitAll(LinearOctreeNode node) {
        for (int i=0; i<8; i++) {
            if (node.isChildExists()) {
                int locCodeChild = (node.getNodeCode()<<3)|i;
                LinearOctreeNode child = getNode(locCodeChild);
                visitAll(child);
            }
        }
    }

    private static int getMsb(int value) {
        for (int i = 0, maxBits = 31, test = ~(~0 >>> 1); 0 != test; ++i, test >>>= 1) {
            if (test == (value & test)) {
                return (maxBits - i);
            }
        }
        return -1;
    }

    public static Vec3i positionForCode(int code) {
        int nodeDepth = getMsb(code)/3;
        Vec3i pos = new Vec3i();
        for (int i = MAX_OCTREE_DEPTH - nodeDepth; i < MAX_OCTREE_DEPTH; i++) {
            int c = code & 7;
            code >>= 3;

            int x = (c >> 2) & 1;
            int y = (c >> 1) & 1;
            int z = (c >> 0) & 1;

            pos.x |= (x << i);
            pos.y |= (y << i);
            pos.z |= (z << i);
        }
        return pos;
    }

    public static int codeForPosition(Vec3i p, int nodeDepth) {
        int code = 1;
        for (int depth = MAX_OCTREE_DEPTH - 1; depth >= (MAX_OCTREE_DEPTH - nodeDepth); depth--) {
            int x = (p.x >> depth) & 1;
            int y = (p.y >> depth) & 1;
            int z = (p.z >> depth) & 1;
            int c = (x << 2) | (y << 1) | z;
            code = (code << 3) | c;
        }
        return code;
    }

    public int testSomeNodeDepth (int nodeCode){
        int clz = Integer.numberOfLeadingZeros(nodeCode & 0xFF) - 24;
        int msb = 32 - clz;
        int msb2 = 32 - Integer.numberOfLeadingZeros(nodeCode) - Integer.bitCount(nodeCode);
        int some = getMsb(nodeCode);
        int some2 = Integer.numberOfLeadingZeros(nodeCode);
        return some/3;
    }

    public static void main(String[] args) {
        Vec3i pos = new Vec3i(48, 0, 63);
        int code = codeForPosition(pos, MAX_OCTREE_DEPTH);
        Vec3i decodedPos = positionForCode(code);
        System.out.println(decodedPos);
        int parentCode = getParentCode(code);
        Vec3i parentDecodedPos = positionForCode(parentCode);
        int calculatedChildrenCode = getChildrenCode(1, parentCode);
        System.out.println(parentDecodedPos);
        int t = 19/8;
        int ost = 19-(8*t);
        System.out.println(ost);
    }
}
