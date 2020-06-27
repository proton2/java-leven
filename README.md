# Java-leven

Dual Contouring Chunking LODs with seams

Nick Gildea Dual Contouring https://github.com/nickgildea/leven implementation in Java LWJGL

The work has just begun and the project is still raw. Only the CPU implementation.

Features:
- Pointer based octree dual contouring;
- Linear octree dual contouring (Simple implementation, in series steps to calculate leafs data);

## Usage

```java
public class ChunkOctreeWrapper extends GameObject {
 
    // Uncomment necessary implementation in constructor
    public ChunkOctreeWrapper() {
        chunkOctree = new ChunkOctree(new SimpleLinearOctreeImpl());
        //chunkOctree = new ChunkOctree(new PointerBasedOctreeImpl());

        rootChunk = chunkOctree.buildChunkOctree();
    }
}
```

<img src="res/logo/screens/screen-01.png" width="800" />
<img src="res/logo/screens/screen-02.png" width="800" />

W/S/A/D - forward, backward, left, right

F1 - solid / wireframe

F2 - Show chunks octree bounds

F3 - enable / disable frustum culling

F4 - show bounds of the seam octree nodes 

middle mouse - camera walking

Use:

Oreon Engine (Java - OpenGL/Vulkan) https://github.com/fynnfluegge/oreon-engine