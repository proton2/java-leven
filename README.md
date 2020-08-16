# Java-leven

Dual Contouring Chunking LODs with seams

Nick Gildea Dual Contouring https://github.com/nickgildea/leven implementation in Java LWJGL

The work has just begun and the project is still raw. Only the CPU implementation.

Features:
- Pointer based octree Dual contouring implementation;
- Linear octree Dual contouring implementations: <br>
  - Simple implementation, in series steps to calculate leafs data.
  - Transition implementation Linear octree dual contouring between simple implementation and NickGildea OpenCL DC implementation.
  - Nick Gildea Leven OpenCL kernels Dual contouring implementation translated to Java.

## Build Instructions
The dependencies are:
  * Maven
  * Java 11
  * I used IntelliJ IDEA Community edition
 

## Usage

```java
public class ChunkOctreeWrapper extends GameObject {
 
    // Uncomment necessary implementation in constructor
    public ChunkOctreeWrapper() {
        //chunkOctree = new ChunkOctree(new PointerBasedOctreeImpl());
        chunkOctree = new ChunkOctree(new SimpleLinearOctreeImpl());
        //chunkOctree = new ChunkOctree(new TransitionLinearOctreeImpl());
        //chunkOctree = new ChunkOctree(new LevenLinearOctreeImpl());

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

<br>

Other interesting implementations for research:

- Dual contouring in Unity. Chunks and compute shaders using<br>
https://github.com/Colt-Zero/DualContouringGPU

- Dual contouring chunks<br>
https://github.com/yixinxie/TestDC/