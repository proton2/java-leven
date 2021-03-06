# Java-leven

Dual Contouring Chunking LODs with seams

Nick Gildea Dual Contouring https://github.com/nickgildea/leven implementation in Java LWJGL

CPU and OpenCL GPU implementations.

## Features:
- Used JBullet physics engine (ray pick and collision detection);
- Many octree Dual contouring implementations:  
  Pointer based octree Dual contouring implementation,<br>
  Linear octree Dual contouring implementations: <br>
  - Simple implementation, in series steps to calculate leafs data.
  - Transition implementation Linear octree dual contouring between simple implementation and NickGildea OpenCL DC implementation - TransitionLinearOctreeImpl.java. This is my favorite implementation.
  - Nick Gildea Leven Dual contouring implementation translated to Java CPU realization - LevenLinearCPUOctreeImpl.java
  - Nick Gildea Leven OpenCL Dual contouring implementation. In this implementation LevenLinearGPUOctreeImpl.java I call OpenCL kernels in java .

## Build Instructions
please, perform
<br>
mvn clean install

The dependencies are:
  * Maven
  * Java 11
  * I used IntelliJ IDEA Community edition
 

## Usage

```java
public class ChunkOctreeWrapper extends GameObject {
 
    // Uncomment necessary implementation in constructor
    public ChunkOctreeWrapper() {
        //VoxelOctree voxelOctree = new PointerBasedOctreeImpl(true, meshGenCtx);
        //VoxelOctree voxelOctree = new SimpleLinearOctreeImpl(meshGenCtx);
        VoxelOctree voxelOctree = new TransitionLinearOctreeImpl(meshGenCtx);
        //VoxelOctree voxelOctree = new LevenLinearCPUOctreeImpl(meshGenCtx);
        //VoxelOctree voxelOctree = new LevenLinearGPUOctreeImpl(kernelHolder, meshGenCtx, ctx);
        chunkOctree = new ChunkOctree(voxelOctree, meshGenCtx);
    }
}
```

<img src="res/logo/screens/screen-01.png" width="800" />
<img src="res/logo/screens/screen-02.png" width="800" />
OpenCL GPU implementation:
<img src="res/logo/screens/screen-03-opencl.png" width="800" />

W/S/A/D - forward, backward, left, right

F1 - solid / wireframe

F2 - Show chunks octree bounds

F3 - enable / disable frustum culling

F4 - show bounds of the seam octree nodes 

middle mouse with mouse move - camera walking (without change direction of movement)

right mouse - ray pick from mouse cursor to intersection point with landscape

arrows - change camera direction (with change direction of movement)
<br>
## Use:

JBullet - I use my modified version of JBullet https://github.com/proton2/jbullet

Oreon Engine (Java - OpenGL/Vulkan) https://github.com/fynnfluegge/oreon-engine

<br>

## Other interesting implementations for research:

- Nick Gildea Dual Contouring
https://github.com/nickgildea/leven

- Dual contouring in Unity. Chunks and compute shaders using<br>
https://github.com/Colt-Zero/DualContouringGPU

- Dual contouring chunks<br>
https://github.com/yixinxie/TestDC/