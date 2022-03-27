# Java-leven

Dual Contouring Chunking LODs with seams

Nick Gildea Dual Contouring https://github.com/nickgildea/leven implementation in Java LWJGL

CPU and OpenCL GPU implementations.

## Сhanges relative to the original solution
- The main version is on the CPU. I use multithreading to speed up the CPU version
- fixed the appearance of holes in the seams between chunks
- fixed seam and chunk mesh overlap at seam intersections
- no double call to generate mesh for rendering and mesh for collisions - mesh is generated only once
- faster selectActiveChunkNodes and ReleaseInvalidatedNodes (no need to recursively traverse the whole tree to invalidate chunks - which improved performance)
- Linear chunk's octree instead of Pointer Based octree. It used Morton codes.
- Chunks reduce instead of queue of CSG operations. Only the leaf chunk is edited. Rougher chunks have a simplification of the voxel lattice with Hermitian data.


## Features:
- Used JBullet physics engine (ray pick and collision detection);
- Many octree Dual contouring implementations:  
  Pointer based octree Dual contouring implementation,<br>
  Linear octree Dual contouring implementations: <br>
  - Simple implementation, in series steps to calculate leafs data.
  - Nick Gildea Leven Dual contouring implementation translated to Java CPU realization - LevenLinearCPUOctreeImpl.java
  - Nick Gildea Leven OpenCL Dual contouring implementation. In this implementation LevenLinearGPUOctreeImpl.java I call OpenCL kernels in java .
- CSG operations (add/delete sphere/box brushes);  

## Build Instructions
please, perform
<br>
mvn clean install

The dependencies are:
  * Maven
  * Java 11
  * I used IntelliJ IDEA Community edition

<p float="left">
<img src="https://github.com/proton2/java-leven/tree/master/res/logo/screens/screen-01.png" width="270" />
<img src="https://github.com/proton2/java-leven/tree/master/res/logo/screens/screen-02.png" width="270" />
<img src="https://github.com/proton2/java-leven/tree/master/res/logo/screens/screen-06.png" width="270" />
</p>

CSG operations:
<p float="left">
<img src="https://github.com/proton2/java-leven/tree/master/res/logo/screens/screen-04-csg.png" width="270" />
<img src="https://github.com/proton2/java-leven/tree/master/res/logo/screens/screen-05-csg.png" width="270" />
<img src="https://github.com/proton2/java-leven/tree/master/res/logo/screens/screen-07.png" width="270" />
</p>

W/S/A/D - forward, backward, left, right

Tab - enable/disable mouse rotation mode

R_Mouse - perform CSG brush to object

R - switch CSG brush / sphere / cube

M - switch CSG operation (add/delete mode)

[ - increase brush size

] - decrease brush size 

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

## interesting for research:

- Nick Gildea Dual Contouring
https://github.com/nickgildea/leven

- Dual contouring chunks<br>
https://github.com/yixinxie/TestDC/

- Dual contouring by SVD
  https://github.com/S-V/DualContouringTest
  
- SVD dissertation
  https://www.vstu.ru/upload/iblock/d6b/d6b39e94a85205ec02495af30f94605e.pdf