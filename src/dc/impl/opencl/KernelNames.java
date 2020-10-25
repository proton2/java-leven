package dc.impl.opencl;

public enum KernelNames {
    SCAN("opencl/scan.cl"),
    DENSITY("opencl/density_simplex.cl"),
    FIND_DEFAULT_EDGES("opencl/findDefaultEdges.cl"),
    NVIDIA_SCAN("opencl/nvidia-scan.cl"),
    CUCKOO("opencl/cuckoo.cl"),
    TEST_CUCKOO("opencl/hash_test.cl"),
    OCTREE("opencl/octree.cl");

    public String getName() {
        return name;
    }

    private final String name;

    KernelNames(String s) {
        name = s;
    }
}
