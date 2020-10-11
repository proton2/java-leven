package dc.impl.opencl;

public enum KernelNames {
    SCAN("opencl/scan.cl"),
    DENSITY("opencl/density_simplex.cl"),
    FIND_DEFAULT_EDGES("opencl/findDefaultEdges.cl");

    public String getName() {
        return name;
    }

    private final String name;

    KernelNames(String s) {
        name = s;
    }
}
