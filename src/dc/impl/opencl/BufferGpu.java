package dc.impl.opencl;

public class BufferGpu {
    private long mem;
    private final String name;

    public String getName() {
        return name;
    }

    public BufferGpu(String name, long mem) {
        this.mem = mem;
        this.name = name;
    }

    public long getMem() {
        return mem;
    }

    public void setMem(long mem) {
        this.mem = mem;
    }
}
