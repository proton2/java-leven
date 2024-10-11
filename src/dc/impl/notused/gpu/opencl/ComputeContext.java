package dc.impl.notused.gpu.opencl;

import java.nio.IntBuffer;

public class ComputeContext {
    private final long clDevice;
    private final long clQueue;
    private final long clContext;
    private final IntBuffer errcode_ret;
    public final int defaultMaterial = 0;

    public ComputeContext(long clDevice, long clQueue, long clContext, IntBuffer errcode_ret) {
        this.clDevice = clDevice;
        this.clQueue = clQueue;
        this.clContext = clContext;
        this.errcode_ret = errcode_ret;
    }

    public long getClDevice() {
        return clDevice;
    }

    public long getClQueue() {
        return clQueue;
    }

    public long getClContext() {
        return clContext;
    }

    public IntBuffer getErrcode_ret() {
        return errcode_ret;
    }
}
