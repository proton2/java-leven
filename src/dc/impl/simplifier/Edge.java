package dc.impl.simplifier;

public class Edge{
    private int min;
    private int max;
    private long idx;

    public Edge(int minIndex, int maxIndex) {
        this.min = minIndex;
        this.max = maxIndex;
        idx = ((long) max << 32) | min;
    }

    public int getMin() {
        return min;
    }

    public void setMin(int min) {
        this.min = min;
        idx = ((long) max << 32) | min;
    }

    public int getMax() {
        return max;
    }

    public void setMax(int max) {
        this.max = max;
        idx = ((long) max << 32) | min;
    }

    public long getIdx() {
        return idx;
    }
}
