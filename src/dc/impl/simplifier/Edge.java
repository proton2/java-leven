package dc.impl.simplifier;

public class Edge{
    int min;
    int max;
    long idx=0;

    public Edge(int minIndex, int maxIndex) {
        this.min = minIndex;
        this.max = maxIndex;
    }
}
