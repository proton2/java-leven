package test;

public class Array3DMultiThreadTest {
    public static void main(String[] args) {
        int size = 33;
        int availableProcessors = 4;
        int threadBound = (size * size * size) / availableProcessors;

        for (int i = 0; i < availableProcessors; i++) {
            int from = i * threadBound;
            int to = from + threadBound;
            boolean last = (i == availableProcessors - 1 && to <= (size * size * size) - 1);

            int it = from;
            while (it < (last ? size * size * size : to)) {
                int x = it % size;
                int y = (it / size) % size;
                int z = (it / size / size);
                System.out.println(x + " " + y + " " + z);

                if (x + 2 < size) {
                    it = it+2;
                } else if (y + 2 < size) {
                    it = z * size * size + (y+ 2) * size;
                } else {
                    it = (z + 2) * size * size;
                }
            }
        }
    }
}
