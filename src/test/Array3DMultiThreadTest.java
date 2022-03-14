package test;

import core.math.Vec3i;

import java.util.ArrayList;

public class Array3DMultiThreadTest {
    public static void main(String[] args) {
        cicleTest();
    }


    private static void cicleTest(){
        int size = 33;
        int availableProcessors = 6;
        int threadBound = size / availableProcessors;

        ArrayList<Vec3i> threadArr = new ArrayList<>();
        for (int i = 0; i < availableProcessors; i++) {
            int from = i * threadBound;
            int to = (i == availableProcessors - 1) ? size : from + threadBound;

            for (int z = from + from % 2; z < to; z +=2) {
                for (int y = 0; y < size; y += 2) {
                    for (int x = 0; x < size; x += 2) {
                        threadArr.add(new Vec3i(x, y, z));
                    }
                }
            }
        }

        ArrayList<Vec3i> arr = new ArrayList<>();
        for(int z = 0; z < size; z += 2 ) {
            for (int y = 0; y < size; y += 2) {
                for (int x = 0; x < size; x += 2) {
                    arr.add(new Vec3i(x, y, z));
                }
            }
        }

        boolean equalsFlag = true;
        for (int i=0; i<threadArr.size(); i++){
            if(!threadArr.get(i).equals(arr.get(i))){
                System.out.println("array element not equals in index " + i);
                equalsFlag = false;
            }
        }
        if(equalsFlag){
            System.out.println("array element is equals");
        }
    }

    private static void test1(){
        int size = 33;
        int availableProcessors = 4;
        int threadBound = (size * size * size) / availableProcessors;

        ArrayList<Vec3i> threadArr = new ArrayList<>(size * size * size);
        for (int i = 0; i < availableProcessors; i++) {
            int from = i * threadBound;
            int to = from + threadBound;
            boolean last = (i == availableProcessors - 1 && to <= (size * size * size) - 1);

            int it = from;
            while (it < (last ? size * size * size : to)) {
                int x = it % size;
                int y = (it / size) % size;
                int z = (it / size / size);
                threadArr.add(new Vec3i(x, y, z));

                if (x + 2 < size) {
                    it = it+2;
                } else if (y + 2 < size) {
                    it = z * size * size + (y+ 2) * size;
                } else {
                    it = (z + 2) * size * size;
                }
            }
        }

        ArrayList<Vec3i> arr = new ArrayList<>(size * size * size);
        for(int z = 0; z < size; z += 2 ) {
            for (int y = 0; y < size; y += 2) {
                for (int x = 0; x < size; x += 2) {
                    arr.add(new Vec3i(x, y, z));
                }
            }
        }

        boolean equalsFlag = true;
        for (int i=0; i<threadArr.size(); i++){
            if(!threadArr.get(i).equals(arr.get(i))){
                System.out.println("array element not equals in index " + i);
                equalsFlag = false;
            }
        }
        if(equalsFlag){
            System.out.println("array element is equals");
        }
    }
}
