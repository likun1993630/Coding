#include <iostream>
#include "Sort.h"
#include "SortTestHelper.h"

using namespace std;

// 比较SelectionSort, InsertionSort和BubbleSort和ShellSort四种排序算法的性能效率
// ShellSort是这四种排序算法中性能最优的排序算法

int main() {

    int n = 20000;

    // 测试1 一般测试
    cout<<"Test for random array, size = "<<n<<", random range [0, "<<n<<"]"<<endl;

    int *arr1 = SortTestHelper::generateRandomArray(n,0,n);
    int *arr2 = SortTestHelper::copyIntArray(arr1, n);
    int *arr3 = SortTestHelper::copyIntArray(arr1, n);
    int *arr4 = SortTestHelper::copyIntArray(arr1, n);
    int *arr5 = SortTestHelper::copyIntArray(arr1, n);
    int *arr6 = SortTestHelper::copyIntArray(arr1, n);
    int *arr7 = SortTestHelper::copyIntArray(arr1, n);
    int *arr8 = SortTestHelper::copyIntArray(arr1, n);

    SortTestHelper::testSort("Selection Sort", selectionSort, arr1, n);
    SortTestHelper::testSort("Insertion Sort", insertionSort, arr2, n);
    SortTestHelper::testSort("Insertion Sort Op", insertionSortOp, arr3, n);
    SortTestHelper::testSort("Bubble Sort", bubbleSort, arr4, n);
    SortTestHelper::testSort("Bubble Sort2", bubbleSort2, arr5, n);
    SortTestHelper::testSort("Bubble SortOp", bubbleSortOp, arr6, n);
    SortTestHelper::testSort("Shell Sort", shellSort, arr7, n);
    SortTestHelper::testSort("Shell Sort2", shellSort2, arr8, n);

    delete[] arr1;
    delete[] arr2;
    delete[] arr3;
    delete[] arr4;
    delete[] arr5;
    delete[] arr6;
    delete[] arr7;
    delete[] arr8;

    cout<<endl;


    // 测试2 有序性更强的测试
    cout<<"Test for more ordered random array, size = "<<n<<", random range [0, 3]"<<endl;
    arr1 = SortTestHelper::generateRandomArray(n,0,3);
    arr2 = SortTestHelper::copyIntArray(arr1, n);
    arr3 = SortTestHelper::copyIntArray(arr1, n);
    arr4 = SortTestHelper::copyIntArray(arr1, n);
    arr5 = SortTestHelper::copyIntArray(arr1, n);
    arr6 = SortTestHelper::copyIntArray(arr1, n);
    arr7 = SortTestHelper::copyIntArray(arr1, n);
    arr8 = SortTestHelper::copyIntArray(arr1, n);

    SortTestHelper::testSort("Selection Sort", selectionSort, arr1, n);
    SortTestHelper::testSort("Insertion Sort", insertionSort, arr2, n);
    SortTestHelper::testSort("Insertion Sort Op", insertionSortOp, arr3, n);
    SortTestHelper::testSort("Bubble Sort", bubbleSort, arr4, n);
    SortTestHelper::testSort("Bubble Sort2", bubbleSort2, arr5, n);
    SortTestHelper::testSort("Bubble SortOp", bubbleSortOp, arr6, n);
    SortTestHelper::testSort("Shell Sort", shellSort, arr7, n);
    SortTestHelper::testSort("Shell Sort2", shellSort2, arr8, n);

    delete[] arr1;
    delete[] arr2;
    delete[] arr3;
    delete[] arr4;
    delete[] arr5;
    delete[] arr6;
    delete[] arr7;
    delete[] arr8;

    cout<<endl;


    // 测试3 测试近乎有序的数组
    int swapTimes = 100;

    cout<<"Test for nearly ordered array, size = "<<n<<", swap time = "<<swapTimes<<endl;

    arr1 = SortTestHelper::generateNearlyOrderedArray(n, swapTimes);
    arr2 = SortTestHelper::copyIntArray(arr1, n);
    arr3 = SortTestHelper::copyIntArray(arr1, n);
    arr4 = SortTestHelper::copyIntArray(arr1, n);
    arr5 = SortTestHelper::copyIntArray(arr1, n);
    arr6 = SortTestHelper::copyIntArray(arr1, n);
    arr7 = SortTestHelper::copyIntArray(arr1, n);
    arr8 = SortTestHelper::copyIntArray(arr1, n);

    SortTestHelper::testSort("Selection Sort", selectionSort, arr1, n);
    SortTestHelper::testSort("Insertion Sort", insertionSort, arr2, n);
    SortTestHelper::testSort("Insertion Sort Op", insertionSortOp, arr3, n);
    SortTestHelper::testSort("Bubble Sort", bubbleSort, arr4, n);
    SortTestHelper::testSort("Bubble Sort2", bubbleSort2, arr5, n);
    SortTestHelper::testSort("Bubble SortOp", bubbleSortOp, arr6, n);
    SortTestHelper::testSort("Shell Sort", shellSort, arr7, n);
    SortTestHelper::testSort("Shell Sort2", shellSort2, arr8, n);

    delete[] arr1;
    delete[] arr2;
    delete[] arr3;
    delete[] arr4;
    delete[] arr5;
    delete[] arr6;
    delete[] arr7;
    delete[] arr8;

    cout<<endl;


    // 测试4 测试完全有序的数组
    // 对于完全有序的数组，冒泡排序法也将成为O(n)级别的算法
    swapTimes = 0;
    n = 10000000;    // 由于插入排序法和冒泡排序法在完全有序的情况下都将成为O(n)算法
                     // 所以我们的测试数据规模变大，为1000,0000
    cout<<"Test for ordered array, size = " << n << endl;

    arr1 = SortTestHelper::generateNearlyOrderedArray(n, swapTimes);
    arr2 = SortTestHelper::copyIntArray(arr1, n);
    arr3 = SortTestHelper::copyIntArray(arr1, n);
    arr4 = SortTestHelper::copyIntArray(arr1, n);
    arr5 = SortTestHelper::copyIntArray(arr1, n);
    arr6 = SortTestHelper::copyIntArray(arr1, n);
    arr7 = SortTestHelper::copyIntArray(arr1, n);
    arr8 = SortTestHelper::copyIntArray(arr1, n);

    // 在这种情况下，不再测试选择排序算法
    //SortTestHelper::testSort("Selection Sort", selectionSort, arr1, n);
    SortTestHelper::testSort("Insertion Sort", insertionSort, arr2, n);
    SortTestHelper::testSort("Insertion Sort Op", insertionSortOp, arr3, n);
    //SortTestHelper::testSort("Bubble Sort", bubbleSort, arr4, n); //不知为何，非常慢
    SortTestHelper::testSort("Bubble Sort2", bubbleSort2, arr5, n);
    SortTestHelper::testSort("Bubble SortOp", bubbleSortOp, arr6, n);
    SortTestHelper::testSort("Shell Sort", shellSort, arr7, n);
    SortTestHelper::testSort("Shell Sort2", shellSort2, arr8, n);

    delete[] arr1;
    delete[] arr2;
    delete[] arr3;
    delete[] arr4;
    delete[] arr5;
    delete[] arr6;
    delete[] arr7;
    delete[] arr8;

    cout<<endl;

    return 0;
}