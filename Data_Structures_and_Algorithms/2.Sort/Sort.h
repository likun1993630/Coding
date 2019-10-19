#ifndef SORT_H
#define SORT_H

#include <algorithm>

using namespace std;

// 选择排序
template<typename T>
void selectionSort(T arr[], int n){

    for(int i = 0 ; i < n ; i ++){

        int minIndex = i;
        for( int j = i + 1 ; j < n ; j ++ )
            if( arr[j] < arr[minIndex] )
                minIndex = j;

        swap( arr[i] , arr[minIndex] );
    }
}

// 插入排序
template<typename T>
void insertionSort(T arr[], int n){
    for( int i = 1 ; i < n ; i ++ ) {
        // 寻找元素arr[i]合适的插入位置
        for( int j = i ; j > 0 && arr[j] < arr[j-1] ; j -- )
            swap( arr[j] , arr[j-1] );
    }
    return;
}

// 优化的插入排序
template<typename T>
void insertionSortOp(T arr[], int n){

    for( int i = 1 ; i < n ; i ++ ) {
        T e = arr[i];
        int j; // j保存元素e应该插入的位置
        for (j = i; j > 0 && arr[j-1] > e; j--)
            arr[j] = arr[j-1];
        arr[j] = e;
    }

    return;
}

// 冒泡排序写法1
template<typename T>
void bubbleSort(T arr[], int n){
	for (int i = 0; i < n-1; i++)
	{
		for(int j=0; j<n-i-1; j++)
			if(arr[j]>arr[j+1])
				swap(arr[j],arr[j+1]);
	}
}

// 冒泡排序写法2
template<typename T>
void bubbleSort2( T arr[] , int n){
    bool swapped;
    do{
        swapped = false;
        for( int i = 1 ; i < n ; i ++ )
            if( arr[i-1] > arr[i] ){
                swap( arr[i-1] , arr[i] );
                swapped = true;
            }
        // 优化, 每一趟Bubble Sort都将最大的元素放在了最后的位置
        // 所以下一次排序, 最后的元素可以不再考虑
        n --;
    }while(swapped);
}

// 优化的冒泡排序
template<typename T>
void bubbleSortOp( T arr[] , int n){
    int newn; // 使用newn进行优化
    do{
        newn = 0;
        for( int i = 1 ; i < n ; i ++ )
            if( arr[i-1] > arr[i] ){
                swap( arr[i-1] , arr[i] );
                // 记录最后一次的交换位置,
                // 在此之后的元素在下一轮扫描中均不考虑
                newn = i;
            }
        n = newn;
    }while(newn > 0);
}

// ×××自己写的×××  优化的希尔排序
template<typename T>
void shellSort(T arr[], int n){
	// 计算 increment sequence: 1, 4, 13, 40, 121, 364, 1093...
    int h = 1;
    while( h < n/3 )
        h = 3 * h + 1;

	while(h >= 1){
		int b = 0;
		while(b < h){
			for (int i = b+h; i < n; i = i+h)
			{	
				T min = arr[i];
				int j;
				for(j=i; j>b && arr[j-h]>min; j = j-h)
					arr[j]=arr[j-h];
				arr[j] = min;
			}
			b++;
		}
		h = h/3;
	}
}

// 优化的希尔排序
template<typename T>
void shellSort2(T arr[], int n){

    // 计算 increment sequence: 1, 4, 13, 40, 121, 364, 1093...
    int h = 1;
    while( h < n/3 )
        h = 3 * h + 1;

    while( h >= 1 ){

        // h-sort the array
        for( int i = h ; i < n ; i ++ ){

            // 对 arr[i], arr[i-h], arr[i-2*h], arr[i-3*h]... 使用插入排序
            T e = arr[i];
            int j;
            for( j = i ; j >= h && e < arr[j-h] ; j -= h )
                arr[j] = arr[j-h];
            arr[j] = e;
        }

        h /= 3;
    }
}

#endif
