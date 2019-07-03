# 复合类型

复合类型是基于基本整型和浮点型创建的。

## 数组（array）

数组的通用声明：
`typeName arrayName[arraySize];`

arraySize 指定元素数目，必须为整型常数或const值，也可以式常量表达式（如 `8*sizeof(int)`）,即其中所有的值在编译时都是已知的。具体说，arraySize不能是变量，变量的值时在程序运行时设置的。（可使用new来避开这个限制）。

例：
```cpp
short months[12];
```
- 声明了一个名字为months的数组，该数组有12个元素，每个元素都可以存储一个short类型的值。
- 使用months[0]时months数组的第一个元素，months[11] 为最后一个元素。

> 注意编译器不会检查使用的下标是否有效，例如，如果将一个赋值给不存在的元素months[100],编译器并不会指出错误，但当程序运行后，程序会出问题。

数组示意图：

![](./res/4.array7.png)


```cpp
// arrayone.cpp -- small arrays of integers
#include <iostream>
int main()
{
    using namespace std;
    int yams[3];    // creates array with three elements
    yams[0] = 7;    // assign value to first element
    yams[1] = 8;
    yams[2] = 6;

    int yamcosts[3] = {20, 30, 5}; //更快捷的初始化数组的方式
    cout << "Total yams = ";
    cout << yams[0] + yams[1] + yams[2] << endl;
    cout << "The package with " << yams[1] << " yams costs ";
    cout << yamcosts[1] << " cents per yam.\n";
    int total = yams[0] * yamcosts[0] + yams[1] * yamcosts[1];
    total = total + yams[2] * yamcosts[2];
    cout << "The total yam expense is " << total << " cents.\n";

    cout << "\nSize of yams array = " << sizeof yams;
    cout << " bytes.\n";
    cout << "Size of one element = " << sizeof yams[0];
    cout << " bytes.\n";
    return 0; 
}
```
结果：
```
Total yams = 21
The package with 8 yams costs 30 cents per yam.
The total yam expense is 410 cents.

Size of yams array = 12 bytes.
Size of one element = 4 bytes.
```
> 如果没有初始化函数中定义的数组，则其他元素将是不确定的，这意味着元素的值为以前驻留在该内存单元中的值。

> yams是一个数组，而yams[1] 只是一个int变量。

### 数组初始化规则

只有在定义数组时才能使用初始化，此后就不能使用了，也不能将一个数组赋给另一个数组：
```cpp
int cards[4] = {3, 6, 8, 10};  //允许
int hand[4]; //允许
hand[4] = {5, 6, 7, 9}; // 不允许
hand = cards; //不允许

//初始化可以不初始化全部元素，此时编译器就会把其他元素设置为0.
float hotelTips[5] = {5.0, 2.5}; 

//初始化时当[]括号内为空时，编译器将计算元素个数，用来初始化数组，尽量避免这种做法
short things[] = {1, 5, 3, 8};
```

如果想将数组中的所有元素都初始化为0，只需要显示的将第一个元素初始化为0，然后让编译器将其他元素都初始化为0：
```cpp
long totals[500] = {0};
```

### C++11数组初始话方法
- 初始化数组时，可以省略等号
- 可以不在大括号内包含任何东西，将把所以有元素都设置为零
- 列表初始化禁止缩窄转换

```cpp
double earnings[4] {1, 2, 3, 4}; //c++11新增

unsigned int counts[10] = {}; //所有元素置零
float balance[100] {}; //所有元素置零

long plifs[] = {25, 92, 3.0}; //不允许
char slifs[4] {'h', 'i', 1122011, '\0'}; //不允许
char tlifs[4] {'h', 'i', 112, '\0'}; //允许
```

> C++标准模板库（STL）提供了一种数组替代品--模板类vector； C++11 新增了模板类array。

## 字符串

