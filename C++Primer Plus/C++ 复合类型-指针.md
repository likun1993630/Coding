# 指针和自由存储空间

计算机程序在存储数据时必须跟踪3种基本属性：

- 信息存储在何处
- 存储的值为多少
- 存储的信息是什么类型

指针是一个变量，其存储的是值的地址，而不是值本身。

`&` 为取地址符，可以获取变量的地址，如home是一个变量`&home`是home的地址

`*` 运算符为间接值或解引用运算符。该运算符用于指针可以获取该地址存储的值，如 manly是一个指针，则manly表示的是一个地址，而`*manly`表示存储在该地址的值。

取地址实例：
```cpp
// address.cpp -- using the & operator to find addresses
#include <iostream>
int main()
{
    using namespace std;
    int donuts = 6;
    double cups = 4.5;

    cout << "donuts value = " << donuts;
    cout << " and donuts address = " << &donuts << endl;
    cout << "cups value = " << cups;
    cout << " and cups address = " << &cups << endl;
    return 0; 
}
```
结果：
```
donuts value = 6 and donuts address = 0x6dfeec
cups value = 4.5 and cups address = 0x6dfee0
```

> 可以看出0x6dfeec - 0x6dfee0 = 12，可知当前系统double类型使用了12个字节。

指针实例：
```cpp
// pointer.cpp -- our first pointer variable
#include <iostream>
int main()
{
    using namespace std;
    int updates = 6;        // declare a variable
    int * p_updates;        // declare pointer to an int

    p_updates = &updates;   // assign address of int to pointer

// express values two ways
    cout << "Values: updates = " << updates;
    cout << ", *p_updates = " << *p_updates << endl;

// express address two ways
    cout << "Addresses: &updates = " << &updates;
    cout << ", p_updates = " << p_updates << endl;

// use pointer to change value
    *p_updates = *p_updates + 1;
    cout << "Now updates = " << updates << endl;
    return 0; 
}
```
结果：
```
Values: updates = 6, *p_updates = 6
Addresses: &updates = 0x6dfee8, p_updates = 0x6dfee8
Now updates = 7
```

## 声明和初始化指针

指针声明必须指定指针指向的数据的类型

如 `int * p_updates;`:
`* p_updates`的类型为int，含义为：p_updates是指针，而`*p_updates`是int，而不是指针。

> 写法 `int *ptr;``int* ptr;`一样,C++ 偏向第二种风格

```cpp
int* p1, p2； //p1为指针， p2为变量
```

指针初始化：
```cpp
int higgens = 5;
int* pt = &higgens;
```

## 指针潜在的危险
在C++中创建指针时，计算机将分配用来存储地址的内存，但不会分配用来存储指针所指向的数据的内存。

例如：
```cpp
long * fellow; //创建指针
*fellow = 223323； //
```
> 223323将被放在哪里是未知的，是我无法确定的。

> 一定要在对指针应用解引用运算符之前，将指针初始化为一个确定的，适当的地址

## 使用new来分配内存

至此我们使用到的指针的功能：
将指针初始化为变量的地址，变量是在编译时分配的有名称的内存，而指针只是为可以通过名称直接访问的内存提供了有一个别名

指针的真正用武之地：
在运行阶段分配未命名的内存以储存值，在这种情况下，只能通过指针来访问内存。

在C语言中使用库函数malloc（）来分配内存，也适用于C++，但是C++有更好的办法---new运算符

