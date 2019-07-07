# 结构struct

结构是一种比数组更灵活的**数据格式**，同一个结构可以存储多种类型的数据。

创建结构分两步：
- 定义结构描述：描述并标记能够存储在结构中的各种数据类型
- 按照描述创建结构体

例：
```cpp
struct inflatable //structure declaration
{
  char name[20];
  float volume;
  double price;
};
```
![](./res/4.structinflatable.png)

- inflatble为标识符，是这种新数据格式的名称
- 定义了三个成员

使用上面定义的数据格式：
```cpp
inflatable hat; //定义inflatable类型的变量hat
inflatable woopie_cusion; //
inflatable mainframe; //

struct inflatable goose; //可以加关键字struct，也可以不加
```

struct成员访问：

使用 `.` 运算符访问成员，如 hat.volume 指结构的volume成员

## 在程序中使用结构
```cpp
// structur.cpp -- a simple structure
#include <iostream>
struct inflatable   // structure declaration
{
    char name[20];
    float volume;
    double price;
};

int main()
{
    using namespace std;
    inflatable guest =
    {
        "Glorious Gloria",  // name value
        1.88,               // volume value
        29.99               // price value
    };  // guest is a structure variable of type inflatable
// 初始化，注意使用逗号隔开，可以不换行。
    inflatable pal =
    {
        "Audacious Arthur",
        3.12,
        32.99
    };  // pal is a second variable of type inflatable
    cout << "Expand your guest list with " << guest.name;
    cout << " and " << pal.name << "!\n";
// pal.name is the name member of the pal variable
    cout << "You can have both for $";
    cout << guest.price + pal.price << "!\n";
    return 0; 
}
```
结果：
```
Length of string in charr before input: 3
Length of string in str before input: 0
```

### 结构体声明
在同一个文件中，结构体的声明有2种选择：
- 将声明放在main（）函数中，紧跟在开始括号后面
- 将声明放在main（）函数前面，这种方式叫做外部声明

> 通常应该使用外部声明

![](./res/4.structdeclaration.png)

### C++11 结构体初始化
同数组一样，C++11也支持将列表初始化用于结构，且等号是可选的
> 不允许缩窄

```cpp
inflatable duch {"Daphe", 0.12, 9.98}; 

//将成员穿不设为零
inflatable mayor {}；
```

### 结构体其他属性
- 可以将结构体作为参数传递给函数
- 可以让函数返回一个结构
- （成员赋值）可以使用赋值运算符（=）将结构赋给另一个**同类型**的结构，元素将一一对应

## 结构数组
infatable 结构包含一个数组（name）。也可以创建元素为结构的数组，方法和创建基本类型完全相同

如：创建一个包含100个inflatable结构的数组：
```cpp
inflatable gifts[100]; 

```
> 结构数组gifts每个元素（从gift[0]到gift[99]）都是inflatable对象。

```cpp
cin >> gifts[0].volume;
cout << gifts[99].price << endl;

//初始胡结果数组
infatable guests[2] = 
{
  {"Bambi", 0.5, 21.99}, //第一个结构
  {"Godzilla", 2000, 565.99} //第二个结构
};
```

## 结构中的位字段
与C语言一样，C++ 允许只当占用特定位数的结构成员，这使得创建与某个硬件设备上的寄存器对应的数据结构非常方便。

如：
```cpp
struct torgle_register
{
  unsigned int SN : 4; //4bits 给 SN的值
  unsigned int : 4; //4bits 空着不用
  bool goodIn : 1; // 1bits 给googIn
  bool goodTorgle : 1; // 1bits 给googTorgle
}

torgle_register tr = {14, true, false};
....
if (tr.goodIn)
....
```

# 共用体union
共用体也是是一种数据格式，它能够存储不同的数据类型，但是只能同时存储其中的一种类型。

例如：
```cpp
union one4all
{
  int int_val;
  long long_val;
  double double_val;
};

//使用方法：
one4all pail;
pail.int_val = 15; //存储 一个整型
cout << pail.int_val;
pail.double_val = 1.38; //存储一个double，但是之前的int被清空
cout << pail.double_val;
```

因此，pail有时可以是int变量，有时可以是double变量。 由于共用体每次只能存储一个值，所以它必须有足够的空间来存储最大的成员，即共用体长度为其最大成员的长度。

> 共用体通常用于节省内存，这对于嵌入式系统意义很大。

# 枚举enum
暂时略过
