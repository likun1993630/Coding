https://blog.csdn.net/qq_16775293/article/details/88189184#_303

https://www.jianshu.com/p/054484d0892a

https://www.codecogs.com/latex/eqneditor.php

<img src="https://latex.codecogs.com/gif.latex?\left&space;\{&space;2334&space;\right&space;\}" title="\left \{ 2334 \right \}" />

<!-- \left \{ 2334 \right \} -->


[TOC]

$$
\frac{a}{b}
$$

c++的enum工具提供了另一种创建符号常量的方式，这种方式可以代替const。enum允许定义新类型。使用enum的句法与使用结构相似。

定义默认枚举类型：

```cpp
enum spectrum {red,  orange, yellow, green, blue, violet, indigo, ultraviolet};
// 默认情况下，将整数赋给枚举量，从0开始。
// 含义 相当于定义了一系列符号常量 red = 0， orange = 1 等
```

- spectrum被称为枚举，是新类型的名称。
- 将red，orange等作为符号常量，它们对应整数值0~7，这些常量叫作枚举量。

使用枚举名声明定义的枚举类型的变量：

```cpp
spectrum band; //band的类型为 spectrum
//band的取值只能在spectrum范围内
band = blue;  // band = 4
band = 2000; //错误
```

枚举只定义了赋值运算：

```cpp
band = orange; //band = 1
```

枚举量是整型，可被提升为int类型，但int类型不能自动转换为枚举类型。

```cpp
int color = blue;
color = 3 + red; // color = 3 //red 被自动转换为int再进行加法运算
```

可以同强制类型转换使赋值给枚举变量的int值有效：

```cpp
band = spectrum(3); //等效于 band = green
```

如果打算只使用常量，而不创建枚举类型的变量，则可以省略枚举类型的名称：

```cpp
enum {red,  orange, yellow, green, blue, violet, indigo, ultraviolet};
//此时就可以使用red,oringe 等对应的值。
int a = orange; // a = 1
```

示例：

```cpp
#include <iostream>

enum spectrum {red,  orange, yellow, green, blue, violet, indigo, ultraviolet};
enum {convar1, convar2, convar3};

int main()
{
	using namespace std;
	spectrum band;
	band = blue;
	cout << "band = "<< band << endl;
	band = orange;
	cout << "band = "<< band << endl;

	int color = blue;
	color = 3 + red;
	cout << "color = "<< color << endl;

	band = spectrum(3);
	cout << "band = "<< band << endl;

	int a = convar2;
	cout << "a = " << a << endl;

	return 0;
}
```

```
band = 4
band = 1
color = 3
band = 3
a = 1
```

### 设置枚举量的值

```cpp
enum bits{one = 1, two = 2, four = 4, eight = 8};
enum bigstep{first, second = 100, third}; //first默认值为0，third默认值为101
enum {zero, null = 0, one, numero = 1}; //zero和null为0，one和numero为1
```

### 枚举的取值范围：

每个枚举都有取值范围，通过强制类型转换，可以将取值范围中的任何整数值赋给枚举变量，即使这个值不是枚举值。

取值范围计算：

- 上限：大于最大值2的幂的最小取值减1
- 下限：
  - 不小于零 则下限为0
  - 方法同上限，然后加负号

```cpp
enum bits{one = 1, two = 2, four = 4, eight = 8}; 
bits myflag;
myflag = bits(6); //myflag = 6 //6属于取值范围
```

