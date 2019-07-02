## 数据类型

内置的C++ 类型分为两组:
- 基本类型: 整数和浮点数
- 复合类型：数组 字符串 指针和结构

## 变量

变量名规则：
- 只能只能使用字符字符，数字和下划线
- 名字的第一个字符不能使数字
- 区分大小写
- 禁止使用C++关键字
- 单下划线和双下划线开头有特殊含义

## 整型
C++中的整型包含四种类型
- short
- int
- long
- long long

由于不同类型的长度在不同系统中不同，所以C++只规定最小长度：

- short 至少16位
- int至少与short一样长
- long至少32位，且至少与int一样长
- longlong字少64位，且至少与long一样长

注：
8 bit 范围  0/255 或-128/127
16bit 范围 0/65535 或-32768/32767
32bit 范围 0/4294672295 或 -2147336148/2147336147

### 系统中整数最大长度
系统中的整数的最大长度可以使用`sizeof`运算符返回类型或变量的长度，单位为字节。

头文件`climits`包含关于整型限制的信息。例如`INT_MAX` 为`int`的最大取值，`CHAR_BIT` 为字节位数

```cpp
//limits.cpp --some integer limits

#include <iostream>
#include <climits>

int main()
{	
	using namespace std;
	int n_int = INT_MAX;
	short n_short = SHRT_MAX;
	long n_long = LONG_MAX;
	long long n_llong = LLONG_MAX;

	// sizeof operator yields size of type or of variable
	cout << "int is " << sizeof(int) << " bytes. " <<endl;
	cout << "short is " << sizeof n_short << " bytes. " <<endl;
	cout << "long is " << sizeof n_long << " bytes. " <<endl;
	cout << "long long is " << sizeof n_llong << " bytes. " <<endl;
	cout << endl;

	cout << "Maximum values: " <<endl;
	cout << "int: " << n_int << endl;
	cout << "short: " << n_short << endl;
	cout << "long: " << n_long << endl;
	cout << "long long : " << n_llong << endl<< endl;

	cout << "Minimum int value = " << INT_MIN << endl;
	cout << "Bits per byte = " << CHAR_BIT << endl;

	return 0;
}
```
结果：
```
int is 4 bytes.
short is 2 bytes.
long is 4 bytes.
long long is 8 bytes.

Maximum values:
int: 2147483647
short: 32767
long: 2147483647
long long : 9223372036854775807

Minimum int value = -2147483648
Bits per byte = 8
```
> climits 文件中包含下面类似的语句：
```cpp
# define INT_MAX 32676
```

### 初始化
初始化将赋值和声明合并在一起，如：`int n_int = INT_MAX;`

初始化的形式：
```cpp
int uncles = 5;
int aunts = uncles;
int chairs = aunts + uncles + 4;

//C++特性
int wren(432); //将432 值赋值给wren变量

//C++ 11 新特性:
int hamburgers = {24}; //24赋值给hamburgers
int emus{7} ;// 7赋值给emus，如果{}内为空，则初始化为0
int rheas = {7}; //与上面等价，如果{}内为空，则初始化为0
```
> 注意：当程序执行到该声明时，表达式中的所有值都必须是已知的。

> C++ 11 使大括号初始化器可用于任何类型，可以使用等号也可以不使用，是一种通用的初始化语法。如数组等

## 无符号类型
```cpp
unsigned short change;
unsigned int rovert;
unsigned quarterback; // 等价于 unsigned int quarterback
unsigned long gone;
unsigned long long lang_lang;
```

### 整型溢出

65页
