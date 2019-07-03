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
# define INT_MAX 32767
```
在C++编译过程中，首先将源代码传递给预处理器。在这里，#define 和 # include一样，也是一个预处理编译指令。该编译指令告诉预处理器：在程序中查找INT_MAX，并将所有的INT_MAX都替换为32767。因此#define编译指令的工作方式与文本编辑器的全局搜索并替换命令相似。修改后的程序将在完成这些替换后被编译。也可以使用#define 来定义自己的符号常量。
> #define 编译指令是C语言遗留下来的，C++中还有更好的创建符号常量的方法（const）。

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

```cpp
// exceed.cpp -- exceeding some integer limits
#include <iostream>
#define ZERO 0      // makes ZERO symbol for 0 value
#include <climits>  // defines INT_MAX as largest int value
int main()
{
    using namespace std;
    short sam = SHRT_MAX;     // initialize a variable to max value
    unsigned short sue = sam;// okay if variable sam already defined

    cout << "Sam has " << sam << " dollars and Sue has " << sue;
    cout << " dollars deposited." << endl
         << "Add $1 to each account." << endl << "Now ";
    sam = sam + 1;
    sue = sue + 1; 
    cout << "Sam has " << sam << " dollars and Sue has " << sue;
    cout << " dollars deposited.\nPoor Sam!" << endl;
    sam = ZERO;
    sue = ZERO;
    cout << "Sam has " << sam << " dollars and Sue has " << sue;
    cout << " dollars deposited." << endl;
    cout << "Take $1 from each account." << endl << "Now ";
    sam = sam - 1;
    sue = sue - 1;
    cout << "Sam has " << sam << " dollars and Sue has " << sue;
    cout << " dollars deposited." << endl << "Lucky Sue!" << endl;
	// cin.get();
    return 0; 
}
```

结果：
```
Sam has 32767 dollars and Sue has 32767 dollars deposited.
Add $1 to each account.
Now Sam has -32768 dollars and Sue has 32768 dollars deposited.
Poor Sam!
Sam has 0 dollars and Sue has 0 dollars deposited.
Take $1 from each account.
Now Sam has -1 dollars and Sue has 65535 dollars deposited.
Lucky Sue!
```
典型的整型溢出行为：

![])(./res/3.intoverflow.png)

## 整型字面值
整型字面值（常量）是显式的书写的常量，如212。 C++中可以在代码中直接书写8进制，10进制和16进制的常量，如042，42 和 0x42.

```cpp
// hexoct1.cpp -- shows hex and octal literals
#include <iostream>
int main()
{
    using namespace std;
    int chest = 42;     // decimal integer literal
    int waist = 0x42;   // hexadecimal integer literal
    int inseam = 042;   // octal integer literal

    cout << "Monsieur cuts a striking figure!\n";
    cout << "chest = " << chest << " (42 in decimal)\n";
    cout << "waist = " << waist << " (0x42 in hex)\n";
    cout << "inseam = " << inseam << " (042 in octal)\n";
	// cin.get();
    return 0; 
}
```
结果
```
Monsieur cuts a striking figure!
chest = 42 (42 in decimal)
waist = 66 (0x42 in hex)
inseam = 34 (042 in octal)
```
> 默认情况下cout以10进制格式显示整数。

> cout 可使用控制符 dec,hex和oct格式显示整数

### C++ 如何确定常量的类型

- 整型变量通过声明告知编译器
- 整型常量如`cout << "a = " << 12 ;` 
	- 默认存储为int类型，如果太长则使用long或者long long
	- 可以通过后缀限定类型，如 `22022L ` 为long常量
	- l/L 表示long，u/U表示unsigned int，ul表示unsigned long 等等
> 对于8进制和16进制与10进制有所不同，详见书籍p47

## char 类型（字符和小整数）
char类型是专为存储字符（如字母和数字）而设计的。

计算机存储数字简单，但是存储字母就需要将字母编码成数字，然后才能存入内存中。

char类型是一种最短整型，但是也足够长用来表示计算机系统中所有的基本符号-所有字母，数字和标点符号等。在大多数系统上，只使用一个字节的内存。

char最常被用来处理字符，但是也可以也用做比short更短的整型

字符与整数密切相关，因为它们在内部其实是被存储为整数。每个可打印的字符以及许多不可打印的字符都被分配一个唯一的数字。用于编码字符的最常见方法是 ASCII（美国信息交换标准代码的首字母简写）。当字符存储在内存中时，它实际上是存储的数字代码。当计算机被指示在屏幕上打印该值时，它将显示与数字代码对应的字符。例如，数字 65 对应大写字母 A，66 对应大写字母 B



```cpp
// chartype.cpp -- the char type
#include <iostream>
int main( )
{
    using namespace std;
    char ch1 = '2'; //acsii = 50
    char ch2 = '3'; //acsii = 51
    char ch3;
    ch3 = ch1 + ch2;
    int ch4 = ch3;

    // 'e' = 101
    cout << ch3 << endl;
    cout << ch4 << endl;
    return 0;
}
```
