# 内存模型和命名空间

## 单独编译

通常C++大型程序都是由多个源代码文件组成，这些文件代码可能共享一些数据，这样的程序涉及到文件的单独编译。

C++鼓励将组件函数放到单独的文件中，然后可以单独编译这些文件，然后将它们链接成可执行程序。（C++编译器既编译程序，也管理链接器。）如果只修改了一个文件，则可以只重新编译该文件，然后将它与其他文件的编译版本链接。

C++ 代码文件的一般形式：（以对结构进行操作的函数为例）

- 头文件：包含结构声明和使用这些结构的函数原型

- 源代码文件：包含与结构相关的函数的代码

- 源代码文件（main）：包含调用与结构相关的函数的代码

这种组织方式也与OOP方法一致。 一个头文件包含了用户定义类型的定义，另一个文件包含操纵用户定义类型的函数的代码。这两个文件组成一个软件包，可用于各种程序中。

头文件中常包含以下内容：

- 函数原型
- 使用#define或const定义的符号常量
- 结构声明
- 类声明
- 模板声明
- 内联函数

> 结构声明放在头文件中是可以的，因为它们不创建变量
>
> 被声明为const的数据和内联函数有特殊的链接属性

**同一个文件中只能将同一个头文件包含一次**

头文件中include:

 `<func.h>` :编译器将在存储标准头文件的主机系统的文件系统中查找

`"func.h"` : 编译器将首先查找当前的工作目录或源代码目录，如果没有找到头文件，则将在标准位置查找。

> 在g++ 编译器中，只需要将源代码文件加入到项目中，而不用加入头文件，因为#include指令管理头文件。
>
> 不要使用#include来包含源代码文件，这样导致多重声明

示例：

- coordin.h

```cpp
// coordin.h -- structure templates and function prototypes
// structure templates
#ifndef COORDIN_H_
#define COORDIN_H_

struct polar
{
    double distance;    // distance from origin
    double angle;        // direction from origin
};
struct rect
{
    double x;        // horizontal distance from origin
    double y;        // vertical distance from origin
};

// prototypes
polar rect_to_polar(rect xypos);
void show_polar(polar dapos); 

#endif
```

- file1.cpp

```cpp
// file1.cpp -- example of a three-file program
#include <iostream>
#include "coordin.h" // structure templates, function prototypes
using namespace std;
int main()
{
    rect rplace;
    polar pplace;

    cout << "Enter the x and y values: ";
    while (cin >> rplace.x >> rplace.y)  // slick use of cin
    {
        pplace = rect_to_polar(rplace);
        show_polar(pplace);
        cout << "Next two numbers (q to quit): ";
    }
    cout << "Bye!\n";
    return 0; 
}
```



- file2.cpp

```cpp
// file2.cpp -- contains functions called in file1.cpp
#include <iostream>
#include <cmath>
#include "coordin.h" // structure templates, function prototypes

// convert rectangular to polar coordinates
polar rect_to_polar(rect xypos)
{
    using namespace std;
    polar answer;

    answer.distance =
        sqrt( xypos.x * xypos.x + xypos.y * xypos.y);
    answer.angle = atan2(xypos.y, xypos.x);
    return answer;      // returns a polar structure
}

// show polar coordinates, converting angle to degrees
void show_polar (polar dapos)
{
    using namespace std;
    const double Rad_to_deg = 57.29577951;

    cout << "distance = " << dapos.distance;
    cout << ", angle = " << dapos.angle * Rad_to_deg;
    cout << " degrees\n";
}
```

在Linux系统中编译上述例程的过程：

![1563272021393](res/1563272021393.png)

## 存储持续性，作用域 和 链接性

###  存储持续性

C++使用不同 的方案来存储数据，这些方案的区别在于数据保留在内存中的时间

- 自动存储持续性：在函数定义中声明的变量（包括函数参数）的内存持续性为自动的。它们在程序开始执行其所属的函数或代码块时被创建，在执行完函数或代码块时，它们使用的内存被释放。
- 静态存储持续性：在函数定义外定义的变量和使用关键字static定义的变量的存储持续性都为静态。它们在程序整个运行过程中都存在。
- 线程存储持续性（C++11）：变量使用关键字thread_local声明，其生命周期与所属的线程一样长。
- 动态存储持续性：用new运算符分配的内存将一直存在，直到使用delete运算将其释放或程序结束为止。也叫做自由存储或堆。

### 作用域（scope）和链接性

作用域描述了名称在文件（翻译单元）的多大范围内可见。例如：

- 函数中定义的变量可在函数中使用，但不能在其他函数使用
- 而在文件中的函数定义之前定义的变量则可在该文件内所有函数中使用

链接性描述了名称如何不同单元间共享。

- 链接性为外部的名称可在文件间共享
- 链接性为内部的名称只能由一个文件中的函数共享
- 自动变量的名称没有链接性，因为它们不能共享

C++变量的作用域：

- 作用域为局部的变量只在定义它的代码块中可用
  - 自动变量的作用域为局部
- 作用域为全局（也叫文件作用域）的变量在定义位置到文件结尾之间都可用
  - 静态变量的作用域可能是局部或者全局（取决于它的定义方式）

- 在函数原型作用域中使用的名称只在包含参数列表的括号内可用（即函数原型的名称不重要）
- 在类中声明的成员的作用域为整个类
- 在命名空间中的声明的变量的作用域为整个名称空间

> C++函数的作用域可以是整个类或整个名称空间（包括全局），但不能是局部的（因为不能再代码块内定义函数，如果函数作用域为局部，则只对它自己可见，因此不能被其他函数调用）

## 各种数据存储方案：

### 自动存储持续性

在默认情况下，在函数中声明的函数参数和变量的存储持续性为自动，作用域为局部，没有链接性。作用域的起点是其声明位置。

程序开始执行这些变量所属的代码块时，将为其分配内存，函数结束后，这些变量将自动消失。

如果再代码块中定义了变量，则该变量的存在时间和作用域都被将限制在该代码块内。

```cpp
int main()
{
	int teledeli = 5;
    {
        int websight = -2;
        cout << websight << teledeli << endl;
    }
    cout << teledeli << endl;
    ...
}
```

>  teledeli 在内部代码块和外部代码块都是可见的，而websight只在内部代码块中可见，它们的作用域是从 定义它的 位置到代码块的结尾。

如果在内部和外部代码块都定一个相同的变量时：

```cpp
int main()
{
	int teledeli = 5;
    {
        int teledeli = 2;
        int websight = -2;
        cout << websight << teledeli << endl;
    }
    cout << teledeli << endl;
    ...
}
```

> 在内部代码块中，新定义的teledeli会在内部代码块隐藏之前在外部代码块定义的teledeli。

![1563280483491](res/1563280483491.png)

### 自动变量和栈

由于自动变量的数目随函数的开始和结束而增减，因此程序必须再运行时对自动变量进行管理，常用的方式是留出一段内存，并将其视为栈，以管理变量的增减。

栈的默认长度取决于实现。新数据被放到原有数据的相邻的上面的内存单元中，程序使用在两个指针来跟踪栈，一个指针指向栈底--栈的开始位置，一个指向栈顶--下一个可用的内存单元。 

当函数被调用时，其自动变量将被加入栈中，栈顶指针指向变量后面的下一个可用的内存单元。函数结束时，栈顶指针被重置为函数被调用前的值，从而释放新变量使用的内存。

栈是LIFO（先进后出），最后加入栈的变量首先被弹出。

![1563288442383](res/1563288442383.png)

### 静态持续变量

C++为静态存储持续性变量提供了3种链接特性：

- 外部链接性（可在其他文件中访问）
- 内部链接性（只能再当前文件中访问）
- 无链接性（只能在当前函数或代码块中访问）

这三种链接特性都在整个程序执行期间存在。因此程序不需要使用栈，编译器将分配固定的内存块来存储所有的静态变量。 

所有的静态持续变量都有下述初始化特征：未被初始化的静态变量的所有位置都被置为0。

3种链接特性的创建：

- 外部链接性：必须在代码块的外面声明
- 内部链接性：必须在代码块的外面声明，并使用static限定符
- 无链接性：必须在代码块内声明，并使用static限定符

例：

```cpp
....
int global = 1000; //静态存储持续性，外部链接性
static int one_file = 50; //静态存储持续性，内部链接性
int main()
{
    ....
}
void funct1(int n)
{
    static int count = 0; //静态存储持续性，无链接性
    int llama = 0;
}
void funct2(int q)
{
    ....
}
....
```

> funct1中llama 与 count的区别：即使funct1（）函数没有被执行，count也留在内存中。
>
> global 和 one_file 都可以在main（），funct1（）和funct2（）使用。one_file只能在包含上述代码的文件中使用。 global可以在其他文件中使用。

![1563291220969](res/1563291220969.png)

#### 静态变量的初始化

初始化的方式：

- 静态初始化：零初始化和常量表达式初始化被统称为静态初始化，在编译器处理文件(翻译单元)时初始化变量。
  - 在常量表达式初始化之前会默认先零初始化，然后根据情况再进行常量表达式初始化
- 动态初始化：变量将在编译后进行初始化。

```cpp
#include <cmath>
int x; //零初始化
int y = 5; //常量表达式初始化
long z = 13 * 13; //常量表达式初始化
int enough = 2 * sizeof(long) + 1; //常量表达式初始化
const double pi = 4.0 * atan(1.0); //动态初始化
// 首先xyz enough pi 都被零初始化，然后编译器计算常量表达式，并将y z enough 分别使用常量表达式进行初始化。 pi需要调用atan（）,需要等到该函数被链接且程序执行时才能再次初始化。
```

#### 静态持续性，外部链接性

链接性为外部的变量通常简称为外部变量，它们的存储持续性为静态，作用域为整个文件。外部变量是在函数外部定义的，因此对所有函数而言都是外部的。例如，可以在main（）前面或头文件中定义它们。可以在文件中位于外部变量定义后面的任何函数中使用它，因此外部变量也称全局变量（相对于局部的自动变量）。

在一个文件中声明或者初始化一个外部变量，在其他文件中如果想使用这个变量，只能使用关键字extern引用声明，并且不进行初始化。如果使用extern并且进行初始化则，就是初始化一个为新的外部变量。

```cpp
//file01.cpp
extern int cats = 20; // 初始化
int dogs = 22; // 初始化
int fleas; // 声明

//file02.cpp
extern int cats; //引用声明
extern int dogs; //引用声明

//file98.cpp
extern int cats; //引用声明
extern int dogs; //引用声明
extern int fleas; //引用声明
```

![1563294458451](res/1563294458451.png)

示例：

external.cpp

```cpp
// external.cpp -- external variable
// compile with support.cpp
#include <iostream>
// external variable
double warming = 0.3;       // warming defined

// function prototypes
void update(double dt);
void local();

int main()                  // uses global variable
{
    using namespace std;
    cout << "Global warming is " << warming << " degrees.\n";
    update(0.1);            // call function to change warming
    cout << "Global warming is " << warming << " degrees.\n";
    local();                // call function with local warming
    cout << "Global warming is " << warming << " degrees.\n";
    return 0;
}
```

support.cpp

```cpp
// support.cpp -- use external variable
// compile with external.cpp
#include <iostream>
extern double warming;  // use warming from another file

// function prototypes
void update(double dt);
void local();

using std::cout;
void update(double dt)      // modifies global variable
{
    extern double warming;  // optional redeclaration 可选
    warming += dt;          // uses global warming
    cout << "Updating global warming to " << warming;
    cout << " degrees.\n";
}

void local()                // uses local variable
{
    double warming = 0.8;   // new variable hides external one

    cout << "Local warming = " << warming << " degrees.\n";
        // Access global variable with the
        // scope resolution operator
    cout << "But global warming = " << ::warming;
    cout << " degrees.\n";
}
```

> local()函数中，局部变量warming将隐藏全局变量
>
> 作用域解析运算符`::` ，放在变量名前，该运算符表示使用变量的全局版本。

外部存储尤其适合表示常量数据，可以使用const防止数据被修改。

如：

```cpp
const char * const months[12] = {
    "January", "February", "March",
    ...
}
```

### 静态持续性 内部链接性


