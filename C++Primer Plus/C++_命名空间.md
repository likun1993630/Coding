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

将static限定符用于作用域为整个文件的变量时，该变量的链接性为内部。

如果想创建在整个文件中都可以使用的静态变量，使用static可以避免与其他文件中的作用域为整个文件的变量发生冲突。

如果文件定义了一个静态外部变量，其名称与另一个文件中声明的常规外部变量相同时，则在该文件中，静态变量将隐藏常规外部变量：

```cpp
//file 1
int errors = 20; //外部变量
...
//-----------------------------
// file2
static int errors = 5; //file2的内部变量
void froobish()
{
    cout << errors; //uses errors defined in file2
    ...
}
```

示例：

twofile1.cpp

```cpp
// twofile1.cpp -- variables with external and internal linkage
#include <iostream>     // to be compiled with two file2.cpp
int tom = 3;            // external variable definition
int dick = 30;          // external variable definition
static int harry = 300; // static, internal linkage
// function prototype
void remote_access();

int main()
{
    using namespace std;
    cout << "main() reports the following addresses:\n";
    cout << &tom << " = &tom, " << &dick << " = &dick, ";
    cout << &harry << " = &harry\n";
    remote_access();
    return 0; 
}
```

twofile2.cpp

```cpp
// twofile2.cpp -- variables with internal and external linkage
#include <iostream>
extern int tom;         // tom defined elsewhere
static int dick = 10;   // overrides external dick
int harry = 200;        // external variable definition,
                        // no conflict with twofile1 harry
void remote_access()
{
    using namespace std;
    cout << "remote_access() reports the following addresses:\n";
    cout << &tom << " = &tom, " << &dick << " = &dick, ";
    cout << &harry << " = &harry\n";
}
```

### 静态存储持续性， 无链接性

将static限定符用于在代码块中定义的变量。

在代码块中使用static时，将导致局部变量的存储持续性为静态的。这意味着，虽然该变量只在该代码块中可用，但它在该代码块不处于活动状态时依然存在。因此在两次函数调用之间，静态变量的值将保持不变。

如果初始化了静态局部变量，则程序只在启动时进行一次初始化。以后再调用函数时，将不会像自动变量那样再次被初始化。

示例：

```cpp
// static.cpp -- using a static local variable
#include <iostream>
// function prototype
void strcount();

int main()
{
    using namespace std;

    for (int i = 0; i < 2; ++i)
    {
        strcount();
        strcount();
    }
    return 0;
}

void strcount()
{
    using namespace std;
    static int total = 0;        // static local variable
    int count = 0;               // automatic local variable
    while (count < 3)               // go to end of string
        count++;
    total += count;
    cout << count << " count\n";
    cout << total << " total\n";
}
```

```cpp
3 count
3 total
3 count
6 total
3 count
9 total
3 count
12 total
```

### 存储说明符和限定符

存储说明符：

- static
- extern
- thread_local
- mutable

限定符：

- const

#### const

内存被初始化后，程序便不能再对它进行修改。

const限定符对默认存储类型有一定的影响，在默认情况下全局变量的链接性为外部，但const全局变量的链接性为内部的。（类似static）

```cpp
const int fingers = 10; // 等效于 static const int fingers = 10
int main(void)
{
    ...
}
```

好处：将一组const常量放在头文件中，并在同一个程序的多个文件中使用该头文件，当预处理将头文件的内容包含到每一个源文件中后，所有源文件都包含了该组const常量，并且相互不冲突。

一个文件需要使用别的文件的该变量时，也需要使用extern关键字来引用声明：

```cpp
extern const int fingers; //另外一个文件想要使用fingers
```

也可以使用extern关键字使其具有外部链接性：

```cpp
extern const int states = 50; //初始化const的外部链接性
```

#### mutable

可以用来指出，即使结构或类变量为const，其某个成员也可以被修改。

```cpp
struct data
{
    char name[30];
    mutable int accesses;
    ...
}
const data veep = {"Hello World", 0, ...};
strcpy(veep.name, "Joye Joux"); //不允许
veep.accesses++; //允许
```

### 函数和链接性

和变量一样，函数也有链接性，虽然可选择的范围比变量小。C++不允许在一个函数中定义另外一个函数，因此**所有函数的存储持续性都自动为静态的，即在整个程序执行期间都一直存在**。

在默认情况下，函数的链接性为外部的，即可以在文件间共享。

可以在函数原型中使用关键字extern来指出函数是在另一个文件中定义的，不过这是可选的（要让程序在另一个文件中查找函数，该文件必须作为程序的组成部分被编译，或者是由链接程序搜索的库文件）。还可以使用关键字static将函数的链接性设置为内部的，使之只能在一个文件中使用用。必须同时在原型和函数定义中使用static关键字：

```cpp
static int private(double x);
...
static int private(double x)
{
    ...
}
```

这意味着该函数只在这个文件中可见，还意味着可以在其他文件中定义同名的的函数。和变量一样，在定义静态函数的文件中，静态函数将覆盖外部定义，因此即使在外部定义了同名的函数，该文件仍将使用静态函数。
单定义规则也适用于**非内联函数**，因此对于每个非内联函数，程序只能包含一个定义。对于链接性为外部的函数来说，这意味着在多文件程序中，只能有一个文件（该文件可能是库文件）包含该函数的定义，但使用该函数的每个文件都应包含其函数原型。
**内联函数**不受这项规则的约束，这允许程序员能够将内联函数的定义放在头文件中。这样，包含了头文件的每个文件都有内联函数的定义。然而，C++要求同一个函数的所有内联定义都必须相同。

### C++查找函数

程序的某个文件中调用一个函数时，C++将到哪里去寻找该函数的定义呢？

如果该文件中的函数原型指出该函数是静态的，则编译器将只在该文件中查找函数定义；否则，编译器（包括链接程序）将在所有的程序文件中查找。如果找到两个定义，编译器将发出错误消息，因为每个外部函数只能有一个定义。
如果在程序文件中没有找到，编译器将在库中搜索。这意味着如果定义了一个与库函数同名的函数，编译器将使用程序员定义的版本，而不是库函数。

### 存储方案与动态分配

动态内存由运算符new和delete控制，而不是由作用域和链接性规则控制。所以可以在一个函数中分配动态内存，而在另一个函数中将其释放。

假设一个函数中包含下面的语句：

```cpp
float * p_fees = new float[20];
```

- 由new分配的内存将一直保留在内存中，直到使用delete运算符将其释放、
- 包含该语句的语句块之完毕后，p_fees指针将消失
- 如果其他函数想使用该内存，则必须将其地址传递或返回给该函数
- 也可以将p_fees的链接性声明为外部的，则本文件中位于该声明后面的所有函数和其他为文件都可以使用它。其他文件中需使用声明`extern float* p_fees;`

### new 用法补充（指针章节和函数章节的new进行拓展）

#### 使用new运算符初始化

```cpp
int * pi = new int (6); // *pi = 6
double * pd = new double (99.99); //*pd = 99.99

int * pin = new int {6}; //C++11
double * pdo = new double {99.99}; // C++11

struct where {double x; double y; double z;};
where * one = new where {2.5, 5.3, 7.2};  //C++11
int * ar = new int [4] {2, 4, 6, 7}; //C++11
```

#### 定位new运算符

定位new运算符可以指定要使用的内存位置：

- 需要包含头文件new

- 提供所需地址的参数

- 不能使用delete释放内存空间

  > new运算符实际上将参数强制转换为 void *，以便能够赋给任何指针类型。

```cpp
#include <new>
struct chaff
{
    char dross[20];
    int slag;
};
char buffer1[50]; //静态持续变量,外部链接性
char buffer2[500]; //静态持续变量,外部链接性
int main()
{
    chaff *p1, *p2;
    int *p3, *p4;
    //new常规形式
    p1 = new chaff; //在heap创建结构
    p3 = new int[20]; //在heap创建int数组
    //定位new形式
    p2 = new (buffer1) chaff; //在buffer1中创建结构
    p4 = new (buffer2) int [20]; //在buufer2中创建int数组
    ...
}
```

示例：

```cpp
// newplace.cpp -- using placement new
#include <iostream>
#include <new> // for placement new
const int BUF = 512;
const int N = 5;
char buffer[BUF];      // chunk of memory
int main()
{
    using namespace std;

    double *pd1, *pd2;
    int i;
    cout << "Calling new and placement new:\n";
    pd1 = new double[N];           // use heap
    pd2 = new (buffer) double[N];  // use buffer array
    for (i = 0; i < N; i++)
        pd2[i] = pd1[i] = 1000 + 20.0 * i;
    cout << "Memory addresses:\n" << "  heap: " << pd1
        << "  static: " <<  (void *) buffer  <<endl; //强制转换，输出地址
    cout << "Memory contents:\n";
    for (i = 0; i < N; i++)
    {
        cout << pd1[i] << " at " << &pd1[i] << "; ";
        cout << pd2[i] << " at " << &pd2[i] << endl;
    }

    cout << "\nCalling new and placement new a second time:\n";
    double *pd3, *pd4;
    pd3= new double[N];            // find new address
    pd4 = new (buffer) double[N];  // overwrite old data
    for (i = 0; i < N; i++)
        pd4[i] = pd3[i] = 1000 + 40.0 * i;
    cout << "Memory contents:\n";
    for (i = 0; i < N; i++)
    {
        cout << pd3[i] << " at " << &pd3[i] << "; ";
        cout << pd4[i] << " at " << &pd4[i] << endl;
    }

    cout << "\nCalling new and placement new a third time:\n";
    delete [] pd1;
    pd1= new double[N];
    pd2 = new (buffer + N * sizeof(double)) double[N]; //偏移40个字节
    for (i = 0; i < N; i++)
        pd2[i] = pd1[i] = 1000 + 60.0 * i;
    cout << "Memory contents:\n";
    for (i = 0; i < N; i++)
    {
        cout << pd1[i] << " at " << &pd1[i] << "; ";
        cout << pd2[i] << " at " << &pd2[i] << endl;
    }
    delete [] pd1;
    delete [] pd3;
    return 0;
}
```

```cpp
Calling new and placement new:
Memory addresses:
  heap: 0xb36cf0  static: 0x4c7020
Memory contents:
1000 at 0xb36cf0; 1000 at 0x4c7020
1020 at 0xb36cf8; 1020 at 0x4c7028
1040 at 0xb36d00; 1040 at 0x4c7030
1060 at 0xb36d08; 1060 at 0x4c7038
1080 at 0xb36d10; 1080 at 0x4c7040

Calling new and placement new a second time:
Memory contents:
1000 at 0xb3a508; 1000 at 0x4c7020
1040 at 0xb3a510; 1040 at 0x4c7028
1080 at 0xb3a518; 1080 at 0x4c7030
1120 at 0xb3a520; 1120 at 0x4c7038
1160 at 0xb3a528; 1160 at 0x4c7040

Calling new and placement new a third time:
Memory contents:
1000 at 0xb36cf0; 1000 at 0x4c7048
1060 at 0xb36cf8; 1060 at 0x4c7050
1120 at 0xb36d00; 1120 at 0x4c7058
1180 at 0xb36d08; 1180 at 0x4c7060
1240 at 0xb36d10; 1240 at 0x4c7068
```

> 定位new运算符使用传递给他的地址，它不跟踪哪些单元被使用，也不查找未使用的内存块。需要手动管理，比如设置相对开头位置设置偏移量。
>
> 不能使用delete删除 定位new使用的内存空间

# 名称空间

p324/341
