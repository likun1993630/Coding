# 运算符重载

- 运算符重载是一种形式的C++多态。
- C++ 允许将重载的概念扩展到运算符上。
- 编译器根据操作数的数目和类型决定使用某运算符的哪个定义。

要重载运算符需要使用被称为运算符函数的的特殊函数形式：

```cpp
operatorop(argument-list) //op 为有效的C++运算符
```

例如：
- `operator+() `  重载+运算符
- `operator*()`  重载*运算符

重载加法运算符 + 的示例：

```cpp
// Time 自定义类

// 函数原型
Time operator+(const Time & t) const; //返回值为Time对象

// 函数定义
Time Time::operator+(const Time & t) const
{
    Time sum;
    sum.minutes = minutes + t.minutes;
	...
    return sum;
}

// 调用
// Time 类对象 total, coding, fixing, A
total = coding + fixing; //运算符表示法
total = coding.operator+(fixing); //成员函数调用形式
A = coding + fixing + total; //等效于：
A = conding.operator+(fixing.operator+(total));
```

不使用运算符重载，而使用成员函数实现加法的形式：

```cpp
// 成员函数原型
const Time Sum(const Time & t) const;

// 成员函数定义
const Time Time::Sum(const Time & t) const
{
    Time sum;
    sum.minutes = minutes + t.minutes;
	...
    return sum;
}
```

重载限制：

- 重载后的运算符必须至少有一个操作数是用户定义的类型，为了防止用户为标准类型重载运算符。
- 不能违反运算符原来的句法规则，例如将两个操作数的运算符重载为一个操作数是不允许的。
- 不能修改运算符的优先级
- 不能创建新的运算符
- 某些运算符禁止重载如，sizeof运算符，`.` 成员运算符
- 大多数运算符可以通过成员或者非成员函数进行重载，但有些运算符只能通过成员函数进行重载：
  - =  赋值运算符
  - （） 函数调用运算符
  - []  下标运算符
  - ->  通过指针访问类成员的运算符

- 允许重载的运算符详见 P388.

## 一个重载运算符的示例

mytime2.h

```cpp
// mytime2.h -- Time class after operator overloading
#ifndef MYTIME2_H_
#define MYTIME2_H_

class Time
{
private:
    int hours;
    int minutes;
public:
    Time();
    Time(int h, int m = 0);
    void AddMin(int m);
    void AddHr(int h);
    void Reset(int h = 0, int m = 0);
    Time operator+(const Time & t) const;
    Time operator-(const Time & t) const;
    Time operator*(double n) const;
    void Show() const;
};
#endif
```

mytime2.cpp

```cpp
// mytime2.cpp  -- implementing Time methods
#include <iostream>
#include "mytime2.h"

Time::Time()
{
    hours = minutes = 0;
}

Time::Time(int h, int m )
{
    hours = h;
    minutes = m;
}

void Time::AddMin(int m)
{
    minutes += m;
    hours += minutes / 60;
    minutes %= 60;
}
void Time::AddHr(int h)
{
    hours += h;
}

void Time::Reset(int h, int m)
{
    hours = h;
    minutes = m;
}

Time Time::operator+(const Time & t) const
{
    Time sum;
    sum.minutes = minutes + t.minutes;
    sum.hours = hours + t.hours + sum.minutes / 60;
    sum.minutes %= 60;
    return sum;
}

Time Time::operator-(const Time & t) const
{
    Time diff;
    int tot1, tot2;
    tot1 = t.minutes + 60 * t.hours;
    tot2 = minutes + 60 * hours;
    diff.minutes = (tot2 - tot1) % 60;
    diff.hours = (tot2 - tot1) / 60;
    return diff;
}

Time Time::operator*(double mult) const
{
    Time result;
    long totalminutes = hours * mult * 60 + minutes * mult;
    result.hours = totalminutes / 60;
    result.minutes = totalminutes % 60;
    return result;
}

void Time::Show() const
{
    std::cout << hours << " hours, " << minutes << " minutes";
}
```

usetime2.cpp

```cpp
// usetime2.cpp -- using the third draft of the Time class
// compile usetime2.cpp and mytime2.cpp together
#include <iostream>
#include "mytime2.h"

int main()
{
    using std::cout;
    using std::endl;
    Time weeding(4, 35);
    Time waxing(2, 47);
    Time total;
    Time diff;
    Time adjusted;

    cout << "weeding time = ";
    weeding.Show();
    cout << endl;
 
    cout << "waxing time = ";
    waxing.Show();
    cout << endl;
    
    cout << "total work time = ";
    total = weeding + waxing;   // use operator+()
    total.Show();
    cout << endl;

    diff = weeding - waxing;    // use operator-()
    cout << "weeding time - waxing time = ";
    diff.Show();
    cout << endl;

    adjusted = total * 1.5;      // use operator+()
    cout << "adjusted work time = ";
    adjusted.Show();
    cout << endl;
    return 0;
}
```

```
weeding time = 4 hours, 35 minutes
waxing time = 2 hours, 47 minutes
total work time = 7 hours, 22 minutes
weeding time - waxing time = 1 hours, 48 minutes
adjusted work time = 11 hours, 3 minutes
```



## 友元

C++提供了访问类对象私有部分的另一种方法：友元。

友元有3种：

- 友元函数
- 友元类
- 友元成员函数

通过让函数成为类的友元，可以赋予该函数与类的成员函数相同的访问权限。

### 需要友元函数的原因

在为类重载二元运算符时常常需要友元。

例如在Time类中，重载乘法运算符：

```cpp
A = B * 2.75; //允许，因为B为类对象
A = 2.75 * B; //不允许，因为2.75无权限访问类成员
```

解决办法：

```cpp
A = 2.75 * B;
// 上述表达式需要与非成员函数调用匹配：
A = operator*(2.75, B);
// 对应的函数原型如下：
Time operator*(double m, const Tiime & t);

// 使该函数成为类的友元函数
```

### 创建友元函数

将友元函数原型放到类的声明中：

```cpp
friend Time operator*(double m, const Time & t);
```

- 该函数不是成员函数，不能通过成员运算符来调用
- 虽然不是成员函数，但它与成员函数的访问权限相同

友元函数定义：

> 不需要Time:: 限定符

```cpp
Time operator*(double mult, const Time & t)
{
    Time result;
    long totalminutes = t.hours * mult * 60 + t.minutes * mult;
    result.hours = totalminutes / 60;
    result.minutes = totalminutes % 60;
    return result;
}
```

- `A = 2.75 * B;` 将被转换为 `A = operator*(2.75, B);`

友元函数定义的另一种方式：（此形式也可以作为非友元函数也可行）

```cpp
Time operator*(double mult, const Time & t)
{
	return t * mult; //使用 t.operator*(m)
}
```

### 上述问题的非友元函数的解决方案：

```cpp
//头文件
Time operator*(double mult, const Time & t);

//函数定义
Time operator*(double mult, const Time & t)
{
    return t * mult;
}

// 函数调用 adjusted total 为类对象
adjusted = 1.5 * total;
```

> 虽有可以使用非友元函数解决该问题，但还是推荐使用友元函数。

## 重载 << 运算符

重载 << 运算符使Time类对象可以使用使用`cout << trip` 进行打印。（trip为类对象）

### 第一个版本的重载：

由于使用重载的运算符时类对象只能位于左边，即第一个操作数。

所以常规重载只能使用这样的形式：`trip << cout;`

如果想使用`cout << trip;`,需要通过友元函数：

```cpp
void operator<<(ostream & os, const Time & t)
{
    os << t.hours << "hours, " << t.minutes << " minutes";
}
```

### 第二个版本的重载：

第一个版本的 << 无法实现：

```cpp
cout << "Trip time: " << trip << "Tuesday \n";
```

因此要将operator<<（） 函数实现为返回一个指向ostream对象的引用：

```cpp
ostream & operator<<(ostream & os, const Time & t)
{
    os << t.hours << "hours, " << t.minutes << " minutes";
    return os;
}
```

<< 运算符重载的一般形式：

```cpp
// 友元函数
ostream & operator<<(ostream & os, const c_name & obj)
{
    os << ...;
    return os;
}
```

> operator<<() 直接访问Time类对象的私有成员，所以它必须是Time类的友元
>
>  operator<<()没有直接访问ostream对象的私有成员，所以可以不是ostream的友元

## 带有友元函数的示例：

mytime3.h

```cpp
// mytime3.h -- Time class with friends
#ifndef MYTIME3_H_
#define MYTIME3_H_
#include <iostream>

class Time
{
private:
    int hours;
    int minutes;
public:
    Time();
    Time(int h, int m = 0);
    void AddMin(int m);
    void AddHr(int h);
    void Reset(int h = 0, int m = 0);
    Time operator+(const Time & t) const;
    Time operator-(const Time & t) const;
    Time operator*(double n) const;
    friend Time operator*(double m, const Time & t)
        { return t * m; }   // inline definition
    friend std::ostream & operator<<(std::ostream & os, const Time & t);

};
#endif
```

mytime3.cpp

```cpp
// mytime3.cpp  -- implementing Time methods
#include "mytime3.h"

Time::Time()
{
    hours = minutes = 0;
}

Time::Time(int h, int m )
{
    hours = h;
    minutes = m;
}

void Time::AddMin(int m)
{
    minutes += m;
    hours += minutes / 60;
    minutes %= 60;
}

void Time::AddHr(int h)
{
    hours += h;
}

void Time::Reset(int h, int m)
{
    hours = h;
    minutes = m;
}

Time Time::operator+(const Time & t) const
{
    Time sum;
    sum.minutes = minutes + t.minutes;
    sum.hours = hours + t.hours + sum.minutes / 60;
    sum.minutes %= 60;
    return sum;
}

Time Time::operator-(const Time & t) const
{
    Time diff;
    int tot1, tot2;
    tot1 = t.minutes + 60 * t.hours;
    tot2 = minutes + 60 * hours;
    diff.minutes = (tot2 - tot1) % 60;
    diff.hours = (tot2 - tot1) / 60;
    return diff;
}

Time Time::operator*(double mult) const
{
    Time result;
    long totalminutes = hours * mult * 60 + minutes * mult;
    result.hours = totalminutes / 60;
    result.minutes = totalminutes % 60;
    return result;
}

std::ostream & operator<<(std::ostream & os, const Time & t)
{
    os << t.hours << " hours, " << t.minutes << " minutes";
    return os; 
}
```

usetime3.cpp

```cpp
//usetime3.cpp -- using the fourth draft of the Time class
// compile usetime3.cpp and mytime3.cpp together
#include <iostream>
#include "mytime3.h"

int main()
{
    using std::cout;
    using std::endl;
    Time aida(3, 35);
    Time tosca(2, 48);
    Time temp;

    cout << "Aida and Tosca:\n";
    cout << aida<<"; " << tosca << endl;
    temp = aida + tosca;     // operator+()
    cout << "Aida + Tosca: " << temp << endl;
    temp = aida* 1.17;  // member operator*()
    cout << "Aida * 1.17: " << temp << endl;
    cout << "10.0 * Tosca: " << 10.0 * tosca << endl;
    return 0; 
}
```

## 一个矢量类示例

vect.h

```cpp
// vect.h -- Vector class with <<, mode state
#ifndef VECTOR_H_
#define VECTOR_H_
#include <iostream>
namespace VECTOR
{
    class Vector
    {
    public:
        enum Mode {RECT, POL}; //Mode 作用域为VECTOR命名空间下的Vecotr类内
    // RECT for rectangular, POL for Polar modes
    private:
        double x;          // horizontal value
        double y;          // vertical value
        double mag;        // length of vector
        double ang;        // direction of vector in degrees
        Mode mode;         // RECT or POL
    // private methods for setting values
        void set_mag();
        void set_ang();
        void set_x();
        void set_y();
    public:
       Vector();
        Vector(double n1, double n2, Mode form = RECT);
        void reset(double n1, double n2, Mode form = RECT);
        ~Vector();
        double xval() const {return x;}       // report x value
        double yval() const {return y;}       // report y value
        double magval() const {return mag;}   // report magnitude
        double angval() const {return ang;}   // report angle
        void polar_mode();                    // set mode to POL
        void rect_mode();                     // set mode to RECT
    // operator overloading
        Vector operator+(const Vector & b) const;
        Vector operator-(const Vector & b) const;
        Vector operator-() const;
        Vector operator*(double n) const;
    // friends
        friend Vector operator*(double n, const Vector & a);
        friend std::ostream & operator<<(std::ostream & os, const Vector & v);
    };

}   // end namespace VECTOR
#endif
```

vect.cpp

```cpp
// vect.cpp -- methods for the Vector class
#include <cmath>
#include "vect.h"   // includes <iostream>
using std::sqrt;
using std::sin;
using std::cos;
using std::atan; //arctan一个数
using std::atan2; //arctan两个数
using std::cout;

namespace VECTOR
{	// 所处作用域： VECTOR命名空间
    // compute degrees in one radian
    const double Rad_to_deg = 45.0 / atan(1.0);
    // should be about 57.2957795130823

    // private methods
    // calculates magnitude from x and y
    void Vector::set_mag()
    { //所处作用域： VECTOR命名空间下Vector类
        mag = sqrt(x * x + y * y);
    }

    void Vector::set_ang()
    {
        if (x == 0.0 && y == 0.0)
            ang = 0.0;
        else
            ang = atan2(y, x);
    }

    // set x from polar coordinate
    void Vector::set_x()
    {
        x = mag * cos(ang);
    }

    // set y from polar coordinate
    void Vector::set_y()
    {
        y = mag * sin(ang);
    }

    // public methods
    Vector::Vector()             // 默认构造函数
    {
        x = y = mag = ang = 0.0;
        mode = RECT;
    }

    // construct vector from rectangular coordinates if form is r
    // (the default) or else from polar coordinates if form is p
    Vector::Vector(double n1, double n2, Mode form) //括号内所处作用域： VECTOR命名空间下Vector类
    { //所处作用域： VECTOR命名空间下Vector类
        mode = form;
        if (form == RECT) //RECT为枚举量
         {
             x = n1;
             y = n2;
             set_mag();
             set_ang();
        }
        else if (form == POL)
        {
             mag = n1;
             ang = n2 / Rad_to_deg;
             set_x();
             set_y();
        }
        else
        {
             cout << "Incorrect 3rd argument to Vector() -- ";
             cout << "vector set to 0\n";
             x = y = mag = ang = 0.0;
             mode = RECT;
        }
    }

    // reset vector from rectangular coordinates if form is
    // RECT (the default) or else from polar coordinates if
    // form is POL
    void Vector:: reset(double n1, double n2, Mode form)
    {
        mode = form;
        if (form == RECT)
         {
             x = n1;
             y = n2;
             set_mag();
             set_ang();
        }
        else if (form == POL)
        {
             mag = n1;
             ang = n2 / Rad_to_deg;
             set_x();
             set_y();
        }
        else
        {
             cout << "Incorrect 3rd argument to Vector() -- ";
             cout << "vector set to 0\n";
             x = y = mag = ang = 0.0;
             mode = RECT;
        }
    }

    Vector::~Vector()    // destructor
    {
    }

    void Vector::polar_mode()    // set to polar mode
    {
        mode = POL;
    }

    void Vector::rect_mode()     // set to rectangular mode
    {
        mode = RECT;
    }

    // operator overloading
    // add two Vectors
    Vector Vector::operator+(const Vector & b) const
    {
        return Vector(x + b.x, y + b.y); //调用非默认构造函数
    }

    // subtract Vector b from a
    Vector Vector::operator-(const Vector & b) const
    {
        return Vector(x - b.x, y - b.y);
    }

    // reverse sign of Vector
    Vector Vector::operator-() const
    {
        return Vector(-x, -y);
    }

    // multiply vector by n
    Vector Vector::operator*(double n) const
    {
        return Vector(n * x, n * y);
    }

    // friend methods
    // multiply n by Vector a
    Vector operator*(double n, const Vector & a)
    {
        return a * n;
    }

    // display rectangular coordinates if mode is RECT,
    // else display polar coordinates if mode is POL
    std::ostream & operator<<(std::ostream & os, const Vector & v)
    { //所处作用域： VECTOR命名空间，并没有在类内，所以此处使用枚举量需要指定类作用域
        if (v.mode == Vector::RECT)
             os << "(x,y) = (" << v.x << ", " << v.y << ")";
        else if (v.mode == Vector::POL)
        {
             os << "(m,a) = (" << v.mag << ", "
                 << v.ang * Rad_to_deg << ")";
        }
        else
             os << "Vector object mode is invalid";
        return os; 
    }

}  // end namespace VECTOR

```

使用构造函数：

```cpp
Vector folly(3.0, 4.0); //调用非默认函数
Vector foolery(20.0, 30.0, VECTOR::Vector::POL); //注意作用域解析运算符
Vector rector(20.0, 30.0, 2); //错误//2无法隐式转换为枚举型（原因作用域内枚举无法隐式转换）
```

重载+加法的方式对比：

```cpp
// 非完整形式，无法计算弧度参数
Vector Vector::operator+(const Vector & b) const
{
    Vector sum;
    sum.x = x + b.x;
    sum.y = y + b.y;
    return sum;
}

// 完整功能
Vector Vector::operator+(const Vector & b) const
{
    Vector sum;
    sum.x = x + b.x;
    sum.y = y + b.y;
    sum.set_ang(sum.x, sum.y); //假设存在set_ang函数
    sum.set_msg(sum.x, sum.y); //假设存在set_msg函数
    return sum;
}
// 最好的方法，使用构造函数
Vector Vector::operator+(const Vector & b) const
{
    return Vector(x + b.x, y + b.y); //调用非默认构造函数
}
```

randwalk.cpp

```cpp
// randwalk.cpp -- using the Vector class
// compile with the vect.cpp file
#include <iostream>
#include <cstdlib>      // rand(), srand() prototypes
#include <ctime>        // time() prototype
#include "vect.h"
int main()
{
    using namespace std;
    using VECTOR::Vector; //使用VECTOR空间的Vector类
    // time(0) 返回当前时间
    // srand() 手动设置种子
    srand(time(0));     // seed random-number generator
    double direction;
    Vector step;
    Vector result(0.0, 0.0);
    unsigned long steps = 0;
    double target;
    double dstep;
    cout << "Enter target distance (q to quit): ";
    while (cin >> target)
    {
        cout << "Enter step length: ";
        if (!(cin >> dstep))
            break;
        while (result.magval() < target)
        {
            direction = rand() % 360;
            step.reset(dstep, direction, Vector::POL); //需要指定POL的命名空间
            result = result + step; //result默认为RECT模式
            steps++;
        }
        cout << "After " << steps << " steps, the subject "
            "has the following location:\n";
        cout << result << endl;
        result.polar_mode();
        cout << " or\n" << result << endl;
        cout << "Average outward distance per step = "
            << result.magval()/steps << endl;
        steps = 0;
        result.reset(0.0, 0.0);
        cout << "Enter target distance (q to quit): ";
    }
    cout << "Bye!\n";
    return 0; 
}
```

```cpp
Enter target distance (q to quit): 100
Enter step length: 5
After 509 steps, the subject has the following location:
(x,y) = (94.6195, 34.1839)
 or
(m,a) = (100.605, 19.8636)
Average outward distance per step = 0.197653
```

将输出重定向到文件：

```cpp
#include <fstream>
...
ofstream fout;
fout.open("walk.txt");
fout << result << endl;
...
```

## 类的自动转换和强制类型转换

- 在符合规则的情况下，C++可以自动对内置类型进行转换：
  - 如： `double time = 11;`

- 可以将类定义成与基本类型或另一个类相关，使得从一种类型转换为另一种类型是有意义的。可以指示C++如何自动进行转换，或通过强制类型转换来完成。

示例：

```cpp
// 构造函数原型
Stonewt(double lbs);
Stonewt(int stn, double lbs);
Stonewt();
// 函数定义
Stonewt::Stonewt(double lbs)
{
    stone = int(lbs) / Lbs_per_stn;
    ...
    pounds = lbs;
}
```

构造函数原型`Stonewt(double lbs)`可用于将double类型转化为Stonewt类型：

```cpp
Stonewt myCat; // 生成一个Stonewt对象
myCat = 19.6; //使用 Stonewt(double) 将 19.6 转换为 Stonewt对象
```
