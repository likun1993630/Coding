# has- a 关系

has-a关系：

- 新的类将包含另一个类的对象
- 获得实现，但不获得接口

实现方式：

- 包含
- 私有继承
- 保护继承

# 包含对象成员的类（包含）

包含（containment）

要开发一个Student类

问题描述：

- 将学生简化为姓名和一组考试分数，即类包含两个数据成员
- 姓名使用string类表示
- 分数使用valarray类表示

## valarray类简介

- 需要包含头文件valarray
- 用于数据处理
- 为模板类

```cpp
valarray<int> q_values;
valarray<double> weights;

double gpa[5] = {3.2, 2.9, 3.8, 2.9, 3.3};
valarray<double> v1; // 长度为零
valarray<int> v2(8); //8个元素
valarray<int> v3(10,8); // 8个元素，值为全10
valarray<double> v4(gpa,4); //4个元素，使用gpa的前四个元素给v4

valarray<int> v5 = {20, 32, 17, 9}; //C++11
```

- 方法：
  - operator[] : 访问各个元素
  - size(): 返回元素个数
  - sum(): 返回所有元素的总合
  - max(): 返回最大的元素
  - min(): 返回最小的元素

## 使用包含设计Student类

Student类适合建立has-a关系，即创建一个包含其他类对象的类。

```cpp
class Student
{
private:
    string name;
    valarray<double> scores;
    ...
};
```

同样，上述类将数据成员声明为私有的。这意味着Student类的成员函数可以使用string和
valarray<double>类的公有接来访问和修改name和scores对象，但在类的外面不能这样做，而只能通过Student类的公有接口访问name和score。对于这种情况，通常被描述为Student类获得了其成员对象的实现，但没有继承接口。

![1567632810553](res/1567632810553.png)

studentc.h
```cpp
// studentc.h -- defining a Student class using containment
#ifndef STUDENTC_H_
#define STUDENTC_H_

#include <iostream>
#include <string>   
#include <valarray>
class Student
{   
private:
    typedef std::valarray<double> ArrayDb;
    std::string name;       // contained object
    ArrayDb scores;         // contained object
    // private method for scores output
    std::ostream & arr_out(std::ostream & os) const;
public:
    Student() : name("Null Student"), scores() {}
    explicit Student(const std::string & s)
        : name(s), scores() {}
    explicit Student(int n) : name("Nully"), scores(n) {}
    Student(const std::string & s, int n)
        : name(s), scores(n) {}
    Student(const std::string & s, const ArrayDb & a)
        : name(s), scores(a) {}
    Student(const char * str, const double * pd, int n)
        : name(str), scores(pd, n) {}
    ~Student() {}
    double Average() const;
    const std::string & Name() const;
    double & operator[](int i);
    double operator[](int i) const;
// friends
    // input
    friend std::istream & operator>>(std::istream & is,
                                     Student & stu);  // 1 word
    friend std::istream & getline(std::istream & is,
                                  Student & stu);     // 1 line
    // output
    friend std::ostream & operator<<(std::ostream & os,
                                     const Student & stu);
};

#endif
```

- 在类定义的私有部分使用了typedef，所以类方法和友元函数可以使用ArrayDb类型，在Student类实现中也可以使用，但是在类外无法使用。
- 使用explicit 关闭构造函数的隐式转换

### 初始化被包含的对象

```cpp
Student(const char * str, const double * pd, int n)
    : name(str), scores(pd, n) {}
```

因为该构造函数初始化的是成员对象，而不是继承的对象，所以在初始化列表中使用的是成员名，而不是类名。

当初始化列表包含多个项目时，这些项目被初始化的顺序为它们被声明的顺序，而不是它们在初始化列表中的顺序：

```cpp
Student(const char * str, const double * pd, int n)
    : scores(pd, n), name(str) {}
// name 成员仍将首先被初始化，因为在类定义中它首先被声明
```

### 使用被包含对象的接口

被包含对象的接口不是公有的，但可以在类方法中使用它。

```cpp
double Student::Average() const
{
    if(scores.size() > 0)
        return scores.sum()/scores.size();
    else
        return 0;
}
// 这里，在类方法中使用了scores对象的size（）方法和sum（）方法
```

```cpp
ostream & operator<<(ostream & os, const Student & stu)
{
    os << "Scores for" << stu.name << ":\n";
    ...
}
// 这里，使用了 string类重载的 << 运算符
// 除此之外，也可以使用公有方法Name（），返回字符串来完成打印
```

注意，valarray类并没有实现 << 的定义，所以只能通过以下方式实现：

```cpp
// 私有方法 //用于辅助
ostream & Student::arr_out(ostream & os) const
{
    int i;
    int lim = scores.size();
    if(lim > 0)
    {
        for(i = 0; i<lim;i++)
        {
            os << scores[i] << "";
            if (i % 5 == 4)
                os << endl;
        }
        if(i % 5 != 0)
            os << endl;
    }
    else
        os << "empty array";
    return os;
}

ostream & operator<<(ostream & os, const Student & stu)
{
    os << "Scores for" << stu.name << ":\n";
    stu.arr_out(os);
    return os;
}
```

student.cpp

```cpp
// studentc.cpp -- Student class using containment
#include "studentc.h"
using std::ostream;
using std::endl;
using std::istream;
using std::string;

//public methods
double Student::Average() const
{
    if (scores.size() > 0)
        return scores.sum()/scores.size();  
    else
        return 0;
}

const string & Student::Name() const
{
    return name;
}

double & Student::operator[](int i)
{
    return scores[i];         // use valarray<double>::operator[]()
}

double Student::operator[](int i) const
{
    return scores[i];
}

// private method
ostream & Student::arr_out(ostream & os) const
{
    int i;
    int lim = scores.size();
    if (lim > 0)
    {
        for (i = 0; i < lim; i++)
        {
            os << scores[i] << " ";
            if (i % 5 == 4)
                os << endl;
        }
        if (i % 5 != 0)
            os << endl;
    }
    else
        os << " empty array ";
    return os; 
}

// friends

// use string version of operator>>()
istream & operator>>(istream & is, Student & stu)
{
    is >> stu.name;
    return is; 
}

// use string friend getline(ostream &, const string &)
istream & getline(istream & is, Student & stu)
{
    getline(is, stu.name);
    return is;
}

// use string version of operator<<()
ostream & operator<<(ostream & os, const Student & stu)
{
    os << "Scores for " << stu.name << ":\n";
    stu.arr_out(os);  // use private method for scores
    return os;
}
```

use_stuc.cpp

```cpp
// use_stuc.cpp -- using a composite class
// compile with studentc.cpp
#include <iostream>
#include "studentc.h"
using std::cin;
using std::cout;
using std::endl;

void set(Student & sa, int n);

const int pupils = 3;
const int quizzes = 5;

int main()
{
    Student ada[pupils] = 
        {Student(quizzes), Student(quizzes), Student(quizzes)};
    
    int i;
    for (i = 0; i < pupils; ++i)
        set(ada[i], quizzes);
    cout << "\nStudent List:\n";
    for (i = 0; i < pupils; ++i)
        cout << ada[i].Name() << endl;
    cout << "\nResults:";
    for (i = 0; i < pupils; ++i)
    {
        cout << endl << ada[i];
        cout << "average: " << ada[i].Average() << endl;
    }
    cout << "Done.\n";

    return 0;
}

void set(Student & sa, int n)
{
    cout << "Please enter the student's name: ";
    getline(cin, sa);
    cout << "Please enter " << n << " quiz scores:\n";
    for (int i = 0; i < n; i++)
        cin >> sa[i];
    while (cin.get() != '\n')
        continue; 
}
```

# 私有继承

私有继承是实现has-a的另一种方式。

- 使用私有继承，基类的公有成员和保护成员都将成为派生类的私有成员，即基类方法将不会成为派生类对象公有接口的一部分，但可以在派生类的成员函数中使用它们。
- 使用私有继承，类将继承实现，比如：如果string类派生出Student类，后者将有一个string类的组件，可用于保存字符串，Student 方法可以使用string方法来访问string组件。
- 包含将对象作为一个命名的成员对象添加到类中，而私有继承将对象作为一个未被命名的继承对象添加类中。（通过继承或包含添加的对象叫作子对象）
- 私有继承获得实现，不获得接口

## 使用私有继承设计Student类

私有继承可以使用关键字 private，也可以省略访问限定符（无限定符默认为私有继承）

使用string类和valarray类派生Student类：

```cpp
class Student : private std::string, private std::valarray<double>
{
public:
...
};
// 适用多个基类的继承被称为多重继承，后面再详细介绍
```

- Student类不需要私有数据，因为两个基类已经提供了所需的所有数据成员
- 这里的私有继承提供了两个无名称的子对象成员

### 初始化基类组件

构造函数中使用成员初始化列表时，使用类名而不是成员名来标识构造函数：

```cpp
Student(const char * str, const double * pd, int n)
    : std::string(str), ArrayDb(pd, n){}
```

