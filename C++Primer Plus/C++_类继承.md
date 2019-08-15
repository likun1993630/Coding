# 类继承

C++使用继承来扩展和修改类。它能够从已有的类派生出新的类，而派生类继承了原有类（称为基类）的特征，包括方法。

类的继承可以实现：

- 可以在己有类的基础上添加功能。例如，对于数组类，可以添加数学运算。
- 可以给类添加数据。例如，对于字符串类，可以派生出一个类，并添加指定字符串显示颜色的数
  据成员。
- 可以修改类方法的行为。例如，对于代表提供给飞机乘客的服务的passenger类，可以派生出提供
  更高级别服务的FirstClassPassenger类。

当然，可以通过复制原始类代码，并对其进行修改来完成上述工作，但继承机制只需提供新特性，甚至不需要访问源代码就可以派生出类。因此，如果购买的类库只提供了类方法的头文件和编译后代码，仍可以使用库中的类派生出新的类。

## 一个简单的基类

从一个类派生出另一个类时，原始类称为基类，继承类称为派生类

### TableTennisPlayer类

tabtenn0.h

```cpp
// tabtenn0.h -- a table-tennis base class
#ifndef TABTENN0_H_
#define TABTENN0_H_
#include <string>
using std::string;
// 简单的基类
class TableTennisPlayer
{
private:
    string firstname;
    string lastname;
    bool hasTable;
public:
    TableTennisPlayer (const string & fn = "none",
                       const string & ln = "none", bool ht = false);
    void Name() const;
    bool HasTable() const { return hasTable; };
    void ResetTable(bool v) { hasTable = v; };
};
#endif
```

- 使用string类来存储姓名，相比于使用字符数组，更方便灵活和安全

tabtenn0.cpp

```cpp
//tabtenn0.cpp -- simple base-class methods
#include "tabtenn0.h"
#include <iostream>

// 使用成员初始化列表
// 也可以使用普通的初始化方式
TableTennisPlayer::TableTennisPlayer (const string & fn, 
    const string & ln, bool ht) : firstname(fn),
	    lastname(ln), hasTable(ht) {}
    
void TableTennisPlayer::Name() const
{
    std::cout << lastname << ", " << firstname;
}
```

usett0.cpp

````cpp
// usett0.cpp -- using a base class
#include <iostream>
#include "tabtenn0.h"

int main ( void )
{
    using std::cout;
    TableTennisPlayer player1("Chuck", "Blizzard", true);
    TableTennisPlayer player2("Tara", "Boomdea", false);
    player1.Name();
    if (player1.HasTable())
        cout << ": has a table.\n";
    else
        cout << ": hasn't a table.\n";
    player2.Name();
    if (player2.HasTable())
        cout << ": has a table";
    else
        cout << ": hasn't a table.\n";
    return 0;
}
````

- 可以使用C风格字符串初始化string对象：原因，string类有一个将const char * 作为参数的构造函数。

### 派生一个类

从基类TableTennisClass类派生一个RatedPalyer，RatedPlayer包括成员在比赛中的比分

```cpp
// RatedPalyer 由基类 TableTennisClass 派生而来
class RatedPlayer : public TableTennisClass
{
 ...
};
```

- public 表明TableTennisClass 为公有基类，这被称为公有派生
- 公有派生特征：
  - 派生类对象包含基类对象
  - 基类的公有成员将成为派生类的公有成员
  - 基类的私有部分也将成为派生类的一部分，但只能过基类的公有和保护方法访问
  - 派生类对象存储了基类的数据成员（派生类继承了基类的实现）
  - 派生类对象可以使用基类的方法（派生类继承了基类的接口）
- RatedPlayer对象可以存储名字和是否由桌球，可以使用Name（），hasTable（），ResetTable（） 方法

派生类的其他要求：

- 派生类需要自己的构造函数
- 派生类可以根据需要添加额外的数据成员和成员函数

```cpp
class RatedPlayer : public TableTennisPlayer
{
private:
	unsigned int rating; // 添加数据成员
public:
	RatedPlayer(unsigned int r=0, const string & fn = "none", 
			const string & ln = "none", bool ht = false);
	RatedPlayer(unsigned int r, const TableTennisPlayer & tp);
	unsigned int Rating() const {return rating; } //添加方法
	void ResetRating(unsigned int r) {rating = r; } //添加方法
};
```

- 派生类的构造函数必须给新成员和继承的成员提供数据

### 构造函数：访问权限的考虑

派生类不能直接访问基类的私有成员，而必须通过基类方法进行访问。

例如，RatedPlayer构造函数不能直接设置继承的成员（firstname、lastname和hasTable），而必须使用基类的公有方法来访问私有的基类成员。具体地说，派生类构造函数必须使用基类构造函数。

创建派生类对象时，程序首先创建基类对象。从概念上说，这意味着基类对象应当在程序进入派生类构造函数之前被创建。C++使用成员初始化列表语法来完成这种工作。

```cpp
RatedPlayer:RatedPlayer(unsigned int r, const string & fn,const string & ln,
					bool ht) : TableTennisPlayer(fn, ln, ht)
{
    rating = r;
}
```

- `TableTennisPlayer(fn, ln, ht)` 是成员初始化列表，为可执行代码，它将调用TableTennisPlayer的构造函数。

但省略成员初始化列表时，程序将使用默认的基类的构造函数。除非要使用基类的默认构造函数，否则应该显式调用正确的基类构造函数。

```cpp
RatedPlayer:RatedPlayer(unsigned int r, const string & fn,const string & ln,
					bool ht)  //无成员初始化列表
{
    rating = r;
}
// 等效于：
RatedPlayer:RatedPlayer(unsigned int r, const string & fn,const string & ln,
					bool ht)  : TableTennisPlayer()
{
    rating = r;
}
```

第二个构造函数的含义

```cpp
RatedPlayer:RatedPlayer(unsigned int r, const TableTennisPlayer & tp) 
    : TableTennisPlayer(tp)
{
    rating = r;
}
```

- 此处，tp的类型为 TableTennisPlayer& ，因此将调用基类的复制构造函数，但基类没有显式的定义复制构造函数，所以会调用默认复制构造函数，但是不会产生问题。（虽然基类中使用了string类对象，并且string类使用了动态内存分配，但基类在使用默认复制构造函数时，对于string类对象会调用string类的复制构造函数来复制string成员，所以不会产生问题。）

可以对派生类成员使用成员初始化列表语法，所以第二个构造函数可以写成如下形式：

```cpp
// 第二个构造函数的另一种写法
RatedPlayer:RatedPlayer(unsigned int r, const TableTennisPlayer & tp) 
    : TableTennisPlayer(tp), rating(r) {}
```

派生类构造函数的要点：

- 首先创建基类对象
- 派生类构造函数应通过成员初始化列表将基类信息传递给基类构造函数
- 派生类构造函数应初始化派生类新增的数据成员

派生类对象的创建过程：

创建派生类对象时，程序首先调用基类构造函数，然后再调用派生类构造函数。基类构造函数负责初始化继承的数据成员；派生类构造函数主要用于初始化新增的数据成员。派生类的构造函数总是调用一个基类构造函数。可以使用初始化器列表语法指明要使用的基类构造函数，否则将使用默认的基类构造函数。
派生类对象过期时，程序将首先调用派生类析构函数，然后再调用基类析构函数。

### 使用派生类

要使用派生类，程序必须能够访问基类的声明。可以将每个类放到独立的头文件中，但由于这两个类是相关的，所以把其类声明放到一起更合适。

tabtenn1.h

```cpp
// tabtenn1.h -- a table-tennis base class
#ifndef TABTENN1_H_
#define TABTENN1_H_
#include <string>
using std::string;
// simple base class
class TableTennisPlayer
{
private:
    string firstname;
    string lastname;
    bool hasTable;
public:
    TableTennisPlayer (const string & fn = "none",
                       const string & ln = "none", bool ht = false);
    void Name() const;
    bool HasTable() const { return hasTable; };
    void ResetTable(bool v) { hasTable = v; };
};

// simple derived class
class RatedPlayer : public TableTennisPlayer
{
private:
    unsigned int rating;
public:
    RatedPlayer (unsigned int r = 0, const string & fn = "none",
                 const string & ln = "none", bool ht = false);
    RatedPlayer(unsigned int r, const TableTennisPlayer & tp);
    unsigned int Rating() const { return rating; }
    void ResetRating (unsigned int r) {rating = r;}
};

#endif
```

tabtenn1.cpp

```cpp
//tabtenn1.cpp -- simple base-class methods
#include "tabtenn1.h"
#include <iostream>

TableTennisPlayer::TableTennisPlayer (const string & fn, 
    const string & ln, bool ht) : firstname(fn),
	    lastname(ln), hasTable(ht) {}
    
void TableTennisPlayer::Name() const
{
    std::cout << lastname << ", " << firstname;
}

// RatedPlayer methods
RatedPlayer::RatedPlayer(unsigned int r, const string & fn,
     const string & ln, bool ht) : TableTennisPlayer(fn, ln, ht)
{
    rating = r;
}

RatedPlayer::RatedPlayer(unsigned int r, const TableTennisPlayer & tp)
    : TableTennisPlayer(tp), rating(r) {}
```

usett1.cpp

```cpp
// usett1.cpp -- using base class and derived class
#include <iostream>
#include "tabtenn1.h"

int main ( void )
{
    using std::cout;
    using std::endl;
    TableTennisPlayer player1("Tara", "Boomdea", false);
    RatedPlayer rplayer1(1140, "Mallory", "Duck", true);
    rplayer1.Name();          // derived object uses base method
    if (rplayer1.HasTable())
        cout << ": has a table.\n";
    else
        cout << ": hasn't a table.\n";
    player1.Name();           // base object uses base method
    if (player1.HasTable())
        cout << ": has a table";
    else
        cout << ": hasn't a table.\n";
    cout << "Name: ";
    rplayer1.Name();
    cout << "; Rating: " << rplayer1.Rating() << endl;
// initialize RatedPlayer using TableTennisPlayer object
    RatedPlayer rplayer2(1212, player1);
    cout << "Name: ";
    rplayer2.Name();
    cout << "; Rating: " << rplayer2.Rating() << endl;
    return 0;
}
```

```
Duck, Mallory: has a table.
Boomdea, Tara: hasn't a table.
Name: Duck, Mallory; Rating: 1140
Name: Boomdea, Tara; Rating: 1212
```

### 派生类和基类之间的特殊关系

- 派生类对象可以使用基类的方法，条件是基类方法不是私有的

  ```cpp
  RatedPlayer rplayer1(1140, "Mallory", "Duck", true);
  rplayer1.Name();
  ```

- 基类指针可以在不进行显式类型转换的情况下指向派生类对象（2）

- 基类引用可以在不进行显示类型转换的情况下引用派生类对象（3）

  ```cpp
  RatedPlayer rplayer1(1140, "Mallory", "Duck", true);
  TableTennisPlayer & rt = rplayer1;
  TableTennisPlayer * pt = &rplayer1;
  rt.Name(); //调用 Name()
  pt->Name(); //调用 Name()
  ```

- 基类指针或引用只能用于调用基类方法

  - 例如，不能使使用基类的引用或指针 rt，pt来调用派生类的 ResetRanking方法

- 不能将基类对象和地址赋给派生类引用和指针

  ```cpp
  TableTennisPlayer player1("Tara", "Boomdea", false);
  RatedPlayer & rr = player; //禁止
  RatedPlayer * pr = player; //禁止
  ```

- 对于形参为基类引用的函数可用于基类对象或派生类对象（由基类引用或指针可以指向派生类对象延伸,即2，3条的延伸）

  ```cpp
  void Show(const TableTennisPlayer & rt)
  {
      using std::cout;
      cout << "Name:";
      rt.Name();
      cout << "\nTable:";
      if (rt.HasTable())
          cout << "yes\n";
      else
          cout << "no\n";
  }
  // 形参rt是一个基类引用，它可以指向基类对象或派生类对象，所以可以在Show()中使用TableTennis参数或 RatedPlayer 参数：
  TableTennisPlayer player1("Tara", "Boomdea", false);
  RatedPlayer rplayer1(1140, "Mallory", "Duck", true);
  Show(player1);
  Show(rplayer1);
  ```

- 对于形参为指向基类的指针的函数可用于基类对象或派生类对象（由基类引用或指针可以指向派生类对象延伸,即2，3条的延伸）

  ```cpp
  void Wohs(const TableTennisPlayer * pt);
  ...
  TableTennisPlayer player1("Tara", "Boomdea", false);
  RatedPlayer rplayer1(1140, "Mallory", "Duck", true);
  Wohs(&player1);
  Wohs(&rplayer1);
  ```

- 可以使用派生类对象初始化基类对象（相当于舍弃了派生类对象里的部分信息）

  ```cpp
  RatedPlayer olaf1(1140, "Olaf", "Duck", true);
  TableTennisPlayer olaf2(olaf1);  //使用派生类对象初始化基类对象
  
  // 要初始化olaf2，匹配的构造函数的原型如下：
  TableTennisPlayer(const RatedPlayer &); // 但是这种原型并不存在
  // 类中并没有定义上述构造函数，但存在隐式复制构造函数：
  TableTennisPlayer(const TableTennisPlayer &); 
  // 因为形参是基类引用，因此该函数可以用于派生类，即可以引用派生类
  ```

- 可以将派生类对象赋值给基类对象（相当于舍弃了派生类对象里的部分信息）

  ```cpp
  RatedPlayer olaf1(1140, "Olaf", "Duck", true);
  TableTennisPlayer winner;
  winner = olaf1; //将派生类对象赋值给基类对象
  //即将olaf1的基类部分复制给winner
  
  // 这种情况下程序将使用隐式重载赋值运算符：
  TableTennisPlayer & operator=(const TableTennisPlayer &) const;
  ```

## 继承：is-a 关系

