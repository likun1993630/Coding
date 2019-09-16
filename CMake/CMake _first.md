# 使用Cmake构建的本质

现有一个项目包含三个文件：

- main.cpp 
- sum.cpp
- sum.h

sum函数在sum.h头文件内声明，在sum.cpp中定义。

main.cpp 中使用`#include <sum.h> `

sum.cpp有可能被构建为库文件（libsum.so/ libsum.a）

本质：

通过各种操作告诉Cmake这些文件都在哪里，需要生成哪些库文件，然后main.cpp需要和哪些文件或者库文件链接以生成最终的可执行文件 main。

另外还可以命令Cmake将上面提到的文件复制到某些特定的或者自定义的文件夹，即安装

头文件要点：

- 头文件只需知道路径
- 在include中使用相对路径指明头文件位置
- 或，使用cmake命令 `INCLUDE_DIRECTORIES`

## 文件内容：

main.cpp

```cpp
#include <iostream>
#include "sum.h"

int main()
{
    int Num1 = 1;
    int Num2 = 2;
    int iRet = 0;

    iRet = Sum(Num1, Num2);
    std::cout << "Num1 + Num2 = " << iRet << std::endl;
    return 0;
}
```

sum.h

```cpp
int Sum(int Number1, int Number2);
```

sum.cpp

```cpp
int Sum(int Number1, int Number2)
{
    return Number1 + Number2;
}
```

# 目录结构变式

## P1

所有文件都在同一个目录下：

p1
├── CMakeLists.txt
├── main.cpp
├── sum.cpp
└── sum.h

CMakeLists.txt

```cmake
cmake_minimum_required (VERSION 2.8)
PROJECT (SUM)
ADD_EXECUTABLE(main sum.cpp main.cpp)
```

## P2

```
p2
├── CMakeLists.txt
├── include
│   └── sum.h
├── main.cpp
└── sum.cpp
```

CMakeLists.txt

```cmake
cmake_minimum_required (VERSION 2.8)
PROJECT (SUM)
INCLUDE_DIRECTORIES(include) # 用于包含头文件的路径
ADD_EXECUTABLE(main sum.cpp main.cpp)
```

## P3

```
p3
├── CMakeLists.txt
├── include
│   └── sum.h
├── main.cpp
└── src
    └── sum.cpp
```

CMakeLists.txt

```cmake
cmake_minimum_required (VERSION 2.8)
PROJECT (SUM)
INCLUDE_DIRECTORIES(include)
ADD_EXECUTABLE(main src/sum.cpp main.cpp) #直接指定各个文件的路径
```

## P4

```
p4
├── CMakeLists.txt
├── include
│   └── sum.h
└── src
    ├── main.cpp
    └── sum.cpp
```

CMakeLists.txt

```cmake
cmake_minimum_required (VERSION 2.8)
PROJECT (SUM)
INCLUDE_DIRECTORIES(include)
ADD_EXECUTABLE(main src/sum.cpp src/main.cpp) #直接指定各个文件的路径
```

## P5

在根目录和src目录各有一个CMakeLists.txt，构建的方式可以是：

将src/sum.cpp先构建成一个共享库，然后再与main链接生成最终的可执行文件。

```
p5
├── CMakeLists.txt
├── include
│   └── sum.h
├── main.cpp
└── src
    ├── CMakeLists.txt
    └── sum.cpp
```

CMakeLists.txt

```cmake
cmake_minimum_required (VERSION 2.8)
PROJECT (SUM)
INCLUDE_DIRECTORIES(include)
ADD_SUBDIRECTORY(src) #将src加入构建项目，并且在构建build目录生成src
ADD_EXECUTABLE(main main.cpp)
TARGET_LINK_LIBRARIES(main sum)
```

src/CMakeLists.txt

```cmake
add_library(sum SHARED sum.cpp)
```

## P6

```
p6
├── CMakeLists.txt
├── main.cpp
└── src
    ├── CMakeLists.txt
    ├── sum.cpp
    └── sum.h
```

CMakeLists.txt

```cmake
cmake_minimum_required (VERSION 2.8)
PROJECT (SUM)
ADD_SUBDIRECTORY(src)
INCLUDE_DIRECTORIES(src)
ADD_EXECUTABLE(main main.cpp)
TARGET_LINK_LIBRARIES(main sum)
```

src/CMakeLists.txt

```cmake
add_library(sum SHARED sum.cpp)
```

## P7

特殊，对main.cpp 的include语句进行的修改，借此可以省略指令：

`INCLUDE_DIRECTORIES`

main.cpp

```cpp
#include <iostream>
#include "src/sum.h" //指定头文件相对路径

int main()
{
    int Num1 = 1;
    int Num2 = 2;
    int iRet = 0;

    iRet = Sum(Num1, Num2);
    std::cout << "Num1 + Num2 = " << iRet << std::endl;
    return 0;
}
```

目录结构：

```
p7
├── CMakeLists.txt
├── main.cpp
└── src
    ├── CMakeLists.txt
    ├── sum.cpp
    └── sum.h
```

CMakeLists.txt

```cmake
cmake_minimum_required (VERSION 2.8)
PROJECT (SUM)
ADD_SUBDIRECTORY(src)
ADD_EXECUTABLE(main main.cpp)
TARGET_LINK_LIBRARIES(main sum)
```

src/CMakeLists.txt

```cmake
add_library(sum SHARED sum.cpp)
```





