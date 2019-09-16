# Cmake Practice t1

```shell
mkdir -p test/t1
cd test/t1
touch main.cpp CMakeLists.txt
```

文件目录

```
t1
├── CMakeLists.txt
└── main.cpp
```

main.cpp

```cpp
#include <iostream>
int main()
{
    std::cout << "Hello World form t1 Main! \n";
    return 0;
}
```

CMakeLists.txt

```cmake
PROJECT (HELLO)
SET(SRC_LIST main.cpp)
MESSAGE(STATUS "This is BINARY dir " ${HELLO_BINARY_DIR})
MESSAGE(STATUS "This is SOURCE dir " ${HELLO_SOURCE_DIR})
ADD_EXECUTABLE(hello ${SRC_LIST}) # ADD_EXECUTABLE(hello main.cpp)
```

构建和编译并运行：

```shell
mkdir build
cd build
cmake ..
make
./hello
```

```
-- This is BINARY dir /home/likun/test/t1
-- This is SOURCE dir /home/likun/test/t1
-- Configuring done
-- Generating done
-- Build files have been written to: /home/likun/test/t1
--------------------------------------------------------------
Scanning dependencies of target hello
[ 50%] Building CXX object CMakeFiles/hello.dir/main.cpp.o
[100%] Linking CXX executable hello
[100%] Built target hello
---------------------------------------------------------------
Hello World form t1 Main! 
```

在build文件夹生成了如下文件：

```
...
cmake_install.cmake
Makefile
...
```

其中最重要就是 Makefile。

项目清理：

```shell
make clean //清除上次的make命令所产生的object文件（后缀为“.o”的文件）及可执行文件
```



${ }

- ${ } 用来引用变量,这是 cmake 的变量应用方式

PROJECT( projectname [CXX] [C] [Java])

- `projectname` 为工程名

- [CXX] [C] [Java] 指定支持的语言列表，可忽略，默认支持所有语言

- 这个指令隐式的定义了两个 cmake 变量:
  `<projectname>_BINARY_DIR `以及`<projectname>_SOURCE_DIR`,这里就是
  `HELLO_BINARY_DIR `和 `HELLO_SOURCE_DIR`(所以 CMakeLists.txt 中两个 MESSAGE
  指令可以直接使用了这两个变量)
  
  这里是采用的外部编译，`HELLO_SOURCE_DIR`代指的是工程路径，即`.../test/t1`
  
  `HELLO_BINARY_DIR `指的是编译路径，即`.../test/t1/build`

SET(VAR [VALUE] [CACHE TYPE DOCSTRING [FORCE]])

- SET命令可以用来显式的定义变量
- 如果有多个源文件,也可以定义成: `SET(SRC_LIST main.c t1.c t2.c)`

MESSAGE([SEND_ERROR | STATUS | FATAL_ERROR] "message to display"...)

- 用于向终端输出用户定义的信息,包含了三种类型:
  - SEND_ERROR,产生错误,生成过程被跳过。
  - SATUS ,输出前缀为"message to display" 的信息。
  - FATAL_ERROR,立即终止所有 cmake 过程.

ADD_EXECUTABLE(hello ${SRC_LIST})

- 定义了这个工程会生成一个文件名为 hello 的可执行文件,相关的源文件是 SRC_LIST 中
  定义的源文件列表
- 等效与 ADD_EXECUTABLE(hello main.cpp)

以上CMakeLists.txt的最简形式：

```cmake
PROJECT(HELLO)
ADD_EXECUTABLE(hello main.cpp)
```

# 基本语法

1. 变量使用${}方式取值,但是在 IF 控制语句中是直接使用变量名

2. 指令是大小写无关的,参数和变量是大小写相关的。推荐全部使用大写指令。

3. 指令(参数 1 参数 2...)
   参数使用括弧括起,参数之间使用空格或分号分开。

   ADD_EXECUTABLE(hello main.c func.c)等效于ADD_EXECUTABLE(hello main.c;func.c)

4. SET(SRC_LIST main.c)也可以写成 SET(SRC_LIST “main.c”)

5. MESSAGE(STATUS “This is BINARY dir” ${HELLO_BINARY_DIR})
   也可以写成:
   MESSAGE(STATUS “This is BINARY dir ${HELLO_BINARY_DIR}”)

6. build 目录可以建在其它地方，但是 在build目录下运行cmake需要手动添加路径

   `camke <工程的全路径>`
