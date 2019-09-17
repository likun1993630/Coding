# Cmake Practice t3

静态库与动态库构建

## 任务

- 建立一个静态库和动态库,提供 HelloFunc 函数供其他程序编程使用,HelloFunc
  向终端输出 Hello World 字符串。
- 安装头文件与共享库。

## t3_0

目录结构：

```
t3_0
├── CMakeLists.txt
└── lib
    ├── CMakeLists.txt
    ├── hello.cpp
    └── hello.h
```

hello.cpp

```cpp
#include "hello.h"
#include <iostream>

void HelloFunc()
{	
	std::cout << "Hello World\n";
}
```

hello.h

```cpp
#ifndef HELLO_H
#define HELLO_H

#include <iostream>

void HelloFunc();

#endif
```

CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
PROJECT(HELLOLIB)
ADD_SUBDIRECTORY(lib hellolib) # 指定编译的中间文件和目标文件在build的位置
```

lib/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
SET(LIBHELLO_SRC hello.cpp)
ADD_LIBRARY(hello SHARED ${LIBHELLO_SRC}) #构建 libhello.so 共享库
```

编译：

```shell
cmake ..
make
```

编译后的build目录：

```shell
build
├── CMakeCache.txt
├── CMakeFiles
│   ├── 3.5.1
│   ├── cmake.check_cache
│   ├── CMakeDirectoryInformation.cmake
│  	....
│   └── TargetDirectories.txt
├── cmake_install.cmake
├── hellolib
│   ├── CMakeFiles
│   ├── cmake_install.cmake
│   ├── libhello.so # 共享库
│   └── Makefile
└── Makefile
```



**指定 libhello.so 生成的位置**

- `ADD_SUBDIRECTORY(lib <路径>)`指定...build/<路径>
- 在 lib/CMakeLists.txt 中添加`SET(LIBRARY_OUTPUT_PATH <路径>)`来指定一个新的位置

## 指令 ADD_LIBRARY

```
ADD_LIBRARY(libname
[SHARED|STATIC|MODULE]
[EXCLUDE_FROM_ALL]
source1 source2 ... sourceN)
```

- cmake会将`libname`自动补全为`liblibname.so`或`liblibname.a`

- SHARED,动态库
- STATIC,静态库
- MODULE,在使用 dyld 的系统有效,如果不支持 dyld,则被当作 SHARED 对待。
- EXCLUDE_FROM_ALL 参数的意思是这个库不会被默认构建,除非有其他的组件依赖或者手
  工构建

## 添加静态库

- 按照一般的习惯,静态库名字跟动态库名字应该是一致的,只不过后缀是.a 

  - 即如果动态库为libhello.so,则对应的静态库应给为libhello.a

- 如果只想构建一个静态库，而不构建动态库可以使用指令：

  `ADD_LIBRARY(hello STATIC ${LIBHELLO_SRC})`

- 如果想构建同名的动代库和静态库需要特殊的操作，即使用`SET_TARGET_PROPERTIES`

**指令`SET_TARGET_PROPERTIES`**

```
SET_TARGET_PROPERTIES(target1 target2 ...
						PROPERTIES prop1 value1
						prop2 value2 ...)
```

- 用来设置target输出的名称
- 指定动态库版本和 API 版本

例如：

```cmake
ADD_LIBRARY(hello_static STATIC ${LIBHELLO_SRC})
SET_TARGET_PROPERTIES(hello_static PROPERTIES OUTPUT_NAME "hello")
# 将本来要以名称hello_static生成的库改名为hello
```

> 注意: 使用`OUTPUT_NAME`属性还不足以同时生成同名的静态库和动态库，因为cmake 在构建一个新的 target 时,会尝试清理掉其他使用这个名字的库,因为,在构建 libhello.a 时,就会清理掉 libhello.so.
>
> 需要用到`CLEAN_DIRECT_OUTPUT`属性

例如：

```cmake
# 继续添加
SET_TARGET_PROPERTIES(hello PROPERTIES CLEAN_DIRECT_OUTPUT 1)
SET_TARGET_PROPERTIES(hello_static PROPERTIES CLEAN_DIRECT_OUTPUT 1)
```

**指定动态库版本**

例如：

```cmake
SET_TARGET_PROPERTIES(hello PROPERTIES VERSION 1.2 SOVERSION 1)
# VERSION 指代动态库版本,SOVERSION 指代 API 版本
```

构建之后会在build/lib下生成：

```
libhello.so.1.2
libhello.so.1->libhello.so.1.2
libhello.so ->libhello.so.1
```

## t3

目录结构：

```
t3
├── CMakeLists.txt
└── lib
    ├── CMakeLists.txt
    ├── hello.cpp
    └── hello.h
```

hello.cpp

```cpp
#include "hello.h"
#include <iostream>

void HelloFunc()
{	
	std::cout << "Hello World\n";
}
```

hello.h

```cpp
#ifndef HELLO_H
#define HELLO_H

#include <iostream>

void HelloFunc();

#endif
```

CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
PROJECT(HELLOLIB)
ADD_SUBDIRECTORY(lib hellolib) # 指定编译的中间文件和目标文件在build的位置
```

lib/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
SET(LIBHELLO_SRC hello.cpp)
ADD_LIBRARY(hello SHARED ${LIBHELLO_SRC})
ADD_LIBRARY(hello_static STATIC ${LIBHELLO_SRC})

SET_TARGET_PROPERTIES(hello_static PROPERTIES OUTPUT_NAME "hello")
SET_TARGET_PROPERTIES(hello PROPERTIES CLEAN_DIRECT_OUTPUT 1) #不清理同名库
SET_TARGET_PROPERTIES(hello_static PROPERTIES CLEAN_DIRECT_OUTPUT 1) #不清理同名库

SET_TARGET_PROPERTIES(hello PROPERTIES VERSION 1.2 SOVERSION 1) #指定动态库版本

# 将libhello.a, libhello.so.x 以及 hello.h 安装到系统目录,才能真正让其他人开发使用
# 将 hello 的共享库和静态库安装到<prefix>/lib目录,
# 将 hello.h 安装到<prefix>/include/hello目录
INSTALL(TARGETS hello hello_static LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
INSTALL(FILES hello.h DESTINATION include/hello)
```

编译：

```shell
# 这里其实因该安装在 usr/ 下面，才算是标准路径
# 安装在~/tmp/t3/usr只是为了方便观察目录结构
cmake -DCMAKE_INSTALL_PREFIX=~/tmp/t3/usr ..
make
make install
```

```
➜ build git:(master) ✗ make install                                
[ 50%] Built target hello
[100%] Built target hello_static
Install the project...
-- Install configuration: ""
-- Installing: /home/likun/tmp/t3/usr/lib/libhello.so.1.2
-- Installing: /home/likun/tmp/t3/usr/lib/libhello.so.1
-- Up-to-date: /home/likun/tmp/t3/usr/lib/libhello.so
-- Up-to-date: /home/likun/tmp/t3/usr/lib/libhello.a
-- Up-to-date: /home/likun/tmp/t3/usr/include/hello/hello.h
```

安装目录的结构：

```
tmp/t3/usr
├── include
│   └── hello
│       └── hello.h
└── lib
    ├── libhello.a
    ├── libhello.so -> libhello.so.1
    ├── libhello.so.1 -> libhello.so.1.2
    └── libhello.so.1.2
```

