# Cmake Practice t4

## t4

前提：

项目t3的安装位置：

```
/usr
├── include
│   └── hello
│       └── hello.h
└── lib
    ├── libhello.a
    ├── libhello.so -> libhello.so.1
    ├── libhello.so.1 -> libhello.so.1.2
    └── libhello.so.1.2
```

目录结构：

```
t4
├── CMakeLists.txt
└── src
    ├── CMakeLists.txt
    └── main.cpp
```

main.cpp

```cpp
#include <hello.h>
int main()
{
	HelloFunc();
	return 0;
}
```

CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
PROJECT(NEWHELLO)
ADD_SUBDIRECTORY(src)
```

src/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
INCLUDE_DIRECTORIES(/usr/include/hello) # 包含非标准头文件路径
ADD_EXECUTABLE(main main.cpp)
TARGET_LINK_LIBRARIES(main hello) # 链接动态库
# 如果要链接动态库，可以写 hello，也可以写 libhello.so, cmake会自动补全
# 如果要链接静态库，需要写成 libhello.a
```

> 注意 /usr/include 是标准头文件路径，但是/usr/include/hello 不是
>
> 如果在源代码中使用`include <hello/hello.h>`，就不需要`INCLUDE_DIRECTORIES`，因为编译器可以在标准目录中找到hello文件夹和文件夹内的hello.h



## 添加共享库

`LINK_DIRECTORIES`

```
LINK_DIRECTORIES(directory1 directory2 ...)
```

- 添加非标准的共享库搜索路径,比如,在工程内部同时存在共享库和可执行二进制,在编译时就需要指定一下这些共享库的路径

`TARGET_LINK_LIBRARIES`

```
TARGET_LINK_LIBRARIES(target library1
<debug | optimized> library2
...)
```

- 比如

  ```cmake
  # 链接动态库
  TARGET_LINK_LIBRARIES(main libhello.so)
  # 或
  TARGET_LINK_LIBRARIES(main hello)
  
  # 链接静态库
  TARGET_LINK_LIBRARIES(main libhello.a)
  ```

### 查看目标文件链接关系

```shell
ldd src/main
```

```
...
libhello.so.1 => /usr/lib/libhello.so.1 (0x00007fc0b83f7000)
...
```

### 智能查找头文件和库路径

`FIND_PATH`

```shell
FIND_PATH(<VAR> name1 path1 path2 ...)
# VAR 变量代表包含这个文件的路径
```

使用`FIND_PATH`命令可以查找指定目录下的所需文件，并将该文件的绝对路径赋值给变量，然后可以引用该变量。

比如：

```cmake
FIND_PATH(myHeader NAMES hello.h PATHS /usr/include /usr/include/hello)
IF(myHeader) # 判断文件是否被找到
INCLUDE_DIRECTORIES(${myHeader}) # 使用找到的路径
ENDIF(myHeader)
```

使用`FIND_PATH`命令时，也可以在终端指定要被搜索的路径，相关的两个shell环境变量：

`CMAKE_INCLUDE_PATH` 和 `CMAKE_LIBRARY_PATH`

- `CMAKE_INCLUDE_PATH`  用于头文件的路径
-  `CMAKE_LIBRARY_PATH` 用于库的路径

> 这两个变量默认不存在，需要在shell使用命令将变量导出为shell环境变量

比如：

```shell
# shell 执行
export CMAKE_INCLUDE_PATH=/usr/include/hello
```

```cmake
# CMakeLists.txt 加入
FIND_PATH(myHeader hello.h) #FIND_PATH将在变量CMAKE_INCLUDE_PATH指定的路径内寻找头文件
IF(myHeader)
INCLUDE_DIRECTORIES(${myHeader})
ENDIF(myHeader)
```

`FIND_LIBRARY`

```shell
FIND_LIBRARY(<VAR> name1 path1 path2 ...)
# VAR 变量表示找到的库全路径,包含库文件名
```

比如：

```cmake
FIND_LIBRARY(libX X11 /usr/lib)
IF(NOT libX)
MESSAGE(FATAL_ERROR “libX not found”)
ENDIF(NOT libX)
```

`CMAKE_LIBRARY_PATH `可以用在 `FIND_LIBRARY`

当导出了`CMAKE_LIBRARY_PATH `变量之后，`FIND_LIBRARY` 默认在该变量指定的路径内寻找库，比如：

```cmake
FIND_LIBRARY(mylib hellib) # 此时即会使用CMAKE_LIBRARY_PATH的路径
```

