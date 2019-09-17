# Cmake Practice t2

## 目标

1. 为工程添加一个子目录 src,用来放置工程源代码;
2. 添加一个子目录 doc,用来放置这个工程的文档 hello.txt
3. 在工程目录添加文本文件 COPYRIGHT, README;
4. 在工程目录添加一个 runhello.sh 脚本,用来调用 hello 二进制
5. 将构建后的目标文件放入构建目录的 bin 子目录;
6. 最终安装这些文件:将 hello 二进制与 runhello.sh 安装至/usr/bin,将 doc 目录 的内容以及 COPYRIGHT/README 安装到/usr/share/doc/cmake/t2。

## 第一步：生成目标文件

> 使用文件目录t2_0

目录结构：

```
t2_0
├── CMakeLists.txt
└── src
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

src/CMakeLists.txt

```cmake
ADD_EXECUTABLE(hello main.cpp)
```

CMakeLists.txt

```cmake
PROJECT(HELLO)
ADD_SUBDIRECTORY(src bin)
```

编译：

编译生成的目标文件 hello 位于 .../build/bin 目录中，如果使用命令不指定bin `ADD_SUBDIRECTORY(src)`，则编译生成的目标文件hello将位于.../build/src

所以`ADD_SUBDIRECTORY(src bin)` 中定义bin的作用是，将原本应该放在src的文件放入到bin中，而`ADD_SUBDIRECTORY(src)` 命令的一个作用就是在build目录生成一个src目录。

```shell
build
├── bin
│   ├── CMakeFiles
│   ├── cmake_install.cmake
│   ├── hello # 这就是目标文件，即由main生成代
│   └── Makefile
├── CMakeCache.txt
├── CMakeFiles
│   ├── 3.5.1
│ 	....
│   ├── Makefile2
│   ├── Makefile.cmake
│   ├── progress.marks
│   └── TargetDirectories.txt
├── cmake_install.cmake
└── Makefile
```



`ADD_SUBDIRECTORY(source_dir [binary_dir] [EXCLUDE_FROM_ALL])`

- 用于向当前工程添加存放源文件的子目录,并可以指定中间二进制和目标二进制存
  放的位置
- EXCLUDE_FROM_ALL 参数的含义是将这个目录从编译过程中排除
- 例子定义了将 src 子目录加入工程,并指定编译输出(包含编译中间结果)路径为build/bin 目录。如果不进行 bin 目录的指定,那么编译结果(包括中间结果)都将存放在 build/src 目录(这个目录跟原有的 src 目录对应),指定 bin 目录后,相当于在编译时将 src 重命名为 bin,所有的中间结果和目标二进制都将存放在 bin 目录。

`EXECUTABLE_OUTPUT_PATH` 和 `LIBRARY_OUTPUT_PATH`

- 以通过 SET 指令重新定义以上两个变量来指定最终的目标二进制的位置(指最终生成的 hello 或者最终的共享库,不包含编译生成的中间文件)

- 以上两个变量要放在哪： 放在相应源文件`ADD_EXECUTABLE` 或 `ADD_LIBRARY`所在的 `CMakeLists.txt`中

  ```shell
  # 可执行二进制的输出路径为 build/bin
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
  # 库的输出路径为 build/lib
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
  
  # 这里需要放在 src/CMakeLists.txt 中
  ```

## 安装

Cmake指令：`INSTALL`

- `INSTALL` 指令用于定义安装规则,安装的内容可以包括目标二进制、动态库、静态库以及文件、目录、脚本等

- 关于`INSTALL`指令的详细介绍参见CmakePractice的 16～18 页

`CMAKE_INSTALL_PREFIX`

- 安装路径的前缀，在`INSTALL` 指定安装路径时使用相对路径时，`CMAKE_INSTALL_PREFIX`变量就会自动加在该相对路径前面。



Cmake 自定义安装的逻辑：

Cmake可以自定义一个cmake 脚本文件(也就是`<abc>.cmake` 文件)，并在安装时调用该脚本文件，用来完成想要的操作。

使用方式如下：

`INSTALL([[SCRIPT <file>] [CODE <code>]] [...])`
`SCRIPT` 参数用于在安装时调用 cmake 脚本文件(也就是`<abc>.cmake` 文件)
`CODE` 参数用于执行 CMAKE 指令,必须以双引号括起来。比如:
`INSTALL(CODE "MESSAGE(\"Sample install message.\")")`



## 第二步

> 使用文件目录t2

目录结构：

```
t2
├── CMakeLists.txt
├── COPYRIGHT
├── doc
│   └── hello.txt
├── README
├── runhello.sh
└── src
    ├── CMakeLists.txt
    └── main.cpp
```

`COPYRIGHT`，`hello.txt`，`README`文件内的内容省略（不重要）

src/main.cpp

```cpp
#include <iostream>
int main()
{
    std::cout << "Hello World form t1 Main! \n";
    return 0;
}
```

src/CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
ADD_EXECUTABLE(hello main.cpp)
INSTALL(TARGETS hello RUNTIME DESTINATION bin) # RUNTIME代表可执行二进制
# 使用相对路径，安装路径为：${CMAKE_INSTALL_PREFIX}/bin
```

CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
PROJECT(HELLO)
ADD_SUBDIRECTORY(src bin)
INSTALL(FILES COPYRIGHT README DESTINATION share/doc/cmake/t2)
INSTALL(PROGRAMS runhello.sh DESTINATION bin)
INSTALL(DIRECTORY doc/ DESTINATION share/doc/cmake/t2) # 只安装doc目录内的文件，而不是doc目录
```

runhello.sh

```
hello
```

编译：

```shell
# build 目录下
cmake -DCMAKE_INSTALL_PREFIX=~/tmp/t2/usr .. #指定安装路径的前缀
make
make install #安装
```

make install 终端打印的信息：

```
Install the project...
-- Install configuration: ""
-- Installing: /home/likun/tmp/t2/usr/share/doc/cmake/t2/COPYRIGHT
-- Installing: /home/likun/tmp/t2/usr/share/doc/cmake/t2/README
-- Installing: /home/likun/tmp/t2/usr/bin/runhello.sh
-- Up-to-date: /home/likun/tmp/t2/usr/share/doc/cmake/t2
-- Installing: /home/likun/tmp/t2/usr/share/doc/cmake/t2/hello.txt
-- Installing: /home/likun/tmp/t2/usr/bin/hello
```

安装目录：

```
➜ ~ tree tmp/t2/usr/
tmp/t2/usr/
├── bin
│   ├── hello
│   └── runhello.sh
└── share
    └── doc
        └── cmake
            └── t2
                ├── COPYRIGHT
                ├── hello.txt
                └── README
```



安装目录的其他选择

- 如果你要直接安装到系统,可以使用如下指令设置路径前缀:
  - `cmake -DCMAKE_INSTALL_PREFIX=/usr ..`

- 如果没有定义 CMAKE_INSTALL_PREFIX 会安装到什么地方?
  - `CMAKE_INSTALL_PREFIX` 的默认定义是/usr/local