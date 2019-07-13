# 一个小工程的例子：

## 目标

1. 为工程添加一个子目录 src,用来放置工程源代码;
2. 添加一个子目录 doc,用来放置这个工程的文档 hello.txt
3. 在工程目录添加文本文件 COPYRIGHT, README;
4. 在工程目录添加一个 runhello.sh 脚本,用来调用 hello 二进制
5. 将构建后的目标文件放入构建目录的 bin 子目录;
6. 最终安装这些文件:将 hello 二进制与 runhello.sh 安装至/usr/bin,将 doc 目录 的内容以及 COPYRIGHT/README 安装到/usr/share/doc/cmake/t2。

```shell
mkdir -p cmake/t2
cd cmake/t2
mkdir src
touch src/main.cpp CMakeLists.txt src/CMakeLists.txt
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



`ADD_SUBDIRECTORY(source_dir [binary_dir] [EXCLUDE_FROM_ALL])`

- 用于向当前工程添加存放源文件的子目录,并可以指定中间二进制和目标二进制存
  放的位置
- EXCLUDE_FROM_ALL 参数的含义是将这个目录从编译过程中排除
- 例子定义了将 src 子目录加入工程,并指定编译输出(包含编译中间结果)路径为build/bin 目录。如果不进行 bin 目录的指定,那么编译结果(包括中间结果)都将存放在 build/src 目录(这个目录跟原有的 src 目录对应),指定 bin 目录后,相当于在编译时将 src 重命名为 bin,所有的中间结果和目标二进制都将存放在 bin 目录。

`EXECUTABLE_OUTPUT_PATH` 和 `LIBRARY_OUTPUT_PATH`

- 以通过 SET 指令重新定义以上两个变量来指定最终的目标二进制的位置(指最终生成的 hello 或者最终的共享库,不包含编译生成的中间文件)

  ```shell
  # 可执行二进制的输出路径为 build/bin
  SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
  # 库的输出路径为 build/lib
  SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
  ```
