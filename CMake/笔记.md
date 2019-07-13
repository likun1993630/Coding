# 参考：
![Cmake Practice](./res/CMakePractice.pdf)

# 使用g++ 编译单个文件

```
$ touch main.cpp
$ subl main.cpp
```

main.cpp

```cpp
# include <iostream>
using namespace std;

int main(int argc, char **argv)
{
    cout << "Hello SLAM!" << endl;
    return 0;
}
```

编译：

```shell
// cd 到main所在目录
$ g++ main.cpp
// 运行生成的可执行文件
$ ./a.out
```

# 使用cmake编译main.cpp

```shell
// cd到mian.cpp 所在文件夹
$ touch CMakeLists.txt
$ subl CMakeLists.txt
```

CMakeLists.txt

```cmake
project(helloSLAM)

add_executable( sayHello main.cpp)
```

使用cmake构建

```shell
$ cmake .
```

使用make生成可执行文件

```
$ make
```
编译完成后的目录结构：
```
.
├── CMakeCache.txt
├── CMakeFiles
    ......
├── cmake_install.cmake
├── CMakeLists.txt
├── main.cpp //cpp源文件
├── Makefile
└── sayHello //可执行文件
```

## 建立build文件夹

```shell
//cd 到main.cpp 文件夹
$ mkdir build
$ cd build
$ camke ..
$ make
```
编译完成后的目录结构：
```
├── build
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   │   ├── 3.5.1
│   │   ├── cmake.check_cache
        ......
│   ├── cmake_install.cmake
│   ├── Makefile
│   └── sayHello
├── CMakeLists.txt
└── main.cpp

```

# 使用cmake管理一个简单的库

```shell
$ mkdir include src build
$ touch include/Hello.h src/Hello.cpp main.cpp CMakeLists.txt
```
文件结构：
```
.
├── build
├── CMakeLists.txt
├── include
│   └── Hello.h
├── main.cpp
└── src
    └── Hello.cpp
```

Hello.h

```cpp
#ifndef LIBHELLOSLAM_H_
#define LIBHELLOSLAM_H_

void sayHello();

#endif
```

Hello.cpp

```cpp
#include "Hello.h"
#include <iostream>

using namespace std;

void sayHello()
{
    cout << "Hello SLAM!" << endl;
}
```

main.cpp

```cpp
#include "Hello.h"
int main(int argc, char **argv)
{
    sayHello();
    return 0;
}
```

CMakeLists.txt

```cmake
project(helloSLAM)
include_directories("include")
add_library(libHello src/Hello.cpp)
add_executable(sayHello main.cpp)
target_link_libraries(sayHello libHello)
```

```shell
$ cd build
$ cmake ..
$ make
```

编译完成后的目录结构：
```
.
├── build
│   ├── CMakeCache.txt
│   ├── CMakeFiles
.....
│   ├── cmake_install.cmake
│   ├── liblibHello.a
│   ├── Makefile
│   └── sayHello
├── CMakeLists.txt
├── include
│   └── Hello.h
├── main.cpp
└── src
    └── Hello.cpp
```
