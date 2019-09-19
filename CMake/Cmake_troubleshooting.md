# 案例1

#### 问题描述

已经成功编译了深度学习框架Caffe，例程也可以顺利执行。

但是当我在自己的代码中调用编译好的Caffe库时，却出现了编译错误。此前，我已经在CMakeLists.txt中添加了下面几句话：

```cmake
include_directories(/home/wjg/projects/caffe/build/install/include)
add_executable(useSSD ssd_detect.cpp)
target_link_libraries(useSSD /home/wjg/projects/caffe/build/install/lib/libcaffe.so)
```

执行make后，链接出错，找不到libboost_system.so文件。

这一错误倒是给我提了个醒，我本以为自己的代码中没用到boost，就不必添加boost库路径了，谁知道libcaffe.so中用到的库也需要手动添加进去。

这时候我才意识到动态链接库和静态链接库的区别。前者在程序运行时动态加载，而后者是在编译时就和程序结合到一起了。于是动态链接库即使编译完成，也和其它动态库是分离的，因此每次用都要把所有涉及的动态库全部添加进来。在我的例子中，不仅仅需要添加boost，还有atlas、protobuf等等一大堆动态链接库需要添加。这个时候，一条条添加就显得太过麻烦，可以借助find_package命令一次性添加所有与Caffe相关的动态链接库。

#### find_package用法

使用如下方式查找Caffe库：

```cmake
find_package(Caffe REQUIRED)
```

如果找到Caffe库，就可以在接下来的语句中使用**Caffe_INCLUDE_DIRS**和**Caffe_LIBRARIES**这两个变量，比如

```cmake
find_package(Caffe REQUIRED)

if (NOT Caffe_FOUND)
    message(FATAL_ERROR "Caffe Not Found!")
endif (NOT Caffe_FOUND)

include_directories(${Caffe_INCLUDE_DIRS})

add_executable(useSSD ssd_detect.cpp)
target_link_libraries(useSSD ${Caffe_LIBRARIES})
```

问题是，很多情况下都会找不着，或者找到了错误的位置。要想用对find_package，就需要了解它的工作原理。

#### find_package原理

首先明确一点，cmake本身不提供任何搜索库的便捷方法，所有搜索库并给变量赋值的操作必须由cmake代码完成，比如下面将要提到的FindXXX.cmake和XXXConfig.cmake。只不过，库的作者通常会提供这两个文件，以方便使用者调用。

find_package采用两种模式搜索库：

-  **Module模式**：搜索**CMAKE_MODULE_PATH**指定路径下的**FindXXX.cmake**文件，执行该文件从而找到XXX库。其中，具体查找库并给**XXX_INCLUDE_DIRS**和**XXX_LIBRARIES**两个变量赋值的操作由FindXXX.cmake模块完成。
-  **Config模式**：搜索**XXX_DIR**指定路径下的**XXXConfig.cmake**文件，执行该文件从而找到XXX库。其中具体查找库并给**XXX_INCLUDE_DIRS**和**XXX_LIBRARIES**两个变量赋值的操作由XXXConfig.cmake模块完成。

两种模式看起来似乎差不多，不过cmake默认采取Module模式，如果Module模式未找到库，才会采取Config模式。如果XXX_DIR路径下找不到XXXConfig.cmake文件，则会找/usr/local/lib/cmake/XXX/中的XXXConfig.cmake文件。总之，Config模式是一个备选策略。通常，库安装时会拷贝一份XXXConfig.cmake到系统目录中，因此在没有显式指定搜索路径时也可以顺利找到。

在我遇到的问题中，由于Caffe安装时没有安装到系统目录，因此无法自动找到CaffeConfig.cmake，我在CMakeLists.txt最前面添加了一句话之后就可以了。

```cmake
set(Caffe_DIR /home/wjg/projects/caffe/build)   #添加CaffeConfig.cmake的搜索路径

find_package(Caffe REQUIRED)

if (NOT Caffe_FOUND)
    message(FATAL_ERROR "Caffe Not Found!")
endif (NOT Caffe_FOUND)

include_directories(${Caffe_INCLUDE_DIRS})

add_executable(useSSD ssd_detect.cpp)
target_link_libraries(useSSD ${Caffe_LIBRARIES})
```

其实关于find_package还有许多知识点，可惜我也没能全部掌握。XXXConfig.cmake的默认搜索路径也不止一个，它们有详细的优先级顺序。对于库的开发者来说，如何生成FindXXX.cmake或XXXConfig.cmake文件更是一个复杂工程，需要了解更多的知识，希望以后有机会再深入了解。



# 案例2

