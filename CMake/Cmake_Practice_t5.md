# Cmake Practice t5

## `FIND_PACKAGE`

```
FIND_PACKAGE(<name> [major.minor] [QUIET] [NO_MODULE]
[[REQUIRED|COMPONENTS] [componets...]])
```

`FIND_PACKAGE`用来调用预定义在` CMAKE_MODULE_PATH` 下的 `Find<name>.cmake` 模块,可以自己定义 `Find<name>.cmake`模块,通过 SET(CMAKE_MODULE_PATH dir)将其放入工程的某个目录中供工程使用.

> PACKAGE 包含一个项目的头文件，库等文件

- `REQUIRED `参数,其含义是指这个共享库是否是工程必须的,如果使用了这个参数,说明这
  个链接库是必备库,如果找不到这个链接库,则工程不能编译。



如果使用了预定义的`Find<name>.cmake`，则使用`FIND_PACKAGE`之后将生成如下三个变量：

- `<name>_FOUND` 判断模块是否被找到,如果没有找到,可以按照需要，以此为判断条件，关闭某些特性、给出提醒或者中止编译。

- ` <name>_INCLUDE_DIR` or ` <name>_INCLUDES` 用于将`<name>_INCLUDE_DIR` 加入 `INCLUDE_DIRECTORIES`，即添加头文件路径

- `<name>_LIBRARY` or `<name>_LIBRARIES` 用于将`<name>_LIBRARY `加入到`TARGET_LINK_LIBRARIES` 中，即将库路径用于链接

自定义`Find<name>.cmake`模块时，需要手动为这三个变量赋值，手动定义`Find<name>.cmake`模块本质上就是使用`FIND_PATH`找到头文件路径，使用`FIND_LIBRARY`找到库路径，并将这些路径赋值给相应的变量，还有一些其他的附加操作。

## `FIND_PACKAGE` 原理

首先明确一点，cmake本身不提供任何搜索库的便捷方法，所有搜索库并给变量赋值的操作必须由cmake代码完成，比如下面将要提到的FindXXX.cmake和XXXConfig.cmake。只不过，库的作者通常会提供这两个文件，以方便使用者调用。

find_package采用两种模式搜索库：

-  **Module模式**：搜索**CMAKE_MODULE_PATH**指定路径下的**FindXXX.cmake**文件，执行该文件从而找到XXX库。其中，具体查找库并给**XXX_INCLUDE_DIRS**和**XXX_LIBRARIES**两个变量赋值的操作由FindXXX.cmake模块完成。
-  **Config模式**：搜索**XXX_DIR**指定路径下的**XXXConfig.cmake**文件，执行该文件从而找到XXX库。其中具体查找库并给**XXX_INCLUDE_DIRS**和**XXX_LIBRARIES**两个变量赋值的操作由XXXConfig.cmake模块完成。

两种模式看起来似乎差不多，不过cmake默认采取Module模式，如果Module模式未找到库，才会采取Config模式。如果XXX_DIR路径下找不到XXXConfig.cmake文件，则会找/usr/lib/cmake/XXX/中的XXXConfig.cmake文件。总之，Config模式是一个备选策略。通常，库安装时会拷贝一份XXXConfig.cmake到系统目录中，因此在没有显式指定搜索路径时也可以顺利找到。

例如：

ROS包中就有XXXConfig.cmake，用与find_package指令

## 官方文档解释：

**FIND_PACKAGE**: Load settings for an external project.

```
  FIND_PACKAGE(<name> [major.minor] [QUIET] [NO_MODULE]               [[REQUIRED|COMPONENTS] [components...]])
```

Finds and loads settings from an external project. 

- `<name>_FOUND `will be set to indicate whether the package was  found.  Settings that can be used when` <name>_FOUND` is true are  package-specific.  

  The package is found through several steps.   Directories listed in `CMAKE_MODULE_PATH` are searched for files called  "Find<name>.cmake".  If such a file is found, it is read and  processed by CMake, and is responsible for finding the package.  This  first step may be skipped by using the `NO_MODULE` option.  If no such  file is found, it is expected that the package is another project built  by CMake that has a "<name>Config.cmake" file.  A cache entry  called <name>_DIR is created and is expected to be set to the  directory containing this file.  If the file is found, it is read and  processed by CMake to load the settings of the package.  If  <name>_DIR has not been set during a configure step, the command  will generate an error describing the problem unless the QUIET argument  is specified.  If <name>_DIR has been set to a directory not  containing a "<name>Config.cmake" file, an error is always  generated.  If REQUIRED is specified and the package is not found, a  FATAL_ERROR is generated and the configure step stops executing.  A  package-specific list of components may be listed after the REQUIRED  option, or after the COMPONENTS option if no REQUIRED option is given.   

