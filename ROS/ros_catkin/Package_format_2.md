# Package format 2 (recommended)

[原文地址](https://docs.ros.org/api/catkin/html/howto/format2/index.html)

## 总结

package.xml 

- package.xml 对自己没有影响，只对要安装你的包的人有影响
- package.xml只是为了确保，别人安装你的包之后，使用rosdep能够安装你的包所依赖的东西，进而使别人能够用你的包导出的头文件和库等。
- 如果不知道使用哪种标签，就用 `<depend> </depend>`

# Catkin configuration overview

ROS and other packages may be configured and built using catkin. Every catkin package must include `package.xml` and `CMakeLists.txt` files in its top-level directory.

## package.xml

Your package must contain an XML file named [package.xml](http://wiki.ros.org/catkin/package.xml), as specified by [REP-0140](http://ros.org/reps/rep-0140.html).  These components are all required:

```xml
<package format="2">
  <name>your_package</name>
  <version>1.2.4</version>
  <description>
    This package adds extra features to rosawesome.
  </description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>BSD</license>
  <buildtool_depend>catkin</buildtool_depend>
</package>
```

Substitute your name, e-mail and the actual name of your package, and please write a better description.

The maintainer is who releases the package, not necessarily the original author.  You should generally add one or more `<author>` tags, giving appropriate credit:

```xml
<author>Dennis Richey</author>
<author>Ken Thompson</author>
```

Also, please provide some URL tags to help users find documentation and report problems:

```xml
<url type="website">http://ros.org/wiki/camera1394</url>
<url type="repository">https://github.com/ros-drivers/camera1394.git</url>
<url type="bugtracker">https://github.com/ros-drivers/camera1394/issues</url>
```

## Metapackages

These are special-purpose catkin packages for grouping other packages. Users who install a metapackage binary will also get all packages directly or indirectly included in that group.  Metapackages must not install any code or other files, the `package.xml` gets installed automatically.  They can depend on other metapackages, if desired, but regular catkin packages may not.

Metapackages can be used to resolve [stack](http://wiki.ros.org/Stacks) dependencies declared by legacy [rosbuild](http://wiki.ros.org/rosbuild) packages not yet converted to catkin.  Catkin packages should depend directly on the packages they use, not on any metapackages.

A good use for metapackages is to group the major components of your robot and then provide a comprehensive grouping for your whole system.

In addition to the XML elements mentioned above, a metapackage `package.xml` must contain this:

```xml
<export>
  <metapackage/>
  <architecture_independent/>
</export>
```

In addition to the required `<buildtool_depend>` for catkin, metapackages list the packages in the group using `<exec_depend>` tags:

```xml
<exec_depend>your_custom_msgs</exec_depend>
<exec_depend>your_server_node</exec_depend>
<exec_depend>your_utils</exec_depend>
```

Metapackages must not include any other `package.xml` elements. But, a `CMakeLists.txt` is required, as shown below.

## CMakeLists.txt

Catkin `CMakeLists.txt` files mostly contain ordinary CMake commands, plus a few catkin-specific ones.  They begin like this:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(your_package)
```

Substitute the actual name of your package in the `project()` command.

Metapackage `CMakeLists.txt` files should contain only these two additional lines:

```cmake
find_package(catkin REQUIRED)
catkin_metapackage()
```

Regular catkin packages generally provide additional information for dependencies, building targets, installing files and running tests. They are *required* to use these two commands, usually with additional arguments:

```cmake
find_package(catkin REQUIRED COMPONENTS ...)
...
catkin_package(...)
```

[Package format 2 (recommended)](https://docs.ros.org/api/catkin/html/howto/format2/index.html#how-to-do-common-tasks-2) pages describe those tasks in detail. As you follow them, observe the usual command order:

1. `cmake_minimum_required()`
2. `project()`
3. `find_package()`
4. `add_message_files()`, `add_service_files()`, `add_action_files()`, all catkin-specific
5. `generate_messages()`, catkin-specific
6. `catkin_package()`, catkin-specific
7. `add_library()`, `add_executable()`, `target_link_libraries()`
8. `install()`
9. `catkin_add_gtest()`, `catkin_add_nosetests()`, `add_rostest()`, `add_rostest_gtest()`, all catkin-specific

# C++ catkin library dependencies/解决依赖关系（依赖于其他的ros包）

Catkin libraries are provided by ROS packages whether you install them from Ubuntu packages or build from source.

When your package depends on a catkin C++ library, there are usually several kinds of dependencies which must be declared in your `package.xml` and `CMakeLists.txt` files.

catkin C++ library 就是常见的用c++ 写的ROS包

无论是从Ubuntu软件包安装还是从源代码构建，都可以使用ROS软件包提供Catkin库。

当你的包依赖于catkin C ++库时，通常有几种依赖项必须在`package.xml`和`CMakeLists.txt`文件中声明。

## package.xml

Your package dependencies are declared in `package.xml`.  If they are missing or incorrect, you may be able to build from source and run tests in your own workspace, but your package will not work correctly when released to the ROS community.  Others rely on this information to install the software they need for using your package.

包依赖项在`package.xml`中声明。 如果它们丢失或不正确，您可以在自己的工作区中构建源代码并运行测试，但是当发布到ROS社区时，您的软件包将无法正常工作。 其他人依靠这些信息来安装他们使用您的软件包所需的软件包。

### `<depend>`

It is generally sufficient to mention each ROS package dependency once, like this:

```xml
<depend>roscpp</depend>
```

Sometimes, you may need or want more granularity for certain dependencies.  The following sections explain how to do that.  If in doubt, use the `<depend>` tag, it’s simpler.

有时，您可能需要或希望更精准的来处理某些依赖项。 以下部分说明了如何执行此操作。 如果不确定是哪种依赖，请使用`<depend>`标签，它更简单。

### `<build_depend>`

If you only use some particular dependency for building your package, and not at execution time, you can use the `<build_depend>` tag. For example, the ROS angles package only provides C++ headers and CMake configuration files:

如果你只使用一些特定的依赖项来构建你的包，而不是在执行时，你可以使用`<build_depend>`标签。 例如，ROS angles包仅提供C ++头文件和CMake配置文件：

```xml
<build_depend>angles</build_depend>
```

```shell
# angles二进制包的内容
➜ kinetic find . -name "*angles*"      
./lib/pkgconfig/angles.pc
./lib/python2.7/dist-packages/angles-1.9.11.egg-info
./lib/python2.7/dist-packages/angles
./include/angles/angles.h
./share/angles/cmake/anglesConfig.cmake
./share/angles/cmake/anglesConfig-version.cmake
```

> angles 包的 angles.h 头文件内不仅是函数原型，还有函数定义，所以不需要库，只需要包含头文件即可编译。

With this type of dependency, an installed binary of your package does not require the angles package to be installed.

**But**, that could create a problem if your package exports a header that includes the `<angles/angles.h>` header.  In that case you also need a `<build_export_depend>`.

但是，如果你的包导出（就是安装 install）的头文件包含`<angles / angles.h>`头文件，则可能会产生问题。 在这种情况下，您还需要一个`<build_export_depend>`。

>  如果别人需要用到你的包，并在c++源代码中include你的包的头文件，但是他的电脑里没有安装angles包，就会造成别人的包无法找到<angles / angles.h>头文件，也就是无法正确编译。

### `<build_export_depend>`

If you export a header that includes `<angles/angles.h>`, it will be needed by other packages that `<build_depend>` on yours:

```xml
<build_export_depend>angles</build_export_depend>
```

This mainly applies to headers and CMake configuration files.  Library packages referenced by libraries you export should normally specify `<depend>`, because they are also needed at execution time.

`<build_export_depend>`这主要适用于头文件和CMake配置文件。 如果你的包导出库，你就需要使用`<depend>`标签，因为别人在编译和执行时都需要它们。（因为是.so动态库）

### `<exec_depend>`

This tag declares dependencies for shared libraries, executables, Python modules, launch scripts and other files required when running your package.  For example, the ROS openni_launch package provides launch scripts, which are only needed at execution time:

此标记声明了运行程序包时所需的共享库，可执行文件，Python模块，启动脚本和其他文件的依赖项。 例如，ROS openni_launch包提供了启动脚本，只在执行时需要：

```xml
<exec_depend>openni_launch</exec_depend>
```

## CMakeLists.txt

CMake does not know about `package.xml` dependencies, although catkin does.  For your code to compile, the `CMakeLists.txt` must explicitly declare how to resolve all of your header and library references.

CMake并不能解析`package.xml`内描述的依赖关系，但是catkin可以（catkin拓展了CMake）。 为了编译代码，`CMakeLists.txt`必须明确声明如何解析所有的头文件和库的引用。

### Finding the library

First, CMake needs to find the library.  For catkin dependencies, this is easy:

首先，CMake需要找到库。 对于catkin依赖项，这很容易：

```cmake
find_package(catkin REQUIRED COMPONENTS roscpp)
```

This `find_package()` call defines CMake variables that will be needed later for the compile and linkedit steps.  List all additional catkin dependencies in the same command:

这个`find_package（）`调用定义了以后需要编译和链接步骤的CMake变量。 在同一命令中列出所有其他catkin依赖项：

```cmake
find_package(catkin REQUIRED COMPONENTS angles roscpp std_msgs)
# find_package(catkin REQUIRED COMPONENTS 其他的ros包)
```

Make sure all these packages are also mentioned in your `package.xml` using a `<depend>` or `<build_depend>` tag.

确保在`package.xml`使用`<depend>`或`<build_depend>`标记这些包。

### Include directories

Before compiling, collect all the header paths you found earlier:

在编译之前，包含之前找到的所有头文件的路径：

```cmake
include_directories(include ${catkin_INCLUDE_DIRS})
# 注意第一个 include 指的是你的包的头文件目录
```

```shell
# 比如：
beginner_tutorials
├── CMakeLists.txt
├── include
├── launch
├── msg
├── package.xml
├── scripts
├── src
└── srv
# 这种情况下就需要在include_directories 加入 include
```

The `include` parameter is needed only if that subdirectory of your package contains headers used to compile your programs.

仅当包的子目录包含用于编译程序的头文件时，才需要`include`参数。

### Exporting interfaces/导出接口

You must declare the library and header packages needed by all the interfaces you export to other ROS packages:

你必须声明导出到其他ROS包的所有接口所需的库和头文件的包：

```cmake
catkin_package(CATKIN_DEPENDS angles roscpp std_msgs)
```

Make sure all these packages are also mentioned in your `package.xml` using a `<depend>` or `<build_export_depend>` tag.

确保在`package.xml`中使用`<depend>`或`<build_export_depend>`标记提到所有这些包。

The `catkin_package()` command is only called once.  It may need additional parameters, depending on what else your package exports.

`catkin_package（）`命令只被调用一次。 它可能需要其他参数，具体取决于你的包导出的其他内容。

## Next steps

If your package also depends on non-catkin libraries provided by the operating system, you must provide [C++ system library dependencies](https://docs.ros.org/api/catkin/html/howto/format2/system_library_dependencies.html#system-library-dependencies-2), too.

Then, you are ready for [Building and installing C++ libraries and headers](https://docs.ros.org/api/catkin/html/howto/format2/building_libraries.html#building-libraries-2) and [Building and installing C++ executables](https://docs.ros.org/api/catkin/html/howto/format2/building_executables.html#building-executables-2).

# C++ system library dependencies/（依赖系统库）

System libraries are part of your operating system distribution.

When your package depends on a system C++ library, there are usually several kinds of dependencies which must be declared in your `package.xml` and `CMakeLists.txt` files.

系统库是操作系统的一部分。

当你的包依赖于系统C ++库时，通常有几种依赖项必须在`package.xml`和`CMakeLists.txt`文件中声明。

## package.xml

Your system package dependencies are declared in `package.xml`.  If they are missing or incorrect, you may be able to build from source and run tests on your own machine, but your package will not work correctly when released to the ROS community.  Others depend on this information to install the software they need for using your package.

### `<build_depend>`

This tag declares packages needed for building your programs, including development files like headers, libraries and configuration files.  For each build dependency, specify the corresponding [rosdep](http://wiki.ros.org/rosdep) key.  On many Linux distributions these are called “development” packages, and their names generally end in `-dev` or `-devel`, like this:

此标记声明构建程序所需的包，包括头文件，库和配置文件等开发文件。 对于每个构建依赖项，指定相应的[rosdep] 命令。 在许多Linux发行版中，这些被称为“开发”包，它们的名称通常以`-dev`或`-devel`结尾，如下所示：

```xml
<build_depend>libgstreamer0.10-dev</build_depend>
```

Some C++ packages, like Eigen, have no run-time library, and everything is defined in the header files:

一些C ++包，比如Eigen，在运行时不需要库，所有内容都在头文件中定义：

```xml
<build_depend>eigen</build_depend>
```

### `<build_export_depend>`

If your package exports a header that includes an Eigen header like `<Eigen/Geometry>`, then other packages that `<build_depend>` on yours will need Eigen, too.  To make that work correctly, declare it like this:

如果你的包导出一个包含Eigen头文件的头文件，如`<Eigen / Geometry>`，那么其它`<< build_depend>`你的包的包也需要Eigen。 要使其正常工作，请按以下方式声明：

```xml
<build_export_depend>eigen</build_export_depend>
```

This type of dependency mainly applies to headers and CMake configuration files, and it typically names the “development” package:

这种类型的依赖主要适用于头文件和CMake配置文件，它通常命名为“development”包：

```xml
<build_export_depend>libgstreamer0.10-dev</build_export_depend>
```

### `<exec_depend>`

The `<exec_depend>` is for shared libraries, executables, Python modules, launch scripts and other files required for running your package.  Specify the run-time rosdep key, if possible, like this:

`<exec_depend>`用于运行包所需的共享库，可执行文件，Python模块，启动脚本和其他文件。它指定run-time的rosdep命令，如下所示：

```xml
<exec_depend>libgstreamer0.10-0</exec_depend>
```

Many existing rosdep entries only name the library’s “development” package.  If no appropriate run-time package key is defined, consider [contributing the missing rules](http://docs.ros.org/independent/api/rosdep/html/contributing_rules.html) so users need not install unnecessary files.  If you cannot provide a run-time rosdep for some reason, you can use the “development” package for the exec dependency, too.

### `<depend>`

This tag combines all the previous types of dependencies into one.  It is not recommended for system dependencies, because it forces your package’s binary installation to depend on the “development” package, which is not generally necessary or desirable:

此标记将所有先前类型的依赖项合并为一个。 不推荐用于系统依赖，因为它强制你的包安装的二进制文件依赖于“开发”包，这通常不是必需的或不可取的：

```xml
<depend>curl</depend>
```

## CMakeLists.txt

CMake does not know about `package.xml` dependencies.  For your code to compile, the `CMakeLists.txt` must explicitly declare how to resolve all of your header and library references.

### Finding the library

First, CMake needs to find the library.  If you are lucky, someone has already provided a *CMake module* as was done for boost.  Most boost C++ components are fully implemented in the header files.  They do not require a separate shared library at run-time:

首先，CMake需要找到库。 如果你很幸运，有人已经提供了 `CMake module`，就比如Boost库就默认安装了`CMake module`。 大多数boost C ++组件都在头文件中完全实现。 它们在运行时不需要单独的共享库：

```cmake
find_package(Boost REQUIRED)
```

But, the boost thread impementation *does* require a library, so specify “COMPONENTS thread” if you need it:

但是，boost thread确实需要一个库，因此如果需要，请指定“COMPONENTS thread”：

```cmake
find_package(Boost REQUIRED COMPONENTS thread)
```

These `find_package()` calls define CMake variables that will be needed later for the compile and linkedit steps.  While the CMake community recommends standard names for those variables, some packages may not follow their recommendations.  If you run across a case like that and can’t figure out what to do, get help on [answers.ros.org](http://answers.ros.org).

Sometimes, no CMake module is available, but the library’s development package provides a [pkg-config](http://www.freedesktop.org/wiki/Software/pkg-config/) file.  To use that, first load the CMake `PkgConfig` module, then access the build flags provided by the library:

有时，没有`CMake module`可用，但库的开发包提供了[pkg-config]文件。 要使用它，首先加载CMake`PkgConfig`模块，然后访问库提供的构建标志：

```cmake
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED libgstreamer-0.10)
```

The first `pkg_check_modules()` parameter declares a prefix for CMake variables like `GSTREAMER_INCLUDE_DIRS` and `GSTREAMER_LIBRARIES`, later used for the compile and linkedit.  The `REQUIRED` argument causes configuration to fail unless the following “module” is the base name of a pkg-config file provided by the library, in this example `libgstreamer-0.10.pc`.

### Include directories

Before compiling, collect all the header paths you found earlier using `find_package()` or `pkg_check_modules()`:

```cmake
include_directories(include ${Boost_INCLUDE_DIRS} ${GSTREAMER_INCLUDE_DIRS})
```

The `include` parameter is needed only if that subdirectory of your package contains headers used to compile your programs.  If your package also depends on other catkin packages, add `${catkin_INCLUDE_DIRS}` to the list.

### Exporting interfaces

The `catkin_package()` command is only called once.  In addition to any other parameters, it must declare the non-catkin system library and header packages needed by the interfaces you export to other ROS packages:

`catkin_package（）`命令只被调用一次。 除了任何其他参数，它还必须声明导出到其他ROS包的接口所需的非catkin系统库和头文件包：

```cmake
catkin_package(DEPENDS Boost GSTREAMER)
```

Make sure all these packages are also mentioned in your `package.xml` using a `<build_export_depend>` or `<depend>` tag.

For this to work, you must have found those dependencies earlier, using `find_package()` or `pkg_check_modules()`, and they must define the CMake variables `${name}_INCLUDE_DIRS` and `${name}_LIBRARIES`.  Note that the package name is case sensitive. While catkin packages always use a lowercase name, other packages might use uppercase (as `GSTREAMER`) or mixed case (like `Boost`).

Some packages provide variable names that do not comply with these recommendations.  In that case, you must pass the absolute paths explicitly as `INCLUDE_DIRS` and `LIBRARIES`.

某些包提供的变量名称不符合这些建议。 在这种情况下，您必须将绝对路径显式地传递为 “INCLUDE_DIRS” 和 “LIBRARIES”。

## Next steps

At this point, you are ready for [Building and installing C++ libraries and headers](https://docs.ros.org/api/catkin/html/howto/format2/building_libraries.html#building-libraries-2) and [Building and installing C++ executables](https://docs.ros.org/api/catkin/html/howto/format2/building_executables.html#building-executables-2).

# C++ message or service dependencies

When your C++ programs depend on ROS messages or services, they must be defined by catkin packages like [std_msgs](http://wiki.ros.org/std_msgs) and [sensor_msgs](http://wiki.ros.org/sensor_msgs), which are used in the examples below.

当你的C ++程序依赖于ROS的消息或服务时，它们必须由catkin包定义，如[std_msgs]和[sensor_msgs]，在下面的例子中使用。

Dependencies on these packages must be declared in your `package.xml` and `CMakeLists.txt` files to resolve message references.

必须在`package.xml`和`CMakeLists.txt`文件中声明对这些包的依赖关系以解析消息引用。

## package.xml

For each C++ message dependency, `package.xml` should provide a `<depend>` tag with the ROS package name:

对于每个C ++消息依赖项，`package.xml`应该提供带有ROS包名称的`<depend>`标记：

```xml
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
```

## CMakeLists.txt

For C++ access to ROS messages, CMake needs to find the message or service headers:

对于C ++访问ROS消息，CMake需要查找消息或服务生成的头文件：

```cmake
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs)
include_directories(include ${catkin_INCLUDE_DIRS})
```

The `include` parameter is needed only if that subdirectory of your package contains headers also needed to compile your programs.

Since you presumably have build targets using the message or service headers, add this to ensure all their headers get built before any targets that need them:

由于你可能使用消息或服务头文件构建目标....

```cmake
add_dependencies(your_program ${catkin_EXPORTED_TARGETS})
add_dependencies(your_library ${catkin_EXPORTED_TARGETS})
```

# Python module dependencies

When your Python package imports other python modules, `package.xml` should provide a `<exec_depend>` with the appropriate package name.

当你的Python包导入其他python模块时，`package.xml`应该提供一个带有相应包名的`<exec_depend>`。

For system dependencies, like `python-numpy` or `python-yaml`, use the corresponding rosdep name:

对于系统依赖，如`python-numpy`或`python-yaml`，使用相应的rosdep名称：

```xml
<exec_depend>python-numpy</exec_depend>
<exec_depend>python-yaml</exec_depend>
```

These names are usually already defined in the [rosdistro repository](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml).  If you need a module not yet defined there, please [fork that repository and add them](http://docs.ros.org/independent/api/rosdep/html/contributing_rules.html).

Several ROS infrastructure modules, like `python-rospkg` or `python-rosdep` itself, apply to multiple ROS releases and are released independently of them.  Resolve those module dependencies like other system packages, using the rosdep name:

几个ROS基础架构模块，如`python-rospkg`或`python-rosdep`本身，适用于多个ROS版本，并独立发布。像其他系统包一样解决这些模块依赖关系：

```xml
<exec_depend>python-rosdep</exec_depend>
<exec_depend>python-rospkg</exec_depend>
```

When you import from another ROS Python package, like `rospy` or `roslaunch`, always use the catkin package name:

当你从另一个ROS Python包导入时，比如`rospy`或`roslaunch`，直接使用catkin包名：

```xml
<exec_depend>roslaunch</exec_depend>
<exec_depend>rospy</exec_depend>
```

ROS message or service definitions are defined as modules by ROS packages like [std_msgs](http://wiki.ros.org/std_msgs) and [sensor_msgs](http://wiki.ros.org/sensor_msgs), used as examples here. Although `<exec_depend>` is adequate when all references are in Python, the recommended method is using a `<depend>` tag with the message package name:

ROS消息或服务定义被ROS包定义为模块，如[std_msgs]和[sensor_msgs]。 虽然当所有引用都在Python中时，`<exec_depend>`就足够了，但推荐的方法是使用带有消息包名称的`<depend>`标签：

```xml
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
```

CMakeLists：

Your `CMakeLists.txt` need not specify Python-only dependencies. They are resolved automatically via `sys.path`.

`CMakeLists.txt`不需要指定仅限Python的依赖项。 它们通过`sys.path`自动解析。

# Building and installing C++ executables/C++可执行文件的构建和安装

For this example, suppose you have an executable build target named `your_node`.

## Headers

Before compiling, collect all the header paths for your build dependencies:

在编译之前，收集构建依赖项的所有头文件路径：

```cmake
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${GSTREAMER_INCLUDE_DIRS})
```

These parameters are just examples.  The `include` parameter is needed only if that subdirectory of your source package contains headers used to compile your programs.  All your catkin package header dependencies are resolved via `${catkin_INCLUDE_DIRS}`.

所有catkin包的头文件的依赖项都通过`$ {catkin_INCLUDE_DIRS}`解析。

## Building

To build `your_node`, add this command to your `CMakeLists.txt`, listing all required C++ source files, but not the headers:

```cmake
add_executable(your_node src1.cpp src2.cpp src_etc.cpp)
```

If the list of files is long, a CMake variable can help:

```cmake
set(${PROJECT_NAME}_SOURCES
    src/file1.cpp
    src/file2.cpp
    src/file3.cpp
    src/file4.cpp
    src/file5.cpp
    src/file6.cpp
    src/file_etc.cpp)
add_executable(your_node ${${PROJECT_NAME}_SOURCES})
```

If your program depends on libraries provided by other catkin packages, add this:

如果你的程序依赖于其他catkin包提供的库，请添加：

```cmake
add_executable(your_node ${${PROJECT_NAME}_SOURCES})
target_link_libraries(your_node ${catkin_LIBRARIES})
```

If your program depends on additional non-catkin system libraries, include them in the `target_link_libraries()`:

如果您的程序依赖于其他非catkin系统库，请将它们包含在`target_link_libraries（）`中：

```cmake
add_executable(your_node ${${PROJECT_NAME}_SOURCES})
target_link_libraries(your_node
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${GSTREAMER_LIBRARIES})
```

If the list of libraries is lengthy, you can similarly define a CMake variable for them.

## Installing

ROS executables are installed in a per-package directory, not the distributions’s global `bin/` directory.  There, they are accessible to [rosrun](http://wiki.ros.org/rosrun) and [roslaunch](http://wiki.ros.org/roslaunch), without cluttering up the shell’s `$PATH`, and their names only need to be unique within each package. There are only a few core ROS commands like `rosrun` and `roslaunch` that install in the global `bin/` directory.

ROS可执行文件安装在每个包目录中，而不是发行版的全局`bin /`目录中。 在那里，[rosrun]和[roslaunch]可以访问它们，而不会破坏shell的`$ PATH`， 并且他们的名字只需要在每个包中都是唯一的。 只有少数核心ROS命令，如`rosrun`和`roslaunch`，安装在全局`bin /`目录中。

List all your executables as TARGETS on an install command like this:

```cmake
install(TARGETS your_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

# Building and installing C++ libraries and headers/C++库和头文件的安装

In catkin, libraries will be installed in a common directory shared by all the packages in that entire ROS distribution.  So, make sure your library names are sufficiently unique not to clash with other packages or system libraries.  It makes sense to include at least part of your package name in each library target name.

在catkin中，库将安装在ROS的公共目录中（/opt/ros/kinetic）。 因此，请确保您的库名称足够独特，以免与其他包或系统库冲突。 在每个库目标名称中包含部分包名称是有意义的。

For this example, suppose `your_package` has a shared library build target named `your_library`.  On Linux, the actual file name will be something like `libyour_library.so`, perhaps with a version number suffix.

对于这个例子，假设`your_package`有一个名为`your_library`的共享库构建目标。 在Linux上，实际的文件名将类似于`libyour_library.so`，可能带有版本号后缀。

## Headers

Before compiling, collect all the header paths for your build dependencies:

```cmake
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${GSTREAMER_INCLUDE_DIRS})
```

That only needs to be done once in your `CMakeLists.txt`.  These parameters are just examples.  The `include` parameter is needed only if that subdirectory of your source package contains headers used to compile your programs.  All your catkin package header dependencies are resolved via `${catkin_INCLUDE_DIRS}`.

## Building

To build your library add this command to your `CMakeLists.txt`, listing all required C++ source files, but not the headers:

```cmake
add_library(your_library libsrc1.cpp libsrc2.cpp libsrc_etc.cpp)
```

If the list of source files is long, a CMake variable can help:

```cmake
set(YOUR_LIB_SOURCES
    libsrc1.cpp
    libsrc2.cpp
    libsrc3.cpp
    libsrc4.cpp
    libsrc_etc.cpp)
add_library(your_library ${YOUR_LIB_SOURCES})
```

If your library depends on libraries provided by other catkin packages, add this command:

```cmake
target_link_libraries(your_library ${catkin_LIBRARIES})
```

If your library depends on additional non-catkin system libraries, include them in the `target_link_libraries()`:

```cmake
target_link_libraries(your_library
                      ${catkin_LIBRARIES}
                      ${Boost_LIBRARIES}
                      ${GSTREAMER_LIBRARIES})
```

If the list of libraries is lengthy, you can similarly define a CMake variable for them.

## Exporting

Your `catkin_package()` needs to export all your library build targets so other catkin packages can use them.  Suppose `your_library` depends on the ROS `std_msgs` package and on the system `Boost` thread library:

你的`catkin_package（）`需要导出你所有的库构建目标，以便其他catkin包可以使用它们。 假设`your_library`依赖于ROS`std_msgs`包和系统`Boost`线程库：

```cmake
catkin_package(CATKIN_DEPENDS std_msgs
               DEPENDS Boost
               INCLUDE_DIRS include
               LIBRARIES your_library)
```

Be sure to list every library on which your libraries depend, and don’t forget to mention them in your `package.xml` using a `<depend>` or `<build_export_depend>` tag:

```xml
<depend>std_msgs</depend>
<build_export_depend>boost</build_export_depend>
```

## Installing

In catkin, libraries are installed in a `lib/` directory shared by all the packages in that entire ROS distribution.  So, be careful what you name them.

Add this command to your `CMakeLists.txt`, mentioning all your library build targets:

```cmake
install(TARGETS your_library your_other_library
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
```

The runtime destination is used for .dll file on Windows which must be placed in the global bin folder.

Libraries typically provide headers defining their interfaces.  Please follow standard ROS practice and place all external header files under `include/your_package/`:

库通常提供定义其接口的头文件。 请遵循标准的ROS习惯并将所有外部头文件放在`include / your_package /`下：

```cmake
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# include/${PROJECT_NAME}/xx.h 为catkin工作空间中的包的头文件目录
# 比如推荐 include/usb_cam/usb_cam.h 而不是 include/usb_cam.h
```

That command installs all the files in your package’s include subtree. Place only your exported headers there.  If yours is a [Subversion](http://subversion.apache.org/) repository, don’t forget to exclude the `.svn` subdirectories like this:

```cmake
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE)
```

# Building messages, services or actions/构建消息服务和动作文件

## package.xml

Your `package.xml` must declare a `<build_depend>` on `message_generation`, and a `<build_export_depend>` as well as `<exec_depend>` on `message_runtime`:

你的`package.xml`必须在`message_generation`上声明一个`<build_depend>`，在`message_runtime`上声明一个`<build_export_depend>`以及`<exec_depend>`：

```xml
<build_depend>message_generation</build_depend>
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```

Your messages services, or actions will probably include fields defined in other ROS messages, like [std_msgs](http://wiki.ros.org/std_msgs).  Declare them like this:

您的消息，服务或动作可能包括在其他ROS消息中定义的字段，如[std_msgs]。 像这样声明它们：

```xml
<depend>std_msgs</depend>
```

The `<depend>` tag is recommended for message dependencies.  That example assumes `std_msgs` is the only dependency.  Be sure to mention all your message package dependencies here, and substitute them all for `std_msgs` in the examples that follow.

建议将`<depend>`标记用于消息依赖。 该示例假设`std_msgs`是唯一的依赖项。 请务必在此处提及所有消息包依赖项，并在后面的示例中将它们全部替换为“std_msgs”。

To generate actions, add `actionlib_msgs` as a dependency:

要生成动作，请添加`actionlib_msgs`作为依赖项：

```xml
<depend>actionlib_msgs</depend>
```

## CMakeLists.txt

For CMake, find the catkin packages for `message_generation` and any messages, services or actions you depend on:

对于CMake，找到`message_generation`的catkin包以及您依赖的任何消息，服务或操作：

```cmake
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
```

For building actions, include `actionlib_msgs` among the dependencies:

如果要构建动作，在依赖项中包含`actionlib_msgs`：

```cmake
find_package(catkin REQUIRED
             COMPONENTS
             actionlib_msgs
             message_generation
             std_msgs)
```

Next, list your message definitions:

```cmake
add_message_files(DIRECTORY msg
                  FILES
                  YourFirstMessage.msg
                  YourSecondMessage.msg
                  YourThirdMessage.msg)
```

Similarly, if you have a service to generate:

```cmake
add_service_files(DIRECTORY srv
                  FILES
                  YourService.srv)
```

To generate actions, add:

```cmake
add_action_files(DIRECTORY action
                 FILES
                 YourStartAction.action
                 YourStopAction.action)
```

Then, generate all your message, service and action targets with this command:

```cmake
generate_messages(DEPENDENCIES std_msgs)
```

Make sure the `catkin_package()` command declares your message, service and action dependencies for other packages:

```cmake
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
```

A good ROS practice is to collect related messages, services and actions into a separate package with no other API.  That simplifies the package dependency graph.

一个好的ROS实践是将相关的消息，服务和操作收集到一个没有其他API的单独包中。 这简化了包依赖图。

However, you *can* provide scripts and programs with the message package.  If you do, message generation targets need to be built before any programs that depend on them.  Every target that directly or indirectly uses one of your message headers must declare an explicit dependency:

但是，您*可以*使用消息包提供脚本和程序。 如果这样做，则需要在依赖于它们的任何程序之前构建消息生成目标。 直接或间接使用您的某个消息头的每个目标都必须声明一个显式依赖：

```cmake
add_dependencies(your_program ${${PROJECT_NAME}_EXPORTED_TARGETS})
```

If your build target also uses message or service headers imported from other catkin packages, declare those dependencies similarly:

如果构建目标还使用从其他catkin包导入的消息或服务标头，请以类似方式声明这些依赖关系：

```cmake
add_dependencies(your_program ${catkin_EXPORTED_TARGETS})
```

Since catkin installs message, service and action targets automatically, no extra `install()` commands are needed for them.

由于catkin自动安装消息，服务和操作目标，因此不需要额外的`install（）`命令。

# 后面的章节暂时省略