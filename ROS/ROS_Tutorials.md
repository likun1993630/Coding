# 创建ROS工作空间

```shell
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
```
创建完成后的目录：
```
catkin_ws
└── src
    └── CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake
```

> 即使这个工作空间是空的（在'src'目录中没有任何软件包，只有一个CMakeLists.txt链接文件），你依然可以编译它：
```shell
$ cd ~/catkin_ws/
$ catkin_make
```
make编译之后的目录：
```
catkin_ws
├── build
│   ├── catkin
│   ├── catkin_generated
│   ├── CATKIN_IGNORE
│   ├── catkin_make.cache
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   ├── cmake_install.cmake
│   ├── CTestTestfile.cmake
│   ├── gtest
│   ├── Makefile
│   └── test_results
├── devel
│   ├── env.sh
│   ├── lib
│   ├── setup.bash
│   ├── setup.sh
│   ├── _setup_util.py
│   └── setup.zsh
└── src
    └── CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake

```
catkin_make命令在catkin 工作空间中是一个非常方便的工具。如果你查看一下当前目录应该能看到'build'和'devel'这两个文件夹。在'devel'文件夹里面你可以看到几个setup.*sh文件。source这些文件中的任何一个都可以将当前工作空间设置在ROS工作环境的最顶层，想了解更多请参考catkin文档。

接下来首先source一下新生成的setup.*sh文件： 

```shell
$ source devel/setup.zsh
```

要想保证工作空间已配置正确需确保ROS_PACKAGE_PATH环境变量包含你的工作空间目录，采用以下命令查看： 
```shell
$ echo $ROS_PACKAGE_PATH

/home/likun/catkin_ws/src:/opt/ros/kinetic/share
#两个路径，一个是刚刚创建的catkin_ws/src，即存放package的地方，另一个是ROS的共享package库文件夹。
#并且每个路径之间用冒号分隔开来,你可以在ROS_PACKAGE_PATH中添加更多其它路径，每条路径使用冒号':'分隔。
```

# ROS文件系统
- Packages: 软件包，是ROS应用程序代码的组织单元，每个软件包都可以包含程序库、可执行文件、脚本或者其它手动创建的东西。
- Manifest (package.xml): 清单，是对于'软件包'相关信息的描述,用于定义软件包相关元信息之间的依赖关系，这些信息包括版本、维护者和许可协议等

本教程中我们将会用到ros-tutorials程序包，请先安装：
```shell
$ sudo apt-get install ros-kinetic-ros-tutorials
```
## 文件系统工具
目的：用来简化 ls cd等的操作

### 使用rospack
rospack允许你获取软件包的有关信息。在本教程中，我们只涉及到rospack中find参数选项，该选项可以返回软件包的路径信息。 

用法：
```shell
rospack find [包名称]
```

示例：
```shell
$ rospack find roscpp

/opt/ros/kinetic/share/roscpp
```

### 使用 roscd
roscd是rosbash命令集中的一部分，它允许你直接切换(cd)工作目录到某个软件包或者软件包集当中。 

用法：
```shell
$ roscd [本地包名称[/子目录]]
```
示例：
```shell
$ roscd roscpp
```
> 为了验证我们已经切换到了roscpp软件包目录下，现在我们可以使用Unix命令pwd来输出当前工作目录：
> ```shell
> $ pwd
> 
> /opt/ros/kinetic/share/roscpp
> # 你可以看到 /opt/ros/kinetic/share/roscpp和之前使用rospack find得到的路径名称是一样的。
> ```
> 注意，就像ROS中的其它工具一样，roscd只能切换到那些路径已经包含在ROS_PACKAGE_PATH环境变量中的软件包.

#### roscd到子目录

使用roscd也可以切换到一个软件包或软件包集的子目录中。

示例：
```shell
$ roscd roscpp/cmake
$ pwd

/opt/ros/kinetic/share/roscpp/cmake
```

### 使用roscd log
使用roscd log可以切换到ROS保存日记文件的目录下。
> 需要注意的是，如果没有执行过任何ROS程序，系统会报错说该目录不存在。

示例：
```shell
$ roscd log
```
### 使用rosls
rosls是rosbash命令集中的一部分，它允许你直接按软件包的名称而不是绝对路径执行ls命令（罗列目录）。

用法：
```shell
$ rosls [本地包名称[/子目录]]
```

示例：
```shell
$ rosls roscpp_tutorials

cmake  launch  package.xml  srv
```

# 创建ROS程序包

## 一个catkin程序包由什么组成?
一个程序包要想称为catkin程序包必须符合以下要求：
- 该程序包必须包含catkin compliant package.xml文件
  - 这个package.xml文件提供有关程序包的元信息。 
- 程序包必须包含一个catkin 版本的CMakeLists.txt文件，而Catkin metapackages中必须包含一个对CMakeList.txt文件的引用。
- 每个目录下只能有一个程序包。
  - 这意味着在同一个目录下不能有嵌套的或者多个程序包存在。 
  
最简单的程序包也许看起来就像这样：
```
 my_package/
      CMakeLists.txt
      package.xml
```

## 在catkin工作空间中的程序包（catkin工作空间的目录结构）
开发catkin程序包的一个推荐方法是使用catkin工作空间，一个简单的工作空间也许看起来像这样：

```
workspace_folder/        -- WORKSPACE
  src/                   -- SOURCE SPACE
    CMakeLists.txt       -- 'Toplevel' CMake file, provided by catkin
    package_1/
      CMakeLists.txt     -- CMakeLists.txt file for package_1
      package.xml        -- Package manifest for package_1
    ...
    package_n/
      CMakeLists.txt     -- CMakeLists.txt file for package_n
      package.xml        -- Package manifest for package_n
```

## 创建一个catkin程序包
首先切换到之前通过创建catkin工作空间教程创建的catkin工作空间中的src目录下： 

```shell
$ cd ~/catkin_ws/src
```
现在使用catkin_create_pkg命令来创建一个名为'beginner_tutorials'的新程序包，这个程序包依赖于std_msgs、roscpp和rospy：
```shell
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```
> 这将会创建一个名为beginner_tutorials的文件夹，这个文件夹里面包含一个package.xml文件和一个CMakeLists.txt文件，这两个文件都已经自动包含了部分你在执行catkin_create_pkg命令时提供的信息,如依赖。
> std_msgs rospy roscpp 依赖包的信息保存在package.xml文件中

src目录将变为：

```
src
├── beginner_tutorials
│   ├── CMakeLists.txt
│   ├── include
│   │   └── beginner_tutorials
│   ├── package.xml
│   └── src
└── CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake
```
catkin_create_pkg命令会要求你输入package_name，如果有需要你还可以在后面添加一些需要依赖的其它程序包： 
```shell
$ catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
```

## 程序包依赖关系
### 一级依赖

之前在使用catkin_create_pkg命令时提供了几个程序包作为依赖包，现在我们可以使用rospack命令工具来查看一级依赖包。 
```shell
$ rospack depends1 beginner_tutorials 

std_msgs
rospy
roscpp
```
> rospack列出了在运行catkin_create_pkg命令时作为参数的依赖包，这些依赖包保存在package.xml文件中。

查看package.xml
```shell
$ roscd beginner_tutorials
$ cat package.xml

<package>
...
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
...
</package>
```

### 间接依赖
在很多情况中，一个依赖包还会有它自己的依赖包，比如，rospy还有其它依赖包。 

```shell
$ rospack depends1 rospy

genpy
roscpp
rosgraph
rosgraph_msgs
roslib
std_msgs
```

一个程序包还可以有好几个间接的依赖包，幸运的是使用rospack可以递归检测出所有的依赖包:
```shell
$ rospack depends beginner_tutorials

cpp_common
rostime
roscpp_traits
roscpp_serialization
genmsg
genpy
message_runtime
rosconsole
std_msgs
rosgraph_msgs
xmlrpcpp
roscpp
rosgraph
catkin
rospack
roslib
rospy
```

## 自定义程序包
接下来将剖析catkin_create_pkg命令生成的每个文件并详细描述这些文件的组成部分以及如何自定义这些文件。 
> 文件中这样的符号内的内容为注释 ` <!--  --> `

### 自定义 package.xml
自动生成的package.xml文件应该在你的新程序包中。现在让我们一起来看看新生成的package.xml文件以及每一个需要你注意的标签元素。 

- 描述标签
```
<description>The beginner_tutorials package</description>
```
> 将描述信息修改为任何你喜欢的内容，但是按照约定第一句话应该简短一些，因为它覆盖了程序包的范围。如果用一句话难以描述完全那就需要换行了。

- 维护者标签

```
  <!-- One maintainer tag required, multiple allowed, one person per tag --> 
  <!-- Example:  -->
  <!-- <maintainer email="jane.doe@example.com">Jane Doe</maintainer> -->
  <maintainer email="user@todo.todo">user</maintainer>
```
- 许可标签：
```
  <!-- One license tag required, multiple allowed, one license per tag -->
  <!-- Commonly used license strings: -->
  <!--   BSD, MIT, Boost Software License, GPLv2, GPLv3, LGPLv2.1, LGPLv3 -->
  <license>TODO</license>
```
> 选择一种许可协议并填写到这里。一些常见的开源许可协议有BSD、 MIT、 Boost Software License、 GPLv2、 GPLv3、 LGPLv2.1和LGPLv3。
> 比如`<license>BSD</license>`

- 依赖项标签：
用来描述程序包的各种依赖项，这些依赖项分为build_depend、 buildtool_depend、 run_depend、 test_depend。在之前的操作中，因为我们将 std_msgs、 roscpp、 和 rospy作为catkin_create_pkg命令的参数，所以生成的依赖项看起来如下： 

```
<!-- The *_depend tags are used to specify dependencies -->
<!-- Dependencies can be catkin packages or system dependencies -->
<!-- Examples: -->
<!-- Use build_depend for packages you need at compile time: -->
<!--   <build_depend>genmsg</build_depend> -->
<!-- Use buildtool_depend for build tool packages: -->
<!--   <buildtool_depend>catkin</buildtool_depend> -->
<!-- Use exec_depend for packages you need at runtime: -->
<!--   <exec_depend>python-yaml</exec_depend> -->
<!-- Use test_depend for packages you need only for testing: -->
<!--   <test_depend>gtest</test_depend> -->
<buildtool_depend>catkin</buildtool_depend>
<build_depend>roscpp</build_depend>
<build_depend>rospy</build_depend>
<build_depend>std_msgs</build_depend>
```
除了catkin中默认提供的buildtool_depend，所有我们列出的依赖包都已经被添加到build_depend标签中。在本例中，因为在编译和运行时我们需要用到所有指定的依赖包，因此还需要将每一个依赖包分别添加到run_depend标签中：

```
<buildtool_depend>catkin</buildtool_depend>

<build_depend>roscpp</build_depend>
<build_depend>rospy</build_depend>
<build_depend>std_msgs</build_depend>

<exec_depend>roscpp</exec_depend>
<exec_depend>rospy</exec_depend>
<exec_depend>std_msgs</exec_depend>
```
-  最后完成的 package.xml
   现在看下面最后去掉了注释和未使用标签后的package.xml文件就显得更加简洁了： 
```
<?xml version="1.0"?>
<package format="2">
  <name>beginner_tutorials</name>
  <version>0.1.0</version>
  <description>The beginner_tutorials package</description>

  <maintainer email="you@yourdomain.tld">Your Name</maintainer>
  <license>BSD</license>
  <url type="website">http://wiki.ros.org/beginner_tutorials</url>
  <author email="you@yourdomain.tld">Jane Doe</author>

  <buildtool_depend>catkin</buildtool_depend>

  <build_depend>roscpp</build_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>

  <exec_depend>roscpp</exec_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>

</package>
```

# 编译ROS程序包
## 使用 catkin_make

catkin_make 是一个命令行工具，它简化了catkin的标准工作流程。你可以认为catkin_make是在CMake标准工作流程中依次调用了cmake 和 make。 

使用方法:
```shell
# 在catkin工作空间下
$ catkin_make [make_targets] [-DCMAKE_VARIABLES=...]
```

CMake标准工作流程主要可以分为以下几个步骤：
```shell
# 在一个CMake项目里
$ mkdir build
$ cd build
$ cmake ..
$ make
$ make install  # (可选)
```

多个catkin项目可以放在工作空间中一起编译：
```shell
# In a catkin workspace
$ catkin_make
$ catkin_make install  # (可选)
```

## 开始编译你的程序包
按照之前的创建一个ROS程序包教程，你应该已经创建好了一个catkin 工作空间 和一个名为beginner_tutorials的catkin 程序包。

```shell
$ cd ~/catkin_ws/
$ catkin_make


```
你可以看到很多cmake 和 make 输出的信息： 

```
Base path: /home/user/catkin_ws
Source space: /home/user/catkin_ws/src
Build space: /home/user/catkin_ws/build
Devel space: /home/user/catkin_ws/devel
Install space: /home/user/catkin_ws/install
####
#### Running command: "cmake /home/user/catkin_ws/src
-DCATKIN_DEVEL_PREFIX=/home/user/catkin_ws/devel
-DCMAKE_INSTALL_PREFIX=/home/user/catkin_ws/install" in "/home/user/catkin_ws/build"
####
-- The C compiler identification is GNU 4.2.1
-- The CXX compiler identification is Clang 4.0.0
-- Checking whether C compiler has -isysroot
-- Checking whether C compiler has -isysroot - yes
-- Checking whether C compiler supports OSX deployment target flag
-- Checking whether C compiler supports OSX deployment target flag - yes
-- Check for working C compiler: /usr/bin/gcc
-- Check for working C compiler: /usr/bin/gcc -- works
-- Detecting C compiler ABI info
-- Detecting C compiler ABI info - done
-- Check for working CXX compiler: /usr/bin/c++
-- Check for working CXX compiler: /usr/bin/c++ -- works
-- Detecting CXX compiler ABI info
-- Detecting CXX compiler ABI info - done
-- Using CATKIN_DEVEL_PREFIX: /tmp/catkin_ws/devel
-- Using CMAKE_PREFIX_PATH: /opt/ros/groovy
-- This workspace overlays: /opt/ros/groovy
-- Found PythonInterp: /usr/bin/python (found version "2.7.1") 
-- Found PY_em: /usr/lib/python2.7/dist-packages/em.pyc
-- Found gtest: gtests will be built
-- catkin 0.5.51
-- BUILD_SHARED_LIBS is on
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- ~~  traversing packages in topological order:
-- ~~  - beginner_tutorials
-- ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
-- +++ add_subdirectory(beginner_tutorials)
-- Configuring done
-- Generating done
-- Build files have been written to: /home/user/catkin_ws/build
####
#### Running command: "make -j4" in "/home/user/catkin_ws/build"
####
```
> catkin_make首先输出它所使用到的每个空间所在的路径。需要注意的是由于这些空间存在默认配置的原因，有几个文件夹已经在catkin工作空间自动生成了
> build 目录是build space的默认所在位置，同时cmake 和 make也是在这里被调用来配置并编译你的程序包。
> devel 目录是devel space的默认所在位置, 同时也是在你安装程序包之前存放可执行文件和库文件的地方。 

# 理解 ROS节点
## 图概念概述
- Nodes:节点,一个节点即为一个**可执行文件**，它可以通过ROS与其它节点进行通信。
- Messages:消息，消息是一种ROS数据类型，用于订阅或发布到一个话题。
- Topics:话题,节点可以发布消息到话题，也可以订阅话题以接收消息。
- Master:节点管理器，ROS名称服务 (比如帮助节点找到彼此)。
- rosout: ROS中相当于stdout/stderr。
- roscore: 主机+ rosout + 参数服务器 (参数服务器会在后面介绍)。 

## 节点：
一个节点其实只不过是ROS程序包中的一个可执行文件。ROS节点可以使用ROS客户库与其他节点通信。节点可以发布或接收一个话题。节点也可以提供或使用某种服务。

## 客户端库
ROS客户端库允许使用不同编程语言编写的节点之间互相通信:
- rospy = python 客户端库
- roscpp = c++ 客户端库 

## roscore
roscore 是你在运行所有ROS程序前首先要运行的命令。

##  使用rosnode
rosnode 显示当前运行的ROS节点信息。rosnode list 指令列出活跃的节点: 
```shell
# 提前运行roscore
$ rosnode list

/rosout
```
> 这表示当前只有一个节点在运行: rosout。因为这个节点用于收集和记录节点调试输出信息，所以它总是在运行的。 

rosnode info 命令返回的是关于一个特定节点的信息。
```shell
$ rosnode info /rosout


Node [/rosout]
Publications: 
 * /rosout_agg [rosgraph_msgs/Log]
Subscriptions: 
 * /rosout [unknown type]
Services: 
 * /rosout/get_loggers
 * /rosout/set_logger_level
contacting node http://Ubuntu16:35409/ ...
Pid: 11287
```

## 使用 rosrun
rosrun 允许你使用包名直接运行一个包内的节点(而不需要知道这个包的路径)。
用法:
```shell
$ rosrun [package_name] [node_name]
```
现在我们可以运行turtlesim包中的 turtlesim_node。
然后, 在一个 新的终端:

```shell
$ rosrun turtlesim turtlesim_node
```
在一个 新的终端:
```shell
$ rosnode list

/rosout
/turtlesim
```
