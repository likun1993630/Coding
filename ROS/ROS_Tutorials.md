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
> 件中这样的符号内的内容为注释  <!--  --> 



