# ROS文件系统
主要介绍了ROS的工程结构，也就是ROS的文件系统结构。
- catkin的编译系统
- catkin工作空间的创建和结构
- package软件包的创建和结构
- MakeLists.txt文件，package.xml以及其他常见文件。

## Catkin编译系统
对于源代码包，我们只有编译才能在系统上运行。而Linux下的编译器有gcc、g++，随着源文件的增加，直接用gcc/g++命令的方式显得效率低下，人们开始用Makefile来进行编译。然而随着工程体量的增大，Makefile也不能满足需求，于是便出现了Cmake工具。CMake是对make工具的生成器，是更高层的工具，它简化了编译构建过程，能够管理大型项目，具有良好的扩展性。对于ROS这样大体量的平台来说，就采用的是CMake，并且ROS对CMake进行了扩展，于是便有了Catkin编译系统。

![](./res/catkin.jpg) 

目前的ROS同时支持着rosbuild和Catkin两种编译系统，但ROS的核心软件包也已经全部转换为Catkin。rosbuild已经被逐步淘汰，所以建议初学者直接上手Catkin。

### Catkin特点

Catkin是基于CMake的编译构建系统，具有以下特点：

- Catkin沿用了包管理的传统像 find_package()基础结构,pkg-config
- 扩展了CMake，例如
	- 软件包编译后无需安装就可使用
	- 自动生成find_package()代码，pkg-config文件
	- 解决了多个软件包构建顺序问题

一个Catkin的软件包（package）必须要包括两个文件：

- package.xml: 包括了package的描述信息
	- name, description, version, maintainer(s), license
	- opt. authors, url's, dependencies, plugins, etc...

- CMakeLists.txt: 构建package所需的CMake文件:

	- 调用Catkin的函数/宏
	- 解析package.xml
	- 找到其他依赖的catkin软件包
	- 将本软件包添加到环境变量

### Catkin工作原理
catkin编译的工作流程如下:

1. 首先在工作空间catkin_ws/src/下递归的查找其中每一个ROS的package。
2. package中会有package.xml和CMakeLists.txt文件，Catkin(CMake)编译系统依据CMakeLists.txt文件,从而生成makefiles(放在catkin_ws/build/)。
3. 然后make刚刚生成的makefiles等文件，编译链接生成可执行文件(放在catkin_ws/devel)。

也就是说，Catkin就是将cmake与make指令做了一个封装从而完成整个编译过程的工具。catkin有比较突出的优点，主要是：

- 操作更加简单
- 一次配置，多次使用
- 跨依赖项目编译

### 使用catkin_make进行编译
要用catkin编译一个工程或软件包，只需要用catkin_make指令。一般当我们写完代码，执行一次catkin_make进行编译,调用系统自动完成编译和链接过程，构建生成目标文件。编译的一般性流程如下，在1.5节我们编译ROS-Academy-for-Beginners教学包就是这样的流程。

```shell
$ cd ~/catkin_ws #回到工作空间,catkin_make必须在工作空间下执行
$ catkin_make    #开始编译
$ source ~/catkin_ws/devel/setup.bash #刷新坏境
```
> 注意: catkin编译之前需要回到工作空间目录(比如/catkin_ws)，catkin_make在其他路径下编译不会成功。编译完成后，如果有新的目标文件产生（原来没有），那么一般紧跟着要source刷新环境，使得系统能够找到刚才编译生成的ROS可执行文件。这个细节比较容易遗漏，致使后面出现可执行文件无法打开等错误。

catkin_make命令也有一些可选参数，例如：

```shell
catkin_make [args]
  -h, --help            帮助信息
  -C DIRECTORY, --directory DIRECTORY
                        工作空间的路径 (默认为 '.')
  --source SOURCE       src的路径 (默认为'workspace_base/src')
  --build BUILD         build的路径 (默认为'workspace_base/build')
  --use-ninja           用ninja取代make
  --use-nmake           用nmake取'make
  --force-cmake         强制cmake，即使已经cmake过
  --no-color            禁止彩色输出(只对catkin_make和CMake生效)
  --pkg PKG [PKG ...]   只对某个PKG进行make
  --only-pkg-with-deps  ONLY_PKG_WITH_DEPS [ONLY_PKG_WITH_DEPS ...]
                        将指定的package列入白名单CATKIN_WHITELIST_PACKAGES，
                        之编译白名单里的package。该环境变量存在于CMakeCache.txt。
  --cmake-args [CMAKE_ARGS [CMAKE_ARGS ...]]
                        传给CMake的参数
  --make-args [MAKE_ARGS [MAKE_ARGS ...]]
                        传给Make的参数
  --override-build-tool-check
                        用来覆盖由于不同编译工具产生的错误
```
## Catkin工作空间
Catkin工作空间是创建、修改、编译catkin软件包的目录。catkin的工作空间，直观的形容就是一个仓库，里面装载着ROS的各种项目工程，便于系统组织管理调用。在可视化图形界面里是一个文件夹。我们自己写的ROS代码通常就放在工作空间中，本节就来介绍catkin工作空间的结构。

### 初始化catkin工作空间

介绍完catkin编译系统，我们来建立一个catkin的工作空间。首先我们要在计算机上创建一个初始的catkin_ws/路径，这也是catkin工作空间结构的最高层级。输入下列指令，完成初始创建。

```shell
$ mkdir -p ~/catkin_ws/src　　
$ cd ~/catkin_ws/
$ catkin_make #初始化工作空间
```
> 注意：1. catkin_make命令必须在工作空间这个路径上执行 2.原先的初始化命令catkin_init_workspace仍然保留

### 结构介绍
catkin的结构十分清晰，具体的catkin工作空间结构图如下。初看起来catkin工作空间看起来极其复杂，其实不然，catkin工作空间的结构其实非常清晰。

```
─ build
│   ├── catkin
│   │   └── catkin_generated
│   │       └── version
│   │           └── package.cmake
│   ├──
......

│   ├── catkin_make.cache
│   ├── CMakeCache.txt
│   ├── CMakeFiles
│   │   ├──
......

├── devel
│   ├── env.sh
│   ├── lib
│   ├── setup.bash
│   ├── setup.sh
│   ├── _setup_util.py
│   └── setup.zsh
└── src
└── CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake
```

catkin工作空间的结构,它包括了src、build、devel三个路径，在有些编译选项下也可能包括其他。但这三个文件夹是catkin编译系统默认的。它们的具体作用如下：

- src/: ROS的catkin软件包（源代码包）
- build/: catkin（CMake）的缓存信息和中间文件
- devel/: 生成的目标文件（包括头文件，动态链接库，静态链接库，可执行文件等）、环境变量

在编译过程中，它们的工作流程如图：

![](./res/catkin_flow.jpg) 

后两个路径由catkin系统自动生成、管理，我们日常的开发一般不会去涉及，而主要用到的是src文件夹，我们写的ROS程序、网上下载的ROS源代码包都存放在这里。

在编译时，catkin编译系统会递归的查找和编译src/下的每一个源代码包。因此你也可以把几个源代码包放到同一个文件夹下，如下图所示：

![](./res/catkin_ws.jpg) 

catkin工作空间基本就是以上的结构，package是catkin工作空间的基本单元，我们在ROS开发时，写好代码，然后catkin_make，系统就会完成所有编译构建的工作。

## Package软件包
package是ROS源代码存放的地方，任何ROS的代码无论是C++还是Python都要放到package中，这样才能正常的编译和运行。
一个package可以编译出来多个目标文件（ROS可执行程序、动态静态库、头文件等等）。

### package结构

一个package下常见的文件、路径有：

```
  ├── CMakeLists.txt    #package的编译规则(必须)
  ├── package.xml       #package的描述信息(必须)
  ├── src/              #源代码文件
  ├── include/          #C++头文件
  ├── scripts/          #可执行脚本，如python
  ├── msg/              #自定义消息
  ├── srv/              #自定义服务
  ├── models/           #3D模型文件
  ├── urdf/             #urdf文件
  ├── launch/           #launch文件
```
** 其中定义package的是CMakeLists.txt和package.xml，这两个文件是package中必不可少的。catkin编译系统在编译前，首先就要解析这两个文件。这两个文件就定义了一个package **。

- CMakeLists.txt: 定义package的包名、依赖、源文件、目标文件等编译规则，是package不可少的成分
- package.xml: 描述package的包名、版本号、作者、依赖等信息，是package不可少的成分
- src/: 存放ROS的源代码，包括C++的源码和(.cpp)以及Python的module(.py)
- include/: 存放C++源码对应的头文件
- scripts/: 存放可执行脚本，例如shell脚本(.sh)、Python脚本(.py)
- msg/: 存放自定义格式的消息(.msg)
- srv/: 存放自定义格式的服务(.srv)
- models/: 存放机器人或仿真场景的3D模型(.sda, .stl, .dae等)
- urdf/: 存放机器人的模型描述(.urdf或.xacro)
- launch/: 存放launch文件(.launch或.xml)

### package的创建

创建一个package需要在catkin_ws/src下(catkin_ws 可以自己根据规则随意命名),用到catkin_create_pkg命令，用法是：
```shell
catkin_create_pkg package depends
```
> 其中package是包名，depends是依赖的包名，可以依赖多个软件包。

例如，新建一个package叫做test_pkg,依赖roscpp、rospy、std_msgs(常用依赖)。
```shell
$ catkin_create_pkg test_pkg roscpp rospy std_msgs
```

这样就会在当前路径下新建test_pkg软件包，包括：
```
  ├── CMakeLists.txt
  ├── include
  │   └── test_pkg
  ├── package.xml
  └── src
```
> catkin_create_pkg自动完成了软件包的初始化，填充好了CMakeLists.txt和package.xml，并且将依赖项（创建时填入的依赖项）填进了这两个文件中。

### package相关命令

- roscd

- rosls

- rospack

rospack是对package管理的工具，命令的用法如下：

| rostopic命令              | 作用                      |
|---------------------------|---------------------------|
| rospack help              | 显示rospack的用法         |
| rospack list              | 列出本机所有package       |
| rospack depends [package] | 显示package的依赖包(pacakge.xml文件指定 )      |
| rospack find [package]    | 定位某个package           |
| rospack profile           | 刷新所有package的位置记录 |

> 以上命令如果package缺省，则默认为当前目录(如果当前目录包含package.xml)

- rosdep

rosdep是用于管理ROS package依赖项的命令行工具，用法如下：

| rosdep命令               | 作用                        |
|--------------------------|-----------------------------|
| rosdep check [pacakge]   | 检查package的依赖是否满足   |
| rosdep install [pacakge] | 安装pacakge的依赖           |
| rosdep db                | 生成和显示依赖数据库        |
| rosdep init              | 初始化/etc/ros/rosdep中的源 |
| rosdep keys              | 检查package的依赖是否满足   |
| rosdep update            | 更新本地的rosdep数据库      |

> 一个较常使用的命令是`rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y`,用于安装工作空间中src路径下所有package的依赖项（由pacakge.xml文件指定）。

## CMakeLists.txt

CMakeLists.txt原本是Cmake编译系统的规则文件，而Catkin编译系统基本沿用了CMake的编译风格，只是针对ROS工程添加了一些宏定义。所以在写法上，catkin的CMakeLists.txt与CMake的基本一致。

这个文件直接规定了这个package要依赖哪些package，要编译生成哪些目标，如何编译等等流程。所以CMakeLists.txt非常重要，它指定了由源码到目标文件的规则，catkin编译系统在工作时首先会找到每个package下的CMakeLists.txt，然后按照规则来编译构建。

### CMakeLists.txt写法

CMakeLists.txt的基本语法都还是按照CMake，而Catkin在其中加入了少量的宏，总体的结构如下：

```
cmake_minimum_required() #CMake的版本号 
project()                #项目名称 
find_package()           #找到编译需要的其他CMake/Catkin package
catkin_python_setup()    #catkin新加宏，打开catkin的Python Module的支持
add_message_files()      #catkin新加宏，添加自定义Message/Service/Action文件
add_service_files()
add_action_files()
generate_message()       #catkin新加宏，生成不同语言版本的msg/srv/action接口
catkin_package()         #catkin新加宏，生成当前package的cmake配置，供依赖本包的其他软件包调用
add_library()            #生成库
add_executable()         #生成可执行二进制文件
add_dependencies()       #定义目标文件依赖于其他目标文件，确保其他目标已被构建
target_link_libraries()  #链接
catkin_add_gtest()       #catkin新加宏，生成测试
install()                #安装至本机
```
> 关于cmake的用法查看：
https://github.com/Akagi201/learning-cmake/blob/master/docs/cmake-practice.pdf

### CMakeLists例子
以turtlesim小海龟这个pacakge为例，可以roscd到tuetlesim包下查看。

```
cmake_minimum_required(VERSION 2.8.3)
#CMake至少为2.8.3版

project(turtlesim)
#项目(package)名称为turtlesim，在后续文件中可使用变量${PROJECT_NAME}来引用项目名称turltesim

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rosconsole roscpp roscpp_serialization roslib rostime std_msgs std_srvs)
#cmake宏，指定依赖的其他pacakge，实际是生成了一些环境变量，如<NAME>_FOUND, <NAME>_INCLUDE_DIRS, <NAME>_LIBRARYIS
#此处catkin是必备依赖 其余的geometry_msgs...为组件

find_package(Qt5Widgets REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
#指定C++的头文件路径
link_directories(${catkin_LIBRARY_DIRS})
#指定链接库的路径

add_message_files(DIRECTORY msg FILES
Color.msg Pose.msg)
#自定义msg文件

add_service_files(DIRECTORY srv FILES
Kill.srv
SetPen.srv
Spawn.srv
TeleportAbsolute.srv
TeleportRelative.srv)
#自定义srv文件

generate_messages(DEPENDENCIES geometry_msgs std_msgs std_srvs)
#在add_message_files、add_service_files宏之后必须加上这句话，用于生成srv msg头文件/module，生成的文件位于devel/include中

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs std_srvs)
# catkin宏命令，用于配置ROS的package配置文件和CMake文件
# 这个命令必须在add_library()或者add_executable()之前调用，该函数有5个可选参数：
# (1) INCLUDE_DIRS - 导出包的include路径
# (2) LIBRARIES - 导出项目中的库
# (3) CATKIN_DEPENDS - 该项目依赖的其他catkin项目
# (4) DEPENDS - 该项目所依赖的非catkin CMake项目。
# (5) CFG_EXTRAS - 其他配置选项

set(turtlesim_node_SRCS
src/turtlesim.cpp
src/turtle.cpp
src/turtle_frame.cpp
)
set(turtlesim_node_HDRS
include/turtlesim/turtle_frame.h
)
#指定turtlesim_node_SRCS、turtlesim_node_HDRS变量

qt5_wrap_cpp(turtlesim_node_MOCS ${turtlesim_node_HDRS})

add_executable(turtlesim_node ${turtlesim_node_SRCS} ${turtlesim_node_MOCS})
# 指定可执行文件目标turtlesim_node
target_link_libraries(turtlesim_node Qt5::Widgets ${catkin_LIBRARIES} ${Boost_LIBRARIES})
# 指定链接可执行文件
add_dependencies(turtlesim_node turtlesim_gencpp)

add_executable(turtle_teleop_key tutorials/teleop_turtle_key.cpp)
target_link_libraries(turtle_teleop_key ${catkin_LIBRARIES})
add_dependencies(turtle_teleop_key turtlesim_gencpp)

add_executable(draw_square tutorials/draw_square.cpp)
target_link_libraries(draw_square ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(draw_square turtlesim_gencpp)

add_executable(mimic tutorials/mimic.cpp)
target_link_libraries(mimic ${catkin_LIBRARIES})
add_dependencies(mimic turtlesim_gencpp)
# 同样指定可执行目标、链接、依赖

install(TARGETS turtlesim_node turtle_teleop_key draw_square mimic
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# 安装目标文件到本地系统

install(DIRECTORY images
DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
FILES_MATCHING PATTERN "*.png" PATTERN "*.svg")
```

## package.xml
package.xml也是一个catkin的package必备文件，它是这个软件包的描述文件。
> 在较早的ROS版本(rosbuild编译系统)中，这个文件叫做manifest.xml，用于描述pacakge的基本信息。如果你在网上看到一些ROS项目里包含着manifest.xml，那么它多半是hydro版本之前的项目了。

### package.xml作用

pacakge.xml包含了package的名称、版本号、内容描述、维护人员、软件许可、编译构建工具、编译依赖、运行依赖等信息。
实际上rospack find、rosdep等命令之所以能快速定位和分析出package的依赖项信息，就是直接读取了每一个pacakge中的package.xml文件。它为用户提供了快速了解一个pacakge的渠道。

### package.xml写法
pacakge.xml遵循xml标签文本的写法，由于版本更迭原因，现在有两种格式并存（format1与format2），不过区别不大。老版本（format1）的pacakge.xml通常包含以下标签:

```
<pacakge>           根标记文件  
<name>              包名  
<version>           版本号  
<description>       内容描述  
<maintainer>        维护者 
<license>           软件许可证  
<buildtool_depend>  编译构建工具，通常为catkin  
<build_depend>      编译依赖项，与Catkin中的  
<run_depend>        运行依赖项
```

> 说明：其中1-6为必备标签，1是根标签，嵌套了其余的所有标签，2-6为包的各种属性，7-9为编译相关信息。

在新版本（format2）中，包含的标签为：

```
<pacakge>               根标记文件  
<name>                  包名  
<version>               版本号  
<description>           内容描述  
<maintainer>            维护者 
<license>               软件许可证  
<buildtool_depend>      编译构建工具，通常为catkin    
<depend>                指定依赖项为编译、导出、运行需要的依赖，最常用
<build_depend>          编译依赖项  
<build_export_depend>   导出依赖项
<exec_depend>           运行依赖项
<test_depend>           测试用例依赖项  
<doc_depend>            文档依赖项
```

> 目前Indigo、Kinetic、Lunar等版本的ROS都同时支持两种版本的package.xml，所以无论选哪种格式都可以。

### pacakge.xml例子
以turtlesim软件包为例，其pacakge.xml文件内容如下:

老版本（format1）的写法:

```
<?xml version="1.0"?>       <!--本示例为老版本的pacakge.xml-->
<package>                   <!--pacakge为根标签，写在最外面-->
  <name>turtlesim</name>
  <version>0.8.1</version>
  <description>
    turtlesim is a tool made for teaching ROS and ROS packages.
  </description>
  <maintainer email="dthomas@osrfoundation.org">Dirk Thomas</maintainer>
  <license>BSD</license>

  <url type="website">http://www.ros.org/wiki/turtlesim</url>
  <url type="bugtracker">https://github.com/ros/ros_tutorials/issues</url>
  <url type="repository">https://github.com/ros/ros_tutorials</url>
  <author>Josh Faust</author>

  <!--编译工具为catkin-->
  <buildtool_depend>catkin</buildtool_depend>

  <!--编译时需要依赖以下包-->  
  <build_depend>geometry_msgs</build_depend>    
  <build_depend>qtbase5-dev</build_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>qt5-qmake</build_depend>
  <build_depend>rosconsole</build_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>roscpp_serialization</build_depend>
  <build_depend>roslib</build_depend>
  <build_depend>rostime</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>std_srvs</build_depend>

  <!--运行时需要依赖以下包-->
  <run_depend>geometry_msgs</run_depend>
  <run_depend>libqt5-core</run_depend>
  <run_depend>libqt5-gui</run_depend>
  <run_depend>message_runtime</run_depend>
  <run_depend>rosconsole</run_depend>
  <run_depend>roscpp</run_depend>
  <run_depend>roscpp_serialization</run_depend>
  <run_depend>roslib</run_depend>
  <run_depend>rostime</run_depend>
  <run_depend>std_msgs</run_depend>
  <run_depend>std_srvs</run_depend>
</package>
```

新版本（format2）:
```
<?xml version="1.0"?>
<package format="2">      <!--在声明pacakge时指定format2，为新版格式-->
  <name>turtlesim</name>
  <version>0.8.1</version>
  <description>
    turtlesim is a tool made for teaching ROS and ROS packages.
  </description>
  <maintainer email="dthomas@osrfoundation.org">Dirk Thomas</maintainer>
  <license>BSD</license>

  <url type="website">http://www.ros.org/wiki/turtlesim</url>
  <url type="bugtracker">https://github.com/ros/ros_tutorials/issues</url>
  <url type="repository">https://github.com/ros/ros_tutorials</url>
  <author>Josh Faust</author>

  <!--编译工具为catkin-->
  <buildtool_depend>catkin</buildtool_depend>

  <!--用depend来整合build_depend和run_depend-->  
  <depend>geometry_msgs</depend>
  <depend>rosconsole</depend>
  <depend>roscpp</depend>
  <depend>roscpp_serialization</depend>
  <depend>roslib</depend>
  <depend>rostime</depend>
  <depend>std_msgs</depend>
  <depend>std_srvs</depend>

  <!--build_depend标签未变-->
  <build_depend>qtbase5-dev</build_depend>
  <build_depend>message_generation</build_depend>
  <build_depend>qt5-qmake</build_depend>

  <!--run_depend要改为exec_depend-->
  <exec_depend>libqt5-core</exec_depend>
  <exec_depend>libqt5-gui</exec_depend>
  <exec_depend>message_runtime</exec_depend>
</package>
```

## Metapackage
Metapackage指的是将多个功能接近、甚至相互依赖的软件包的放到一个集合中去。
>在Hydro之前叫做Stack

ROS里常见的Metapacakge有：

| Metapacakge名称 | 描述                                   | 链接                                            |
|-----------------|----------------------------------------|-------------------------------------------------|
| navigation      | 导航相关的功能包集                     | https://github.com/ros-planning/navigation      |
| moveit          | 运动规划相关的（主要是机械臂）功能包集 | https://github.com/ros-planning/moveit          |
| image_pipeline  | 图像获取、处理相关的功能包集           | https://github.com/ros-perception/image_common  |
| vision_opencv   | ROS与OpenCV交互的功能包集              | https://github.com/ros-perception/vision_opencv |
| turtlebot       | Turtlebot机器人相关的功能包集          | https://github.com/turtlebot/turtlebot          |
| pr2_robot       | pr2机器人驱动功能包集                  | https://github.com/PR2/pr2_robot                |
| ...             | ...                                    | ...                                             |

以上列举了一些常见的功能包集，例如navigation、turtlebot，他们都是用于某一方面的功能，以navigation metapackage（官方介绍里仍然沿用stack的叫法）为例，它包括了以下软件包：

| 包名               | 功能                             |
|--------------------|----------------------------------|
| navigation         | Metapacakge，依赖以下所有pacakge |
| amcl               | 定位                             |
| fake_localization  | 定位                             |
| map_server         | 提供地图                         |
| move_base          | 路径规划节点                     |
| nav_core           | 路径规划的接口类                 |
| base_local_planner | 局部规划                         |
| dwa_local_planner  | 局部规划                         |
| ...                | ...                              |

> navigation就是一个简单的pacakge，里面只有几个文件，但由于它依赖了其他所有的软件包。Catkin编译系统会明白，这些软件包都属于navigation metapacakge。

### Metapackage写法
我们以ROS-Academy-for-beginners为例介绍meteapckage的写法，在教学包内，有一个ros-academy-for-beginners软件包，该包即为一个metapacakge，其中有且仅有两个文件：CMakeLists.txt和pacakge.xml。

CMakeLists.txt写法如下：

```
cmake_minimum_required(VERSION 2.8.3)
project(ros_academy_for_beginners)
find_package(catkin REQUIRED)
catkin_metapackage()   #声明本软件包是一个metapacakge
```

pacakge.xml写法如下：
```
<package>
    <name>ros_academy_for_beginners</name>
    <version>17.12.4</version>
    <description>
        --------------------------------------------------------------------------
        A ROS tutorial for beginner level learners. This metapacakge includes some
        demos of topic, service, parameter server, tf, urdf, navigation, SLAM...
        It tries to explain the basic concepts and usages of ROS.
        --------------------------------------------------------------------------
    </description>
    <maintainer email="chaichangkun@163.com">Chai Changkun</maintainer>
    <author>Chai Changkun</author>
    <license>BSD</license>  
    <url>http://http://www.droid.ac.cn</url>

    <buildtool_depend>catkin</buildtool_depend>

    <run_depend>navigation_sim_demo</run_depend>  <!--注意这里的run_depend标签，将其他软件包都设为依赖项-->
    <run_depend>param_demo</run_depend>
    <run_depend>robot_sim_demo</run_depend>
    <run_depend>service_demo</run_depend>
    <run_depend>slam_sim_demo</run_depend>
    <run_depend>tf_demo</run_depend>
    <run_depend>topic_demo</run_depend>

    <export>    <!--这里需要有export和metapacakge标签，注意这种固定写法-->
        <metapackage/>
    </export>
</package>
```

metapacakge中的以上两个文件和普通pacakge不同点是：

- CMakeLists.txt:加入了catkin_metapackage()宏，指定本软件包为一个metapacakge。
- package.xml:标签将所有软件包列为依赖项，标签中添加标签声明。

> metapacakge在我们实际开发一个大工程时可能有用

## 其他常见文件类型
- launch文件

launch文件一般以.launch或.xml结尾，它对ROS需要运行程序进行了打包，通过一句命令来启动。一般launch文件中会指定要启动哪些package下的哪些可执行程序，指定以什么参数启动，以及一些管理控制的命令。 launch文件通常放在软件包的launch/路径中中。

- msg/srv/action文件

ROS程序中有可能有一些自定义的消息/服务/动作文件，为程序的发者所设计的数据结构，这类的文件以.msg,.srv,.action结尾，通常放在package的msg/,srv/,action/路径下。

- urdf/xacro文件

urdf/xacro文件是机器人模型的描述文件，以.urdf或.xacro结尾。它定义了机器人的连杆和关节的信息，以及它们之间的位置、角度等信息，通过urdf文件可以将机器人的物理连接信息表示出来。并在可视化调试和仿真中显示。


- yaml文件

yaml文件一般存储了ROS需要加载的参数信息，一些属性的配置。通常在launch文件或程序中读取.yaml文件，把参数加载到参数服务器上。通常我们会把yaml文件存放在param/路径下

- dae/stl文件

dae或stl文件是3D模型文件，机器人的urdf或仿真环境通常会引用这类文件，它们描述了机器人的三维模型。相比urdf文件简单定义的性状，dae/stl文件可以定义复杂的模型，可以直接从solidworks或其他建模软件导出机器人装配模型，从而显示出更加精确的外形。

- rviz文件

rviz文件本质上是固定格式的文本文件，其中存储了RViz窗口的配置（显示哪些控件、视角、参数）。通常rviz文件不需要我们去手动修改，而是直接在RViz工具里保存，下次运行时直接读取。

