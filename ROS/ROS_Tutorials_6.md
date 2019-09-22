# 消息发布器和订阅器 (自定义消息)(C++)

实现功能：

**自定义一个类型为gps的消息（包括位置x，y和工作状态state信息），一个node以一定频率发布模拟的gps消息，另一个node接收并处理，算出到原点的距离**

包的目录结构

```
topic_demo
├── CMakeLists.txt
├── msg
│   └── gps.msg
├── package.xml
└── src
    ├── listener.cpp
    └── talker.cpp
```

## gps.msg

```shell
# topic_demo/msg/gps.msg
string state   #工作状态
float32 x      #x坐标
float32 y      #y坐标
```

- 可以把gps.msg消息理解为结构体，即，使用成员运算符访问成员

  ```cpp
  struct gps
  {
      string state;
      float32 x;
      float32 y;
  }
  ```

当自定义msg文件后，需要修改`CMakeLists.txt`和`package.xml`，从而让系统能够编译自定义消息，即根据消息gps.msg文件生成消息头文件gps.h。生成的头文件包含类的所有实现。

CMakeLists.txt

```cmake
# 针对于gps.msg的修改

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
message_generation   #需要添加的地方
)

add_message_files(FILES gps.msg)  
# 指定从哪个消息文件生成对应的头文件

generate_messages(DEPENDENCIES std_msgs) 
# DEPENDENCIES后面指定生成msg需要依赖其他什么消息，
# 由于gps.msg用到了flaot32这种ROS标准消息，因此需要再把std_msgs作为依赖

catkin_package( CATKIN_DEPENDS std_msgs message_runtime)
# 不影响消息的生成，如果不发布包，可以省略
# 给其他的包（即别人如果要使用你的包）声明消息的依赖项：
```

package.xml

```xml
<!-- 如果不发布包，可以忽略package.xml -->
<!-- 针对gps.msg的修改 -->
<depend>std_msgs</depend>

<build_depend>message_generation</build_depend>
<build_export_depend>message_runtime</build_export_depend>
<exec_depend>message_runtime</exec_depend>
```

修改完之后既可catkin_make编译。此处选择之后编译。

编译完成后，在`devel`路径下生成`gps.msg`对应的头文件，头文件按照C++的语法规则定义了`topic_demo::gps`类型的数据。gps类是在**namespace topic_demo**命名空间下定义的。要在代码中使用自定义消息类型，只要`#include <topic_demo/gps.h>`，然后声明，按照对结构体操作的方式修改内容即可。

```cpp
topic_demo::gps mygpsmsg;
mygpsmsg.x = 1.6;
mygpsmsg.y = 5.5;
mygpsmsg.state = "working";
```

## 消息发布节点

topic_demo/src/talker.cpp

```cpp
//ROS头文件
#include <ros/ros.h>
//自定义msg产生的头文件
#include <topic_demo/gps.h>

int main(int argc, char **argv)
{
  //用于解析ROS参数，第三个参数为本节点名
  ros::init(argc, argv, "talker");
  //实例化句柄，初始化node
  ros::NodeHandle nh;
  //自定义gps消息并初始化
  topic_demo::gps msg;
  msg.x = 1.0;
  msg.y = 1.0;
  msg.state = "working";
  //创建publisher
  ros::Publisher pub = nh.advertise<topic_demo::gps>("gps_info", 1);
  //定义发布的频率 1 Hz
  ros::Rate loop_rate(1.0);
  //循环发布msg
  while (ros::ok())
  {
    //以指数增长，每隔1秒更新一次
    msg.x = 1.03 * msg.x ;
    msg.y = 1.01 * msg.y;
    ROS_INFO("Talker: GPS: x = %f, y = %f ",  msg.x ,msg.y);
    //以1Hz的频率发布msg
    pub.publish(msg);
    //根据前面定义的频率, sleep 1s
    loop_rate.sleep();//根据前面的定义的loop_rate,设置1s的暂停
  }
  return 0;
} 
```

## 消息接收节点

topic_demo/src/listener.cpp

```cpp
//ROS头文件
#include <ros/ros.h>
//包含自定义msg产生的头文件
#include <topic_demo/gps.h>
//ROS标准msg头文件
#include <std_msgs/Float32.h>

void gpsCallback(const topic_demo::gps::ConstPtr &msg)
{  
    //计算离原点(0,0)的距离
    std_msgs::Float32 distance;
    distance.data = sqrt(pow(msg->x,2)+pow(msg->y,2));
    //float distance = sqrt(pow(msg->x,2)+pow(msg->y,2));
    ROS_INFO("Listener: Distance to origin = %f, state: %s",
             distance.data,msg->state.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("gps_info", 1, gpsCallback);
  //ros::spin()用于调用所有可触发的回调函数。
  //将进入循环，不会返回，类似于在循环里反复调用ros::spinOnce()。
  //而ros::spinOnce()只会去触发一次
  ros::spin(); 
  return 0;
}
```

- 在topic接收方，有一个比较重要的概念，就是**回调(CallBack)**，在本例中，回调就是预先给`gps_info`话题传来的消息准备一个回调函数，你事先定义好回调函数的操作，本例中是计算到原点的距离。只有当有消息来时，回调函数才会被触发执行。具体去触发的命令就是`ros::spin()`，它会反复的查看有没有消息来，如果有就会让回调函数去处理。
- 因此千万不要认为，只要指定了回调函数，系统就回去自动触发，你必须`ros::spin()`或者`ros::spinOnce()`才能真正使回调函数生效。

## CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(topic_demo)

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
message_generation
)

add_message_files(FILES gps.msg)  
# 指定从哪个消息文件生成对应的头文件

generate_messages(DEPENDENCIES std_msgs) 
# DEPENDENCIES后面指定生成msg需要依赖其他什么消息，
# 由于gps.msg用到了flaot32这种ROS标准消息，因此需要再把std_msgs作为依赖

catkin_package( CATKIN_DEPENDS roscpp std_msgs message_runtime)
# 不影响消息的生成，如果不发布包，可以省略
# 给其他的包（即别人如果要使用你的包）声明消息的依赖项

include_directories(${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp )
# #表明在编译talker前，必须先生编译完成自定义消息
add_dependencies(talker topic_demo_generate_messages_cpp)
# #add_dependencies(talker ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp )
add_dependencies(listener topic_demo_generate_messages_cpp)
# #add_dependencies(listener ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(listener ${catkin_LIBRARIES})
```

## package.xml

```xml
<!-- 如果不发布包，可以忽略package.xml -->
<?xml version="1.0"?>
<package>
  <name>topic_demo</name>
  <version>0.0.0</version>
  <description>The publish_subscribe_demo package</description>
  <maintainer email="hanhaomin008@126.com">davidhan</maintainer>
  <license>BSD</license>
    
  <buildtool_depend>catkin</buildtool_depend>
  <depend>std_msgs</depend>
  <depend>roscpp</depend>
    
  <build_depend>message_generation</build_depend>
  <build_export_depend>message_runtime</build_export_depend>
  <exec_depend>message_runtime</exec_depend>
    
  <export></export>
</package>
```

## 编译

```
catkin_make
```

编译完成后：

```shell
devel
├── cmake.lock
├── env.sh
├── include
│   └── topic_demo
│       └── gps.h # 头文件
├── lib
│   ├── pkgconfig
│   │   └── topic_demo.pc
│   ├── python2.7
│   │   └── dist-packages
│   │       └── topic_demo
│   │           ├── __init__.py
│   │           └── msg
│   │               ├── _gps.py
│   │               └── __init__.py
│   └── topic_demo
│       ├── listener # 节点，可执行文件
│       └── talker # 节点，可执行文件
├── local_setup.bash
├── local_setup.sh
├── local_setup.zsh
......
```

## 测试

```shell
roscore
rosrun topic_demo talker
rosrun topic_demo listener
```

```shell
# talker：
[ INFO] [1569152168.941182627]: Talker: GPS: x = 1.030000, y = 1.010000 
[ INFO] [1569152169.941226998]: Talker: GPS: x = 1.060900, y = 1.020100 
[ INFO] [1569152170.941570413]: Talker: GPS: x = 1.092727, y = 1.030301 
[ INFO] [1569152171.941297153]: Talker: GPS: x = 1.125509, y = 1.040604 

# listener：
[ INFO] [1569152184.941929161]: Listener: Distance to origin = 2.033343, state: working
[ INFO] [1569152185.941891324]: Listener: Distance to origin = 2.080636, state: working
[ INFO] [1569152186.941492955]: Listener: Distance to origin = 2.129392, state: working
[ INFO] [1569152187.941849365]: Listener: Distance to origin = 2.179656, state: working
```

