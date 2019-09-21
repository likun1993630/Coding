# 编写简单的消息发布器和订阅器 (C++)

### 编写发布器节点

『节点』(Node) 是指 ROS 网络中可执行文件。接下来，我们将会创建一个发布器节点("talker")，它将不断的在 ROS 网络中广播消息。 

切换到之前创建的 beginner_tutorials package 路径下： 

```shell
cd ~/catkin_ws/src/beginner_tutorials
```

#### 源代码

代码需要实现：

- 初始化 ROS 系统 

- 在 ROS 网络内广播我们将要在 chatter 话题上发布 [std_msgs/String](http://docs.ros.org/api/std_msgs/html/msg/String.html) 类型的消息 

- 以每秒 10 次的频率在 chatter 上发布消息 

在 beginner_tutorials package 路径下创建一个`src`文件夹： 

```shell
mkdir -p ~/catkin_ws/src/beginner_tutorials/src
```
talker.cpp

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
      
    chatter_pub.publish(msg);
      
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
```

#### 代码说明

```cpp
// ros/ros.h 是一个实用的头文件，它引用了 ROS 系统中大部分常用的头文件。 
#include "ros/ros.h"
// 这引用了 std_msgs/String 消息, 
// 它存放在 std_msgs package 里，是由 String.msg 文件自动生成的头文件
// String.h头文件中，声明并实现了String模板类
#include "std_msgs/String.h"

#include <sstream>
int main(int argc, char **argv)
{
	/*
    ros :: init（）为节点初始化，函数需要传递命令行参数argc和argv，
    以便它可以执行命令行中提供的任何ROS参数和名称重映射。
    对于编程式重新映射，可以使用init（）的其他重载形式直接进行重新映射，
    但是对于大多数命令行程序，传递argc和argv是最简单的方法。
    init（）的第三个参数是节点的名称。在使用ROS系统的任何其他部分之前，
    必须调用ros :: init（）进行节点初始化。
    节点的名称必须是一个 base name ，也就是说，名称内不能包含 / 等符号。 
    */
  ros::init(argc, argv, "talker");

    /*
    NodeHandle是与ROS系统通信的主要访问点。
    构造的第一个NodeHandle将完全初始化此节点，而销毁的最后一个NodeHandle将关闭该节点。
    */
  ros::NodeHandle n;
    
    /*
    通过advertise（）函数告诉ROS要在给定的话题上发布消息。
    通过调用该函数会通知ROS Master节点，ROS Master有一个注册表，
    注册表里有记录这谁正在发布和正在订阅某个消息。 
    进行了advertise（）调用后，ROS Master将通知任何试图订阅此话题的其他节点，
    然后他们将与此节点连接。 
    advertise（）返回一个Publisher对象，该对象使你可以通过调用publish（）来发布有关该话题的消息。
    一旦销毁了返回的Publisher对象的所有副本，该主题将自动取消发布。 
    advertise（）的第二个参数是用于发布消息的消息队列的长度。 
    If messages are published more quickly
    than we can send them, the number here specifies 
    how many messages to buffer up before throwing some away.
    在这里：
    告诉 master 我们将要在 chatter（话题名） 上发布 std_msgs/String 消息类型的消息。
    这样 master 就会告诉所有订阅了 chatter 话题的节点，将要有数据发布。
    第二个参数是发布序列的大小。
    如果我们发布的消息的频率太高，缓冲区中的消息在大于 1000 个的时候就会开始丢弃先前发布的消息。
    NodeHandle::advertise() 返回一个 ros::Publisher 对象,它有两个作用： 
    1) 它有一个 publish() 成员函数可以让你在topic上发布消息； 
    2) 如果消息类型不对,它会拒绝发布。 
    */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

    /*
    ros::Rate 对象可以允许你指定自循环的频率。
    它会追踪记录自上一次调用 Rate::sleep() 后时间的流逝，并休眠直到一个频率周期的时间。 
    在这个例子中，我们让它以 10Hz 的频率运行。
    */
  ros::Rate loop_rate(10); //这里即为构造一个ros::Rate对象loop_rate

    /**
     count时已发送消息的计数
     这用于为每个消息创建唯一的标识。
    */
  int count = 0;
    
    /*
    roscpp 会默认生成一个 SIGINT 句柄，
    它负责处理 Ctrl-C 键盘操作——使得 ros::ok() 返回 false。
	如果下列条件之一发生，ros::ok() 返回false：
    1.SIGINT 被触发 (Ctrl-C)
    2.被另一同名节点踢出 ROS 网络
    3.ros::shutdown() 被程序的另一部分调用
    4.节点中的所有 ros::NodeHandles 都已经被销毁 
    一旦 ros::ok() 返回 false, 所有的 ROS 调用都会失效。 
    */
  while (ros::ok())
  {
    std_msgs::String msg; //声明对象 msg
      //msg只有一个数据成员 "data"，可以通过msg.data访问data成员

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
      //ROS_INFO 和其他类似的函数可以用来代替 printf/cout 等函数
      
      /* 通过publish（）函数发布消息。函数参数是消息对象（C++类对象）。 
      该对象的类型必须与在advertise <>（）调用中作为模板参数给出的类型一致，
      这里消息对象的类型为：std_msgs::String
      */
    chatter_pub.publish(msg);
      
      /*
      在这个例子中并不是一定要调用 ros::spinOnce()，因为我们不接受回调。
      然而，如果你的程序里包含其他回调函数，最好在这里加上 ros::spinOnce()这一语句，
      否则你的回调函数就永远也不会被调用了。 
      */
    ros::spinOnce();
    loop_rate.sleep(); //调用ros::Rate对象的公有方法sleep()
      //调用 ros::Rate 对象来休眠一段时间以使得发布频率为 10Hz。 
    ++count;
  }
  return 0;
}
```

std_msgs/String.h

- 位于`/opt/ros/kinetic/include/std_msgs/String.h`
- String.h 内包含所有实现

String.msg

- 位于`/opt/ros/kinetic/share/std_msgs/msg/String.msg`

- 用与生成String.h头文件

- 内容：

  ```
  string data
  ```

> 关于msg， 参考http://wiki.ros.org/msg
>
> 关于ROS_INFO，参考http://wiki.ros.org/rosconsole

### 编写订阅器节点

实现功能：

- 初始化ROS系统 

- 订阅 `chatter` 话题 

- 进入自循环，等待消息的到达 

- 当消息到达，调用 `chatterCallback()` 函数 

#### 源代码

*listener.cpp*

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::spin();
  return 0;
}
```

#### 代码说明

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

/*
这是一个回调函数，当接收到 chatter 话题的时候就会被调用。
消息是以 boost shared_ptr const指针的形式传输，这就意味着你不需要复制原始数据就能放访问它，
并且不能通过该指针修改消息。
std_msgs::String::ConstPtr为String.h内定义的共享指针shared_ptr。
*/
void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
	/*
    ros :: init（）为节点初始化，函数需要传递命令行参数argc和argv，
    以便它可以执行命令行中提供的任何ROS参数和名称重映射。
    对于编程式重新映射，可以使用init（）的其他重载形式直接进行重新映射，
    但是对于大多数命令行程序，传递argc和argv是最简单的方法。
    init（）的第三个参数是节点的名称。在使用ROS系统的任何其他部分之前，
    必须调用ros :: init（）进行节点初始化。
    节点的名称必须是一个 base name ，也就是说，名称内不能包含 / 等符号。 
    */
  ros::init(argc, argv, "listener");

    /*
    NodeHandle是与ROS系统通信的主要访问点。
    构造的第一个NodeHandle将完全初始化此节点，而销毁的最后一个NodeHandle将关闭该节点。
    */
  ros::NodeHandle n;
    
    /**
	调用subscription（）是告诉ROS你要接收给定话题的消息。 
	这将调用ROS Master节点，该调用将保留谁正在发布和正在订阅的注册表。 
	消息被传递到回调函数，这里称为chatterCallback。 
	subscription（）返回一个订阅者对象，必须保留该对象直到要取消订阅。
    当Subscriber对象的所有副本在作用域不再存在时，此回调将自动从该主题退订。
	subscription（）函数的第二个参数是消息队列的大小。 
	如果消息到达队列的速度快于它们的被处理速度，则丢弃被缓存的最旧的消息。
	在这里：
	告诉ROS master 我们要订阅 chatter 话题上的消息。
	当有消息发布到这个话题时，ROS 就会调用 chatterCallback() 函数。
	第二个参数是队列大小，以防我们处理消息的速度不够快，
	当缓存达到 1000 条消息后，再有新的消息到来就将开始丢弃先前接收的消息。
	NodeHandle::subscribe() 返回 ros::Subscriber 对象,
	你必须让它处于活动状态直到你不再想订阅该消息。
	当这个对象销毁时，它将自动退订 chatter 话题的消息。 
	NodeHandle::subscribe() 函数有很多重载
	*/
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  	/*
  	只有当有消息来时，回调函数才会被触发执行。
  	具体去触发的命令就是ros::spin()，它会反复的查看有没有消息来，如果有就会让回调函数去处理。
	即，ros :: spin（）使程序进入循环，循环调用callback函数。 
	在此版本中，所有回调都将从该线程（主要线程）中调用。 
	按下Ctrl-C或主节点关闭节点时，ros :: spin（）将退出。
	ros::spin() 进入自循环，可以尽可能快的调用消息回调函数。
	如果没有消息到达，它不会占用很多 CPU，所以不用担心。（因为函数没被调用）
	一旦 ros::ok() 返回 false，ros::spin() 就会立刻跳出自循环。
   	*/
  ros::spin();
  return 0;
}
```

## 回调函数与spin()方法

回调函数作为参数被传入到了另一个函数中（在本例中传递的是函数指针），在未来某个时刻（当有新的message到达），就会立即执行。Subscriber接收到消息，实际上是先把消息放到一个**队列**中去，如图所示。队列的长度在Subscriber构建的时候设置好了。当有spin函数执行，就会去处理消息队列中队首的消息。

![img](res/cb_queue.png)

spin具体处理的方法又可分为阻塞/非阻塞,单线程/多线程，在ROS函数接口层面我们有4种spin的方式：

|            spin方法             |  阻塞  |  线程  |
| :-----------------------------: | :----: | :----: |
|          `ros::spin()`          |  阻塞  | 单线程 |
|        `ros::spinOnce()`        | 非阻塞 | 单线程 |
|   `ros::MultiThreadedSpin()`    |  阻塞  | 多线程 |
| `ros::AsyncMultiThreadedSpin()` | 非阻塞 | 多线程 |

单线程与多线程的区别：

![img](res/single-multi-spin.png)

我们常用的`spin()`、`spinOnce()`是单个线程逐个处理回调队列里的数据。有些场合需要用到多线程分别处理，则可以用到`MultiThreadedSpin()`、`AsyncMultiThreadedSpin()`。

### 编译节点

```shell
# 创建包
catkin_create_pkg beginner_tutorialscpp roscpp std_msgs
```

包的结构如下：

```shell
└── src
    ├── beginner_tutorialscpp
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── beginner_tutorialscpp #注意这只是个空的文件夹
    │   ├── package.xml
    │   └── src
    │       ├── listener.cpp
    │       └── talker.cpp
    └── CMakeLists.txt -> /opt/ros/kinetic/share/catkin/cmake/toplevel.cmake
```

CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorialscpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
```

 package.xml

- 默认生成，可保持不变

```xml
<?xml version="1.0"?>
<package format="2">
  <name>beginner_tutorialscpp</name>
  <version>0.0.0</version>
  <description>The beginner_tutorialscpp package</description>
  <maintainer email="likun@todo.todo">likun</maintainer>
  <license>TODO</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <build_export_depend>std_msgs</build_export_depend>
  <exec_depend>roscpp</exec_depend>
  <exec_depend>std_msgs</exec_depend>

  <export>
  </export>
</package>
```

修改完CMakeLists.txt 和 package.xml 之后进行编译

```shell
catkin_make
```

- 编译的中间文件在build文件夹中
- 编译最后的最终文件放在devel文件夹中

```shell
➜ devel tree
.
├── cmake.lock
├── env.sh
├── lib
│   ├── beginner_tutorialscpp
│   │   ├── listener # 可执行文件
│   │   └── talker # 可执行文件
│   └── pkgconfig
│       └── beginner_tutorialscpp.pc
├── local_setup.bash
├── local_setup.sh
├── local_setup.zsh
├── setup.bash
├── setup.sh
├── _setup_util.py
├── setup.zsh
└── share
    └── beginner_tutorialscpp
        └── cmake
            ├── beginner_tutorialscppConfig.cmake
            └── beginner_tutorialscppConfig-version.cmake
```

#### 运行测试

```
roscore
rosrun beginner_tutorialscpp talker 
rosrun beginner_tutorialscpp listener
```

```shell
# talker：
[ INFO] [1569095412.677656146]: hello world 1092
[ INFO] [1569095412.777314125]: hello world 1093
[ INFO] [1569095412.877324116]: hello world 1094

# listener：
[ INFO] [1569095412.777603766]: I heard: [hello world 1093]
[ INFO] [1569095412.877717264]: I heard: [hello world 1094]
[ INFO] [1569095412.977583866]: I heard: [hello world 1095]
```

#### C++深入内容参考

- [roscpp](http://wiki.ros.org/roscpp)
- [Overview](http://wiki.ros.org/action/fullsearch/roscpp/Overview?action=fullsearch&context=180&value=linkto%3A"roscpp%2FOverview")
- [roscpp_tutorials](http://wiki.ros.org/action/fullsearch/roscpp_tutorials?action=fullsearch&context=180&value=linkto%3A"roscpp_tutorials")
- [catkin/CMakeLists.txt](http://wiki.ros.org/catkin/CMakeLists.txt)
- [catkin/Tutorials/using_a_workspace#With_catkin_make](http://wiki.ros.org/catkin/Tutorials/using_a_workspace#With_catkin_make)