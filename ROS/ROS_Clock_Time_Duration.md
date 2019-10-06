# [Clock](http://wiki.ros.org/action/fullsearch/Clock?action=fullsearch&context=180&value=linkto%3A"Clock")/时钟

## 概述

通常，ROS客户端库ROS [client libraries](http://wiki.ros.org/Client Libraries)(roscpp, rospy等)将使用计算机的系统时钟作为时间源，也称为“wall-clock”或“"wall-time”（例如实验室墙壁上的时钟）。 但是，当你运行模拟或回放记录的数据时，通常希望让系统使用模拟时钟，以便你可以对系统的感知时间进行加速，减速或步进控制。 例如，如果要将传感器数据回放到系统中，则可能希望时间与传感器数据的时间戳相对应。

为此，ROS客户端库可以侦听/ clock话题，该话题用于发布“模拟时间”。

为了使你的代码能够更好的利用ROS的“模拟时间”，所有代码都必须使用适当的ROS客户端库的Time API来进行与时间和睡眠相关的操作，而不要使用语言本身的机制，这一点很重要。 无论使用 wall-clock 还是模拟时钟，这都将使您的系统具有一致的时间量度。 ROS客户端库的Time API 都有相关的详细的描述

在多台计算机上使用 wall-clock time时，同步它们之间的时间很重要。 ROS不提供此功能，因为已经有完善的方法（例如ntp，即Network Time Protocol，官方推荐的同步工具为chrony）可以执行此操作。 如果你不同步多台计算机的wall-clocks，它们将无法像tf中所使用的那样进行时间计算。

**NOTE: this is not an API for real-time systems!**

**注意：这不是用于实时系统的API！**

## 使用/clock话题中的模拟时间

为了使ROS节点使用/ clock话题的仿真时间，必须在初始化节点之前将/ use_sim_time参数设置为true。 这可以在launch文件中或从命令行完成。

如果设置了/ use_sim_time参数，则ROS Time API将返回时间= 0，直到它从/ clock主题收到一个值为止。 然后，时间值仅在收到来自/ clock话题的新消息时才更新，并且在两次更新之间保持不变。

如果要使用模拟时间来计算时间的持续值（ time durations），客户端应该始终等到收到第一个非零时间值后再开始，因为/ clock话题的第一个模拟时间值可能是一个很高的值。

## 运行时钟服务器（Clock Server）

时钟服务器（Clock Server）可以是发布消息到/ clock话题的任何节点，并且在单个ROS网络中运行的节点不得超过一个。 在大多数情况下，时钟服务器是模拟器或日志回放工具。

为了解决启动顺序的问题，需要将使用时钟服务器的所有launch文件中的/ use_sim_time参数设置为true。 如果使用rosbag play播放bag文件时使用--clock选项，将在播放bag文件时运行Clock Server。

## Clock Message (/Clock话题的消息)

```shell
# roslib/Clock is used for publishing simulated time in ROS. 
# This message simply communicates the current time.
# For more information, see http://www.ros.org/wiki/Clock
time clock
```

## Client Libraries(在客户端中使用 Time API)

下面是roscpp客户端库中使用ROS Time API的一些简单示例。

>  注意：有关更多文档，请查阅下面列出的相关ROS客户端库软件包的“Code API”。

```cpp
//Get the time and store it in the time variable.
ros::Time time = ros::Time::now();
//Wait a duration of one second.
ros::Duration d = ros::Duration(1, 0);
d.sleep();
```

[C++ (roscpp) Time API](http://wiki.ros.org/roscpp/Overview/Time)

[Python (rospy) Time API](http://wiki.ros.org/rospy/Overview/Time)

# C++ Time API

## Time and Duration

源码：[ros::TimeBase API docs](http://docs.ros.org/latest/api/rostime/html/classros_1_1TimeBase.html), [ros::DurationBase API docs](http://docs.ros.org/latest/api/rostime/html/classros_1_1DurationBase.html)

ROS具有内置的时间(time)和时长(duration)基本类型，[roslib](http://wiki.ros.org/roslib)提供对应的ros :: Time和ros :: Duration类。 时间是特定时间段（例如“今天下午5点”），而时长则是一段时间（例如“ 5小时”）。 持续时间可以是负数。

时间和时长具有相同的表示形式：

```ros
int32 sec
int32 nsec
```

ROS能够为节点设置模拟时钟(simulated Clock)。 在ros中应该使用roscpp's time routines(time API)来获取当前时间，而非使用platform time routines，roscpp's time routines可与模拟时钟时间(simulated Clock)和wall-clock时间无缝地协同工作。

> 按照自己的理解，不管是要使用Ubuntu系统时间即wall-clock还是要使用ros模拟的时间即simulated Clock，都需要通过ROS Time API来完成。

## Getting the Current Time（获取当前时间）

使用方法 ros::Time::now()获取当前时间，方法返回值类型为ros::Time：

```cpp
ros::Time begin = ros::Time::now();
```

- ros::Time 类重载了 << 运算符，可使用cout进行输出。
- ros::Time 类cout输出例如 `1570389986.144982211`，小数点后面是nsec，即纳秒

### Time zero（time为0时）

使用模拟时钟时间时，在话题 /clock上收到第一条消息之前，now（）方法返回时间的时间为0，因此0实质上意味着节点尚不知道时钟时间。 因此，time=0 可以用作判断条件，例如循环遍历now（）直到返回非零。

## 创建 Time 和 Duration 实例

可以将“Time”或“Duration”创建为特定值，即浮点秒数：

```cpp
// 使用ros::Time类的构造函数初始化一个Time实例
ros::Time a_little_after_the_beginning(0.001);
// 使用ros::Duration类的构造函数初始化一个Duration实例
ros::Duration five_seconds(5.0);
```





## 示例

```shell
# 在ros工作空间的src目录下
catkin_create_pkg learning_time roscpp
cd learning_time/src
touch time.cpp
```

