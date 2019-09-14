# Rospy

ROS的客户端库（Client Libarary）简单的理解就是一套接口，ROS为我们机器人开发者提供了不同语言的接口，比如roscpp是C++语言ROS接口，rospy是python语言的ROS接口，我们直接调用它所提供的函数就可以实现topic、service等通信功能。

## Client Library简介

ROS为机器人开发者们提供了不同语言的编程接口，比如C++接口叫做roscpp，Python接口叫做rospy，Java接口叫做rosjava。尽管语言不通，但这些接口都可以用来创建topic、service、param，实现ROS的通信功能。Clinet Lirary有点类似开发中的Helper Class，把一些常用的基本功能做了封装。

目前ROS支持的Clinet Library包括：

| Client Library | 介绍                                                         |
| -------------- | ------------------------------------------------------------ |
| roscpp         | ROS的C++库，是目前最广泛应用的ROS客户端库，执行效率高        |
| rospy          | ROS的Python库，开发效率高，通常用在对运行时间没有太大要求的场合，例如配置、初始化等操作 |
| ...            | ...                                                          |

> 目前最常用的只有roscpp和rospy

从开发客户端库的角度看，一个客户端库，至少需要能够包括master注册、名称管理、消息收发等功能。这样才能给开发者提供对ROS通信架构进行配置的方法。

整个ROS包括的packages如下，你可以看到roscpp、rospy处于什么位置。

![](../../ROS/res/ros_pkgs.png) 

rospy是Python版本的ROS客户端库，提供了Python编程需要的接口，你可以认为rospy就是一个Python的模块(Module)。这个模块位于/opt/ros/kineetic/lib/python2.7/dist-packages/rospy之中。

rospy包含的功能与roscpp相似，都有关于node、topic、service、param、time相关的操作。但同时rospy和roscpp也有一些区别：

```
rospy没有一个NodeHandle，像创建publisher、subscriber等操作都被直接封装成了rospy中的函数或类，调用起来简单直观。
rospy一些接口的命名和roscpp不一致，有些地方需要开发者注意，避免调用错误。
```

相比于C++的开发，用Python来写ROS程序开发效率大大提高，诸如显示、类型转换等细节不再需要我们注意，节省时间。但Python的执行效率较低，同样一个功能用Python运行的耗时会高于C++。因此我们开发SLAM、路径规划、机器视觉等方面的算法时，往往优先选择C++。

ROS中绝大多数基本指令，例如rostopic,roslaunch都是用python开发的，简单轻巧。 

通常我们常用的ROS命令，大多数其实都是一个个Python模块，源代码存放在ros_comm仓库的tools路径下：
https://github.com/ros/ros_comm/tree/lunar-devel/tools 
你可以看到每一个命令行工具（如rosbag、rosmsg）都是用模块的形式组织核心代码，然后在script/下建立一个脚本来调用模块。

## 常用rospy的API

> 具体API请查看http://docs.ros.org/api/rospy/html/rospy-module.html

- node 相关

| 返回值      | 方法                                              | 作用                   |
| ----------- | ------------------------------------------------- | ---------------------- |
|             | rospy.init_node(name, argv=None, anonymous=False) | 注册和初始化node       |
| MasterProxy | rospy.get_master()                                | 获取master的句柄       |
| bool        | rospy.is_shutdown()                               | 节点是否关闭           |
|             | rospy.on_shutdown(fn)                             | 在节点关闭时调用fn函数 |
| str         | get_node_uri()                                    | 返回节点的URI          |
| str         | get_name()                                        | 返回本节点的全名       |
| str         | get_namespace()                                   | 返回本节点的名字空间   |
| ...         | ...                                               | ...                    |

- topic 相关
  函数：

| 返回值       | 方法                                               | 作用                                                  |
| ------------ | -------------------------------------------------- | ----------------------------------------------------- |
| [[str, str]] | get_published_topics()                             | 返回正在被发布的所有topic名称和类型                   |
| Message      | wait_for_message(topic, topic_type, time_out=None) | 等待某个topic的message                                |
|              | spin()                                             | 触发topic或service的回调/处理函数，会阻塞直到关闭节点 |

Publisher类：

| 返回值 | 方法                                          | 作用     |
| ------ | --------------------------------------------- | -------- |
|        | init(self, name, data_class, queue_size=None) | 构造函数 |
|        | publish(self, msg)                            | 发布消息 |
| str    | unregister(self)                              | 停止发布 |

Subscriber类：

| 返回值 | 方法                                                         | 作用     |
| ------ | ------------------------------------------------------------ | -------- |
|        | init_(self, name, data_class, call_back=None, queue_size=None) | 构造函数 |
|        | unregister(self, msg)                                        | 停止订阅 |

- Service相关

函数：

| 返回值 | 方法                                    | 作用             |
| ------ | --------------------------------------- | ---------------- |
|        | wait_for_service(service, timeout=None) | 阻塞直到服务可用 |

Service类(server)：

| 返回值 | 方法                                     | 作用                                                |
| ------ | ---------------------------------------- | --------------------------------------------------- |
|        | init(self, name, service_class, handler) | 构造函数，handler为处理函数，service_class为srv类型 |
|        | shutdown(self)                           | 关闭服务的server                                    |

ServiceProxy类(client)：

| 返回值 | 方法                            | 作用                 |
| ------ | ------------------------------- | -------------------- |
|        | init(self, name, service_class) | 构造函数，创建client |
|        | call(self, args, *kwds)         | 发起请求             |
|        | call(self, args, *kwds)         | 同上                 |
|        | close(self)                     | 关闭服务的client     |

- Param相关

函数：

| 返回值           | 方法                                        | 作用                       |
| ---------------- | ------------------------------------------- | -------------------------- |
| XmlRpcLegalValue | get_param(param_name, default=_unspecified) | 获取参数的值               |
| [str]            | get_param_names()                           | 获取参数的名称             |
|                  | set_param(param_name, param_value)          | 设置参数的值               |
|                  | delete_param(param_name)                    | 删除参数                   |
| bool             | has_param(param_name)                       | 参数是否存在于参数服务器上 |
| str              | search_param()                              | 搜索参数                   |

- 时钟相关

函数：

| 返回值 | 方法            | 作用                   |
| ------ | --------------- | ---------------------- |
| Time   | get_rostime()   | 获取当前时刻的Time对象 |
| float  | get_time()      | 返回当前时间，单位秒   |
|        | sleep(duration) | 执行挂起               |

Time类：

| 返回值 | 方法                        | 作用                            |
| ------ | --------------------------- | ------------------------------- |
|        | init(self, secs=0, nsecs=0) | 构造函数                        |
| Time   | now()                       | 静态方法 返回当前时刻的Time对象 |

Duration类：

| 返回值 | 方法                        | 作用     |
| ------ | --------------------------- | -------- |
|        | init(self, secs=0, nsecs=0) | 构造函数 |

## topic in rospy

用python来写一个节点间消息收发的demo，同样还是创建一个自定义的gps类型的消息，一个节点发布模拟的gps信息，另一个接收和计算距离原点的距离。

### 自定义消息的生成

gps.msg定义如下：

```
string state   #工作状态
float32 x      #x坐标
float32 y      #y坐标
```

我们需要修改CMakeLists.txt文件，方法见5.3节，这里需要强调一点的就是，对创建的msg进行catkin_make会在~/catkin_ws/devel/lib/python2.7/dist-packages/topic_demo下生成msg模块（module）。 有了这个模块，我们就可以在python程序中from topic_demo.msg import gps,从而进行gps类型消息的读写。

