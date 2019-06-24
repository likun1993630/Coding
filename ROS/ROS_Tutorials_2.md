# 创建ROS消息和ROS服务

- 消息(msg): msg文件就是一个描述ROS中所使用消息类型的简单文本。它们会被用来生成不同语言的源代码。
- 服务(srv): 一个srv文件描述一项服务。它包含两个部分：请求和响应。
> msg文件存放在package的msg目录下，srv文件则存放在srv目录下。

msg文件实际上就是每行声明一个数据类型和变量名。可以使用的数据类型如下： 

- int8, int16, int32, int64 (plus uint*) 
- float32, float64 
- string 
- time, duration 
- other msg files 
- variable-length array[] and fixed-length array[C] 
> ROS中的float32和float64类型都对应到Python的float类型

在ROS中有一个特殊的数据类型：Header，它含有时间戳和坐标系信息。在msg文件的第一行经常可以看到Header header的声明.
下面是一个msg文件的样例，它使用了Header，string，和其他另外两个消息类型。

```
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

srv文件分为请求和响应两部分，由'---'分隔。下面是srv的一个样例：其中 A 和 B 是请求, 而Sum 是响应。 
```
int64 A
int64 B
---
int64 Sum
```
## 使用 msg
创建一个 msg：
```shell
# 在之前创建的package里定义新的消息。
$ cd ~/catkin_ws/src/beginner_tutorials
$ mkdir msg
$ echo "int64 num" > msg/Num.msg
```

> 上面是最简单的例子——在.msg文件中只有一行数据。当然，你可以仿造上面的形式多增加几行以得到更为复杂的消息： 
```shell
string first_name
string last_name
uint8 age
uint32 score
```

接下来，还有关键的一步：我们要确保msg文件被转换成为C++，Python和其他语言的源代码：

查看package.xml, 确保它包含一下两条语句: 
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
> 在构建的时候，我们只需要"message_generation"。
> 然而，在运行的时候，我们只需要"message_runtime"。

在编辑器中打开CMakeLists.txt文件。

在 CMakeLists.txt文件中，利用find_packag函数，增加对message_generation的依赖，这样就可以生成消息了。find_package()调用末尾添加的message_generation，这样catkin就知道去啦里找message_generation包：
```
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    std_msgs 
    message_generation # 添加这一行
)
```
> 有时候你会发现，即使你没有调用find_package,你也可以编译通过。这是因为catkin把你所有的package都整合在一起，因此，如果其他的package调用了find_package，你的package的依赖就会是同样的配置。但是，在你单独编译时，忘记调用find_package会很容易出错。


通过在add_message_files()调用的末尾添加消息定义文件来告知catkin我们想要编译他们：
```
add_message_files(
  FILES
  Num.msg
)
```

然后需要去掉generate_message()调用的注释，并已经包含了消息所依赖的所有依赖项：
```
generate_messages(
    DEPENDENCIES
    std_msgs
)
```

最后告知catkin我们将在运行时使用消息，即在catkin_package()调用末尾添加message_runtime:
```
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```


