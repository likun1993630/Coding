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
