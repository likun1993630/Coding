## `__name` 重命名节点并启动
```shell
roscd rospy_tutorials/001_talker_listener/
./talker __name:=talkerlikun
```

```shell
roscd rospy_tutorials/001_talker_listener/
./listener __name:=listenerlikun
```
![](./res/rostopic1.png) 

## 话题 topic
- 话题实现了一种发布/订阅（publish/subscribe）通讯机制。
- 在同一话题上的所有消息必须是同一类型

### 将消息发布到话题上
下面这个例子声明一个话题并在这个话题上发布消息的基本代码，这个节点以2Hz的速率发送连续的整数到conter话题上。节点名称为topic_publisher。

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32

rospy.init_node('topic_publisher')
pub = rospy.Publisher('counter', Int32, queue_size=10)
rate = rospy.Rate(2)

count = 0
while not rospy.is_shutdown():
    pub.publish(count)
    count +=1
    rate.sleep()

```

代码解释：
`#!/usr/bin/env python` 告诉操作系统这个一个python文件，应该传递给一个python解释器。

`from std_msg.msg import Int32` 这里使用了ROS的标准消息包std_msgs. 因为我们使用了一个来自其他包的消息，我们需要告诉ROS的构建系统，也就是在package.xml文件中添加一个依赖（dependency）：`<depend package="std_msgs" />` ,如果没有这个依赖项，ROS将不知道去哪里去找消息的定义，节点就无法运行。

`pub = rospy.Publisher('counter', Int32, queue_size=10)` 赋予话题一个名字（counter）并说明该话题上发布的消息类型是（Int32）。 queue_size=10 列队长度为10

`rate = rospy.Rate(2)` 与`rate.sleep()`保证消息以2Hz的频率发布消息

`is_shutdown()` 函数在节点被关闭时返回一个True

`pub.publish(count)` 发布话题

给该文件增加执行权限：

```shell
chmod u+x topic_publisher.py
```

