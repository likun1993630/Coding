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
下面这个例子声明一个话题并在这个话题上发布消息的基本代码，这个节点以2Hz的速率发送连续的整数到conter节点上。节点名称为topic_publisher。
```python
#!/usr/bin/env python
import rospy
form std_msg.msg import Int32

rospy.init_node('topic_publisher')

```
