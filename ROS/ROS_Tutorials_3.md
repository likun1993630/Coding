# 使用自己创建的消息msg Num.msg

Num.msg：
```
int64 num
```

## 创建Node 用来发布话题
```shell
$ roscd beginner_tutorials/scripts
$ touch talkernum.py
$ chmod +x talkernum.py
```

talkernum.py 内容：
```python
#!/usr/bin/env python
import rospy
from beginner_tutorials.msg import Num

def talker():
    pub = rospy.Publisher('chatternum', Num, queue_size=10)
    rospy.init_node('talkernum', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 0
    while not rospy.is_shutdown():
        i = i + 1
        numi = i
        rospy.loginfo(numi)
        pub.publish(numi)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
> `from beginner_tutorials.msg import Num` 从beginner_tutorials包中的子包msg中导入Num模块。

> `rospy.loginfo(numi)` `pub.publish(numi)` 也可以使用类的示例化调用。

Num.msg的另一用用法：
```python
#!/usr/bin/env python
import rospy
from beginner_tutorials.msg import Num

def talker():
    pub = rospy.Publisher('chatternum', Num, queue_size=10)
    rospy.init_node('talkernum', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 0
    while not rospy.is_shutdown():
        i = i + 1
        msg = Num()
        msg.num = i
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

## 创建Node 用来接受话题

```shell
$ roscd beginner_tutorials/scripts
$ touch listenernum.py
$ chmod +x listenernum.py
```

listenernum.py内容：

```python
#!/usr/bin/env python
import rospy
from beginner_tutorials.msg import Num

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %d", msg.num)
    
def listener():
    rospy.init_node('listenernum', anonymous=True)
    rospy.Subscriber("chatternum", Num, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

## 测试消息发布器和订阅器

```shell
# 新终端
$ roscore

# 新终端
$ rosrun beginner_tutorials talkernum.py

# 新终端
$ rosrun beginner_tutorials listenernum.py
```

