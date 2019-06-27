# ROS 学习网站

官方中文入门教程：
http://wiki.ros.org/cn/ROS/Tutorials

实验楼官方中文中级教程搬运：
https://www.shiyanlou.com/courses/938

Turtlesim官方教程：
http://wiki.ros.org/turtlesim?distro=kinetic

Rospy官方教程：
http://wiki.ros.org/rospy_tutorials

CKZZ教程：
https://www.ncnynl.com/beginning.html

CKZZros-python教程：
https://www.ncnynl.com/category/ros-python/

中科院与Xbot视频教程+gitbook书籍（5星推荐）
https://www.bilibili.com/video/av24585414
https://legacy.gitbook.com/book/sychaichangkun/ros-tutorial-icourse163/details




# 书籍推荐

Uda推荐的书籍
https://cse.sc.edu/~jokane/agitr/

https://blog.csdn.net/ZhangRelay/article/details/52244746



# 何时需要catkin_make 与 source .zshrc
- 当在某一个ROS包中创建了一个python节点时，不需要make，如果想使用tab命令补全，需要source .zshrc
- 当在某一个ROS包中创建了一个自定义msg之后，需要make生成该消息的源代码。

# 运用/spawn 服务生成一个新小海龟
```shell
$ rosservice type /spawn 

turtlesim/Spawn

$ rosservice info /spawn 

Node: /turtlesim
URI: rosrpc://Ubuntu16:60567
Type: turtlesim/Spawn
Args: x y theta name


$ rosservice call /spawn "x: 5.0
y: 5.0
theta: 0.0
name: 'turtle2'"
name: "turtle2"

```
