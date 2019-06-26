# ROS 学习网站

官方中文入门教程：
http://wiki.ros.org/cn/ROS/Tutorials

Turtlesim官方教程：
http://wiki.ros.org/turtlesim?distro=kinetic

Rospy官方教程：
http://wiki.ros.org/rospy_tutorials

CKZZ教程：
https://www.ncnynl.com/beginning.html

CKZZros-python教程：
https://www.ncnynl.com/category/ros-python/


# 何时需要catkin_make 与 source .zshrc
- 当在某一个ROS包中创建了一个python节点时，不需要make，如果想使用tab命令补全，需要source .zshrc
- 当在某一个ROS包中创建了一个自定义msg之后，需要make生成该消息的源代码。
