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

# ROS 工作空间覆盖:
ROS 工作空间的Overlaying机制，即工作空间的覆盖。
```shell
$ env | grep ros
# 或者
$ echo $ROS_PACKAGE_PATH

/home/likun/tutorial_ws/src:/home/likun/catkin_ws/src:/opt/ros/kinetic/share

$ rospack find ...
```
- 工作空间的路径的依次在$ROS_PACKAGE_PATH环境变量中记录
- 新设置的路径在$ROS_PACKAGE_PATH中会自动放置在最前端
- 运行时，ROS会优先查找最前端的工作空间中是否存在指定的功能包
- 如果不存在，就顺序向后查找。



