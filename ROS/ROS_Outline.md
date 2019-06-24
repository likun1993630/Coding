
# 运行官方talker和listener
  无自定义消息类型
```shell
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src

# init 这一步可以省略，因为这一步会在src目录生成一个CMakeList.txt文件，并链接到ros。Catkin_make的时候也会自动链接。
$ catkin_init_workspace

$ cd ~/catkin_ws
$ catkin_make

$ source devel/setup.zsh

# 创建package，并声明依赖
$ cd ~/catkin_ws/src
$ catkin_create_pkg beginner_tutorials std_msgs rospy roscpp

$ cd ~/catkin_ws/
$ catkin_make

$ roscd beginner_tutorials
$ mkdir scripts
$ cd scripts
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
$ chmod +x talker.py

$ roscd beginner_tutorials/scripts/
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
$ chmod +x listener.py

$ cd ~/catkin_ws
$ catkin_make

$ roscore
$ rosrun beginner_tutorials talker.py 
$ rosrun beginner_tutorials listener.py
```

