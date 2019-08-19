通过turtlesim来演示

```
roscore
rosrun turtlesim turtlesim_node 
rosrun turtlesim turtle_teleop_key
```

## 通过bag文件记录话题消息

当发布话题的节点运行后，可以通过`bag`命令记录所有被发布的话题：

```shell
mkdir ~/bagfiles # 建立临时的目录
cd ~/bagfiles
rosbag record -a #-a选项表示将当前发布的所有话题数据都录制保存到一个bag文件中。 
```

> 注意可以通过`rostopic list -v`查看所有话题的详细一些的信息

当消息记录完成后，结束ctrl+c终止record的命令行，在新建的bagfile文件中会生成`年-月-日-时-分-秒.bag`文件。

## 数据重现

我们可以使用**rosbag info**检查看它的内容，使用**rosbag play**命令回放出来

使用 rosbag info 查看文件：

```shell
cd ~/bagfiles
rosbag info <bagfile_name>
```

```shell
# 结果如下：
path:        2019-08-19-23-38-56.bag
version:     2.0
duration:    36.8s
start:       Aug 19 2019 23:38:56.92 (1566250736.92)
end:         Aug 19 2019 23:39:33.76 (1566250773.76)
size:        403.3 KB
messages:    4865
compression: none [1/1 chunks]
types:       geometry_msgs/Twist [9f195f881246fdfa2798d1d3eebca84a]
             rosgraph_msgs/Log   [acffd30cd6b6de30f120938c17c593fb]
             turtlesim/Color     [353891e354491c51aabe32df673fb446]
             turtlesim/Pose      [863b248d5016ca62ea2e895ae5265cf9]
topics:      /rosout                  130 msgs    : rosgraph_msgs/Log   (2 connections)
             /rosout_agg              126 msgs    : rosgraph_msgs/Log  
             /turtle1/cmd_vel          33 msgs    : geometry_msgs/Twist
             /turtle1/color_sensor   2288 msgs    : turtlesim/Color    
             /turtle1/pose           2288 msgs    : turtlesim/Pose
```



使用rosbag play 回放话题：

使用rosbag play 回放话题时就是将原来所有被发布的话题且被记录的都重新发布出来。

比如rosbag记录了turtlesim键盘控制节点发布的话题，使用rosbag play 回放时的play_xxxx。 

![1566251502339](res/1566251502339.png)

```shell
cd ~/bagfiles
rosbag play <bagfile_name>
```

```shell
# 结果：
[ INFO] [1566251281.230071919]: Opening 2019-08-19-23-38-56.bag

Waiting 0.2 seconds after advertising topics... done.

Hit space to toggle paused, or 's' to step.
 [RUNNING]  Bag Time: 1566250736.916518   Duration: 0.000000 / 36.845503         
 [RUNNING]  Bag Time: 1566250736.917423   Duration: 0.000904 / 36.845503         
 [RUNNING]  Bag Time: 1566250737.017535   Duration: 0.101017 / 36.845503         
 [RUNNING]  Bag Time: 1566250737.117672   Duration: 0.201154 / 36.845503         
 [RUNNING]  Bag Time: 1566250737.149518   Duration: 0.233000 / 36.845503 
 ......
```

默认模式下，**rosbag play**命令在公告每条消息后会等待一小段时间（0.2秒）后才真正开始发布bag文件中的内容。等待一段时间的过程可以通知消息订阅器消息已经公告了消息数据可能会马上到来。如果**rosbag play**在公告消息后立即发布，订阅器可能会接收不到几条最先发布的消息。等待时间可以通过-d选项来指定。 

最终/turtle1/command_velocity话题将会被发布，同时在turtuelsim虚拟画面中turtle应该会像之前你通过turtle_teleop_key节点控制它一样开始移动。从运行**rosbag play**到turtle开始移动时所经历时间应该近似等于之前在本教程开始部分运行**rosbag record**后到开始按下键盘发出控制命令时所经历时间。你可以通过-s参数选项让**rosbag play**命令等待一段时间跳过bag文件初始部分后再真正开始回放。另外参数选项-r，它允许你通过设定一个参数来改变消息发布速率。

```shell
rosbag play -r 2 <bagfile_name>   #以两倍的速度发布记录的消息
```

可以单独回放某一个话题：

```
rosbag play <bagfile_name> --topics /topic_name
```

## 记录选定话题

**rosbag record**命令支持只录制某些特别指定的话题到单个bag文件中，这样就允许用户只录制他们感兴趣的话题。 

```
rosbag record -O subset /turtle1/command_velocity /turtle1/pose
```

- 上述命令中的-O参数告诉**rosbag record**将数据记录保存到名为subset.bag的文件中，同时后面的话题参数告诉**rosbag record**只能录制这两个指定的话题

## rosbag record/play 命令的局限性

`rosbag record/play`无法完美重现系统的状态。



## .bag文件转.txt

将`file_name.bag`文件中`topic_name`话题的消息转换到`Txt_name.txt`文件中：

```shell
rostopic echo -b file_name.bag -p /topic_name > Txt_name.txt
```



参考：

http://wiki.ros.org/cn/ROS/Tutorials/Recording%20and%20playing%20back%20data

https://blog.csdn.net/Tansir94/article/details/81513517

