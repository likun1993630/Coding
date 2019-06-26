## Arm Mover

GoToPosition.srv：
```
float64 joint_1
float64 joint_2
---
duration time_elapsed
```

```python
#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from simple_arm.srv import *
```
JointState消息发布到/ simple_arm / joint_states主题，用于监视arm的位置。

> ```
> $ rostopic info /simple_arm/joint_states 
> Type: sensor_msgs/JointState
> Publishers: 
> * /gazebo (http://Ubuntu16:45239/)
> Subscribers: 
> * /robot_state_publisher (http://Ubuntu16:37795/)
> ```
> ```
> $ rosmsg show sensor_msgs/JointState
> std_msgs/Header header
> uint32 seq
> time stamp
> string frame_id
> string[] name
> float64[] position
> float64[] velocity
> float64[] effort
> ```

simple_arm的python包，包括其中的srv模块已经被catkin在biuld的过程中自动生成了。

```python
def at_goal(pos_j1, goal_j1, pos_j2, goal_j2):
    tolerance = .05
    result = abs(pos_j1 - goal_j1) <= abs(tolerance)
    result = result and abs(pos_j2 - goal_j2) <= abs(tolerance)
    return result
```
如果关节位置接近目标，则此函数返回True。 在现实世界中从传感器进行测量时，总会有一些噪音。 gazebo simulator报告的关节位置也是带有误差的。如果两个关节位置都在目标的.05弧度范围内，则返回True。

```python
def clamp_at_boundaries(requested_j1, requested_j2):
    clamped_j1 = requested_j1
    clamped_j2 = requested_j2
    min_j1 = rospy.get_param('~min_joint_1_angle', 0)
    max_j1 = rospy.get_param('~max_joint_1_angle', 2*math.pi)
    min_j2 = rospy.get_param('~min_joint_2_angle', 0)
    max_j2 = rospy.get_param('~max_joint_2_angle', 2*math.pi)
```
clamp_at_boundaries（）负责强制使每个关节始终处于v最小和最大关节角度之间。 如果传入的关节角度在可操作范围之外，它们将被“缩小”到最接近的允许值。

每次调用clamp_at_boundaries（）时，都会从参数服务器检索最小和最大关节角度。 “〜”是私有名称空间限定符，表示我们希望得到的参数在此节点的私有名称空间/ arm_mover /中（例如~min_joint_1_angle解析为/ arm_mover / min_joint_1_angle）。 在rospy.get_param（）无法从param服务器获取参数的情况下，第二个参数是要返回的默认值。rospy.get_param（）调用时，如果参数服务器没有定义相应的参数，那么给定默认值就会以此默认值在参数服务器初始化这个参数。

[命名空间Doc](http://wiki.ros.org/Names#Resolving)

```python
    if not min_j1 <= requested_j1 <= max_j1:
        clamped_j1 = min(max(requested_j1, min_j1), max_j1)
        rospy.logwarn('j1 is out of bounds, valid range (%s,%s), clamping to: %s',
                      min_j1, max_j1, clamped_j1)

    if not min_j2 <= requested_j2 <= max_j2:
        clamped_j2 = min(max(requested_j2, min_j2), max_j2)
        rospy.logwarn('j2 is out of bounds, valid range (%s,%s), clamping to: %s',
                      min_j2, max_j2, clamped_j2)

    return clamped_j1, clamped_j2
```
如有必要，此功能的其余部分可简单地夹紧关节角度。 

如果请求的关节角度超出界限，则会记录警告消息。

[rospy.logwarn](http://wiki.ros.org/rospy_tutorials/Tutorials/Logging)完成三个事：
- stderr(系统标准错误)
- Node's log file（写入Node日志）
- /rosout Topic （pub到rosout 话题）

```python
def move_arm(pos_j1, pos_j2):
    time_elapsed = rospy.Time.now()
    j1_publisher.publish(pos_j1)
    j2_publisher.publish(pos_j2)

    while True:
        joint_state = rospy.wait_for_message('/simple_arm/joint_states', JointState)
        if at_goal(joint_state.position[0], pos_j1, joint_state.position[1], pos_j2):
            time_elapsed = joint_state.header.stamp - time_elapsed
            break

    return time_elapsed
```
move_arm（）命令手臂移动，返回手臂移动时经过的时间。

在函数中，我们使用rospy.wait_for_message（）调用从/ simple_arm / joint_states话题接收JointState消息。 这是阻塞函数调用，这意味着在/ simple_arm / joint_states主话题上收到消息之前它不会返回。

通常，您不应该使用wait_for_message（）。 为了清楚起见，我们在这里简单地使用它，因为move_arm是从handle_safe_move_request（）函数调用的，它要求将响应消息作为返回参数传回。

```python
def handle_safe_move_request(req):
    rospy.loginfo('GoToPositionRequest Received - j1:%s, j2:%s',
                   req.joint_1, req.joint_2)
    clamp_j1, clamp_j2 = clamp_at_boundaries(req.joint_1, req.joint_2)
    time_elapsed = move_arm(clamp_j1, clamp_j2)

    return GoToPositionResponse(time_elapsed)
```
这是服务处理函数。 当服务客户端向safe_move服务发送GoToPosition请求消息时，将调用此函数。 函数参数req的类型为GoToPositionRequest。 服务响应的类型为GoToPositionResponse。

这是服务处理程序函数，只要收到新的服务请求就会调用它。 从该函数返回对服务请求的响应。

rospy.loginfo
- stdout(标准输出)
- Node's log file（写入Node日志）
- /rosout Topic （pub到rosout 话题）

move_arm（）是阻塞的，并且在手臂完成移动之前不会返回。 无法处理传入的消息，并且当arm执行它的移动命令时，不能在python脚本中完成其他有用的工作。 虽然这对于这个例子没有任何实际问题，但通常应该避免这种做法。 避免阻塞执行线程的一个好方法是使用Action。 这里有一些信息性文档描述了何时最好使用主题与服务，而不是动作。[什么时候使用话题，服务，动作](http://wiki.ros.org/ROS/Patterns/Communication#Communication_via_Topics_vs_Services_vs_X)

```python
def mover_service():
    rospy.init_node('arm_mover')
    service = rospy.Service('~safe_move', GoToPosition, handle_safe_move_request)
    rospy.spin()
```
这里使用名称“arm_mover”初始化节点，并使用名称“safe_move”创建GoToPosition服务。 如前所述，“〜”限定符标识safe_move意味着属于此节点的私有名称空间。 生成的服务名称为/ arm_mover / safe_move。 rospy.Service（）调用的第三个参数是在收到服务请求时应该调用的函数,并将请求参数也传递给回调函数。 最后，rospy.spin（）简单地阻塞，直到节点收到关闭请求（比如ctrl+c）。 如果不包含此行，将导致mover_service（）完成一次调用并且返回请求响应，脚本将完成执行。

```python
if __name__ == '__main__':
    j1_publisher = rospy.Publisher('/simple_arm/joint_1_position_controller/command', Float64, queue_size=10)
    j2_publisher = rospy.Publisher('/simple_arm/joint_2_position_controller/command', Float64, queue_size=10)

    try:
        mover_service()
    except rospy.ROSInterruptException:
        pass
 ```
arm_mover：

```python
#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from simple_arm.srv import *

def at_goal(pos_j1, goal_j1, pos_j2, goal_j2):
    tolerance = .05
    result = abs(pos_j1 - goal_j1) <= abs(tolerance)
    result = result and abs(pos_j2 - goal_j2) <= abs(tolerance)
    return result

def clamp_at_boundaries(requested_j1, requested_j2):
    clamped_j1 = requested_j1
    clamped_j2 = requested_j2

    min_j1 = rospy.get_param('~min_joint_1_angle', 0)
    max_j1 = rospy.get_param('~max_joint_1_angle', 2*math.pi)
    min_j2 = rospy.get_param('~min_joint_2_angle', 0)
    max_j2 = rospy.get_param('~max_joint_2_angle', 2*math.pi)

    if not min_j1 <= requested_j1 <= max_j1:
        clamped_j1 = min(max(requested_j1, min_j1), max_j1)
        rospy.logwarn('j1 is out of bounds, valid range (%s,%s), clamping to: %s',
                      min_j1, max_j1, clamped_j1)

    if not min_j2 <= requested_j2 <= max_j2:
        clamped_j2 = min(max(requested_j2, min_j2), max_j2)
        rospy.logwarn('j2 is out of bounds, valid range (%s,%s), clamping to: %s',
                      min_j2, max_j2, clamped_j2)

    return clamped_j1, clamped_j2

def move_arm(pos_j1, pos_j2):
    time_elapsed = rospy.Time.now()
    j1_publisher.publish(pos_j1)
    j2_publisher.publish(pos_j2)

    while True:
        joint_state = rospy.wait_for_message('/simple_arm/joint_states', JointState)
        if at_goal(joint_state.position[0], pos_j1, joint_state.position[1], pos_j2):
            time_elapsed = joint_state.header.stamp - time_elapsed
            break

    return time_elapsed

def handle_safe_move_request(req):
    rospy.loginfo('GoToPositionRequest Received - j1:%s, j2:%s',
                   req.joint_1, req.joint_2)
    clamp_j1, clamp_j2 = clamp_at_boundaries(req.joint_1, req.joint_2)
    time_elapsed = move_arm(clamp_j1, clamp_j2)

    return GoToPositionResponse(time_elapsed)

def mover_service():
    rospy.init_node('arm_mover')
    service = rospy.Service('~safe_move', GoToPosition, handle_safe_move_request)
    rospy.spin()

if __name__ == '__main__':
    j1_publisher = rospy.Publisher('/simple_arm/joint_1_position_controller/command',
                                   Float64, queue_size=10)
    j2_publisher = rospy.Publisher('/simple_arm/joint_2_position_controller/command',
                                   Float64, queue_size=10)

    try:
        mover_service()
    except rospy.ROSInterruptException:
        pass
```

## Arm Mover: Launch and Interact
测试服务

编辑之前的launch.xml 文件：

```shell
$ cd ~/catkin_ws/src/simple_arm/launch
$ vim launch.xml
```

添加：
```
  <!-- The arm mover node -->
  <node name="arm_mover" type="arm_mover" pkg="simple_arm">
    <rosparam>
      min_joint_1_angle: 0
      max_joint_1_angle: 1.57
      min_joint_2_angle: 0
      max_joint_2_angle: 1.0 #为了方便后面测试rosparam set命令
    </rosparam>
  </node>
```
运行：
```
# 启动所有相关节点
$ roslaunch simple_arm robot_spawn.launch

# 初步查看是否正确
$ rosnode list
$ rosservice list

# 查看摄像机图像流，可以使用命令rqt_image_view
$ rqt_image_view /rgb_camera/image_raw

# 由于在launch文件里初始的值为1，而最终需要1.57
$ rosparam set /arm_mover/max_joint_2_angle 1.57

# 命令行调用服务：
$ rosservice call /arm_mover/safe_move "joint_1: 1.57
joint_2: 1.57"
# 最后就可以看到正确的相机视角了


