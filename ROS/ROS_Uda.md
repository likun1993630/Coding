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

rospy.logwarn


