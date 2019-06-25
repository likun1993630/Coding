```python
#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from simple_arm.srv import *
```
JointState消息发布到/ simple_arm / joint_states主题，用于监视arm的位置。
```
$ rostopic info /simple_arm/joint_states 
Type: sensor_msgs/JointState

Publishers: 
* /gazebo (http://Ubuntu16:45239/)

Subscribers: 
* /robot_state_publisher (http://Ubuntu16:37795/)
```
```
$ rosmsg show sensor_msgs/JointState

std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
string[] name
float64[] position
float64[] velocity
float64[] effort
```




