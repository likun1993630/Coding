# 使用参数

## 参数类型
- 可使用整数，浮点数，字符串，布尔值，列表，字典等作为参数
- 字典有额外的意义，可当作命名空间使用。可以这样设置:
```
/gains/P = 1.0
/gains/I = 2.0
/gains/D = 3.0
```
- 在rospy,可以获取输出/gains
```
{'P': 1.0, 'I': 2.0, 'D': 3.0}
```

## 操作参数
> 在python脚本中

- 获取参数，使用` rospy.get_param(param_name)`：

```
# 获取全局参数
rospy.get_param('/global_param_name')

# 获取目前命名空间的参数
rospy.get_param('param_name')

# 获取私有命名空间参数
rospy.get_param('~private_param_name')
# 获取参数，如果没，使用默认值
rospy.get_param('foo', 'default_value')
```

- 设置参数，使用`rospy.set_param(param_name, param_value)`：
```
rospy.set_param('some_numbers', [1., 2., 3., 4.])
rospy.set_param('truth', True)
rospy.set_param('~private_bar', 1+2)
```

- 删除参数，使用`rospy.delete_param('param_name')`：
```
rospy.delete_param('to_delete')
```

- 判断参数是否存在，使用`rospy.has_param('param_name')`：
```
if rospy.has_param('to_delete'):
    rospy.delete_param('to_delete')
```

## 解释参数名
- 在ROS,名称可以映射成不同名，你的节点也可以放入到命名空间。
- rospy一般都能自动解释这些名称。
- 获取实际的名称， `rospy.resolve_name(name)`：
```
value = rospy.get_param('~foo')
rospy.loginfo('Parameter %s has value %s', rospy.resolve_name('~foo'), value)
```

## 搜索参数

- 如果你不知道命名空间，你可以搜索参数。
- 搜索由私有命名空间开始，向上到全局命名空间。
- 使用`rospy.search_param(param_name)`：
```
full_param_name = rospy.search_param('foo')
param_value = rospy.get_param(full_param_name)
```
- 获取参数名后，也可以进行参数操作。

## 制作launch文件

代码:
```
https://github.com/ros/ros_tutorials/blob/kinetic-devel/rospy_tutorials/006_parameters
```
通过launch设置参数，进入launch目录，新建param_talker.launch
```shell
$ roscd beginner_tutorials/launch
$ touch param_talker.launch
$ chmod u+x param_talker.launch
```
输入内容：
```
<launch>

  <!-- set /global_example parameter -->
  <param name="global_example" value="global value" />
    
  <group ns="foo">

    <!-- set /foo/utterance -->
    <param name="utterance" value="Hello World" />

    <param name="to_delete" value="Delete Me" />

    <!-- a group of parameters that we will fetch together -->
    <group ns="gains">
      <param name="P" value="1.0" />
      <param name="I" value="2.0" />
      <param name="D" value="3.0" />      
    </group>
  
    <node pkg="rospy_tutorials" name="param_talker" type="param_talker.py" output="screen">
    
      <!-- set /foo/utterance/param_talker/topic_name -->
      <param name="topic_name" value="chatter" />
      
    </node>
    
  </group>
  
</launch>
```

## 制作节点
使用参数，进入scripts目录，新建param_talker.py
```shell
$ roscd beginner_tutorials/scripts
$ touch param_talker.py
$ chmod +x param_talker.py
```

输入代码：
```python
#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def param_talker():
    rospy.init_node('param_talker')

    # Fetch values from the Parameter Server. In this example, we fetch
    # parameters from three different namespaces:
    #
    # 1) global (/global_example)
    # 2) parent (/foo/utterance)
    # 3) private (/foo/param_talker/topic_name)

    # fetch a /global parameter
    global_example = rospy.get_param("/global_example")
    rospy.loginfo("%s is %s", rospy.resolve_name('/global_example'), global_example)
    
    # fetch the utterance parameter from our parent namespace
    utterance = rospy.get_param('utterance')
    rospy.loginfo("%s is %s", rospy.resolve_name('utterance'), utterance)
    
    # fetch topic_name from the ~private namespace
    topic_name = rospy.get_param('~topic_name')
    rospy.loginfo("%s is %s", rospy.resolve_name('~topic_name'), topic_name)

    # fetch a parameter, using 'default_value' if it doesn't exist
    default_param = rospy.get_param('default_param', 'default_value')
    rospy.loginfo('%s is %s', rospy.resolve_name('default_param'), default_param)
    
    # fetch a group (dictionary) of parameters
    gains = rospy.get_param('gains')
    p, i, d = gains['P'], gains['I'], gains['D']
    rospy.loginfo("gains are %s, %s, %s", p, i, d)    

    # set some parameters
    rospy.loginfo('setting parameters...')
    rospy.set_param('list_of_floats', [1., 2., 3., 4.])
    rospy.set_param('bool_True', True)
    rospy.set_param('~private_bar', 1+2)
    rospy.set_param('to_delete', 'baz')
    rospy.loginfo('...parameters have been set')

    # delete a parameter
    if rospy.has_param('to_delete'):
        rospy.delete_param('to_delete')
        rospy.loginfo("deleted %s parameter"%rospy.resolve_name('to_delete'))
    else:
        rospy.loginfo('parameter %s was already deleted'%rospy.resolve_name('to_delete'))

    # search for a parameter
    param_name = rospy.search_param('global_example')
    rospy.loginfo('found global_example parameter under key: %s'%param_name)
    
    # publish the value of utterance repeatedly
    pub = rospy.Publisher(topic_name, String, queue_size=10)
    while not rospy.is_shutdown():
        pub.publish(utterance)
        rospy.loginfo(utterance)
        rospy.sleep(1)
        
if __name__ == '__main__':
    try:
        param_talker()
    except rospy.ROSInterruptException: pass
```

## 整合测试

启动launch文件
```shell
$ roslaunch beginner_tutorials param_talker.launch
```

结果：
```
SUMMARY
========

PARAMETERS
 * /foo/gains/D: 3.0
 * /foo/gains/I: 2.0
 * /foo/gains/P: 1.0
 * /foo/param_talker/topic_name: chatter
 * /foo/to_delete: Delete Me
 * /foo/utterance: Hello World
 * /global_example: global name
 * /rosdistro: indigo
 * /rosversion: 1.11.16

NODES
  /foo/
    param_talker (beginner_tutorials/param_talker.py)

ROS_MASTER_URI=http://192.168.0.88:11311

core service [/rosout] found
process[foo/param_talker-1]: started with pid [32684]
[INFO] [WallTime: 1478587334.669759] /global_example is global name
[INFO] [WallTime: 1478587334.682391] /foo/utterance is Hello World
[INFO] [WallTime: 1478587334.693382] /foo/param_talker/topic_name is chatter
[INFO] [WallTime: 1478587334.708533] /foo/default_param is default_value
[INFO] [WallTime: 1478587334.729605] gains are 1.0, 2.0, 3.0
[INFO] [WallTime: 1478587334.731529] setting parameters...
[INFO] [WallTime: 1478587334.831270] ...parameters have been set
[INFO] [WallTime: 1478587334.871695] deleted /foo/to_delete parameter
[INFO] [WallTime: 1478587334.895860] found global_example parameter under key: /global_example
[INFO] [WallTime: 1478587334.922108] Hello World
[INFO] [WallTime: 1478587335.925285] Hello World
```
