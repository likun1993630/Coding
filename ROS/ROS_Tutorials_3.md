# 简单的Service和Client

- Defining services
  - ROS服务允许节点之间存在请求/响应通信。 在提供服务的节点内，请求消息由函数或方法处理。 成功处理请求后，提供服务的节点会将消息发送回请求者节点。 在Python中，可以使用以下定义格式创建ROS服务：`service = rospy.Service('service_name', serviceClassName, handler)`
  - service_name是为服务指定的名称。 其他节点将使用此名称来指定他们向其发送请求的服务。
  - The serviceClassName comes from the file name where the service definition exists.  （即.srv的文件名）每个服务都在.srv文件中被定义; 这是一个文本文件，为请求和响应提供正确的消息类型。
  - handler是处理传入服务消息的函数或方法的名称。 每次调用服务时都会调用此函数，并且来自服务调用的消息将作为参数传递给处理程序。 处理程序应返回适当的服务响应消息。

- Using Services
  - 可以直接从命令行调用服务`rosservice call /serviceName [para] `
  - 另外，要在另一个节点内使用ROS服务，要定义一个ServiceProxy，它提供了向服务发送消息的接口：`service_proxy = rospy.ServiceProxy('service_name', serviceClassName)`
  - 通过调用serviceClassNameRequest（）方法创建新的服务消息：
  ```
  msg = serviceClassNameRequest()
  #update msg attributes here to have correct data
  response = service_proxy(msg)
  ```
  > 第三种方法适用范围相对比第二种小，不推荐（我自己主观不推荐）

[Rospy_services Doc](http://wiki.ros.org/rospy/Overview/Services)
 
## Writing a Service Node

AddTwoInts.srv：
```
int64 a
int64 b
---
int64 sum
```

创建服务（“add_two_ints_server”）节点，该节点将接收两个整数并返回总和。

```
$ roscd beginner_tutorials/scripts
$ touch add_two_ints_server.py
$ chmod +x add_two_ints_server.py
```

add_two_ints_server.py文件:
```python
#!/usr/bin/env python
from beginner_tutorials.srv import *
import rospy

def handle_add_two_ints(req):

    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)


def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
```
代码解析：
```python
#!/usr/bin/env python
from beginner_tutorials.srv import *
#自动检索pythonpath中的所有python包，并且最后找到beginner_tutorials python包
#此处即PYTHONPATH=/home/likun/catkin_ws/devel/lib/python2.7/dist-packages
#beginner_tutorials.srv包 中包含_AddTwoInts.py文件，是由AddTwoInts.srv文件build而来，里面定义了服务AddTwoInts的内容。
#_AddTwoInts.py 中定义了三个class，AddTwoIntsRequest，AddTwoIntsResponse，AddTwoInts。
import rospy

def handle_add_two_ints(req):
	# req 即为int64 a 和 int64 b
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    # 打印a b 与 a+b
    return AddTwoIntsResponse(req.a + req.b)
    #在AddTwoInts.srv被编译后，生成的.py文件中，AddTwoIntsResponse是作为一个class存在的，
    #作用就是将req.a + req.b的结果重新转换为AddTwoIntsResponse实例，也就是AddTwoInts.srv中定义的Response类型(---下半部分)，即int64 sum。AddTwoInt.srv文件在被build时，会在动在后面加Response，也就是 --.srv >> --Response()，所以这里才能用类AddTwoIntsResponse进行实例化。

def add_two_ints_server():
    #使用init_node（）声明节点
    rospy.init_node('add_two_ints_server')
    # 声明服务
    # 使用AddTwoInts服务类型声明一个名为add_two_ints的新服务。 所有请求都传递给handle_add_two_ints函数。 
    # handle_add_two_ints使用AddTwoIntsRequest实例调用，并返回AddTwoIntsResponse的实例。
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    #service = rospy.Service('service_name', serviceClassName, handler)
    #service_name 为服务名
    #serviceClassName 为定义对应的 服务.srv 文件名
    #handler 为处理函数，处理函数的传入参数为服务的请求，请求的数据即为在AddTwoInts.srv中定义的 int64 a 与 int64 b,参数a和b将传入handle_add_two_ints形参req
    print "Ready to add two ints."
    #rospy.spin（）使代码不会退出，直到服务被关闭
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
```

## Writing the Client Node

创建客户端：
```shell
$ roscd beginner_tutorials/scripts
$ touch add_two_ints_client.py
$ chmod +x add_two_ints_client.py
```

add_two_ints_client.py 内容：

```python
#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')
    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
```

代码解释：

```python
rospy.wait_for_service('add_two_ints')
```
使程序等待，直到名为add_two_ints 的服务出现，这是比较常用的方法。

```python
add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
```
通过rospy.ServiceProxy接口提供了调用服务的通用方法，返回的add_two_ints是一个函数对象。

补充：
sys.argv：获取运行 Python 程序的命令行参数。argv为列表。
 - sys.argv[0] 通常就是指该 Python 程序，即文件名。
 - sys.argv[1] 代表为 Python 程序提供的第一个参数，
 - sys.argv[2] 代表为 Python 程序提供的第二个参数……依此类推
 
client最简形式：

```python
#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_ints_client(x, y):
	rospy.wait_for_service('add_two_ints')
	add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
	resp1 = add_two_ints(x, y)
	return resp1.sum
	
if __name__ == "__main__":
	x = int(sys.argv[1])
	y = int(sys.argv[2])
	print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
```

## 测试Service和Client

可以通过命令行直接调用服务：

```shell
# 新终端
$ roscore

# 新终端
$ rosrun beginner_tutorials add_two_ints_server.py 
```

```shell
$ rosservice call /add_two_ints "a: 2
b: 2"


sum: 4
```

使用client调用Service：

```shell
# 新终端
$ roscore

# 新终端
$ rosrun beginner_tutorials add_two_ints_server.py 

# 新终端
$ rosrun beginner_tutorials add_two_ints_client.py 1 3  (Python)

#结果：
request: x=1, y=3
sending back response: [4]
```
