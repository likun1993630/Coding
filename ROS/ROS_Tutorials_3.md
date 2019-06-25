# 简单的Service和Client

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
from beginner_tutorials.srv import *
#自动检索pythonpath中的所有python包，
#此处即PYTHONPATH=/home/likun/catkin_ws/devel/lib/python2.7/dist-packages
#beginner_tutorials.srv包 中包含_AddTwoInts.py文件，是由AddTwoInts.srv文件build而来，里面定义了服务AddTwoInts的内容，AddTwoIntsResponse 即为其中定义的class
import rospy

def handle_add_two_ints(req):
	# req 即为int64 a 和 int64 b
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    # 打印a b 与 a+b
    return AddTwoIntsResponse(req.a + req.b)
    #在AddTwoInts.srv被编译后，生成的.py文件中，AddTwoIntsResponse是作为一个class存在的，
    #作用就是将req.a + req.b的结果重新转换为AddTwoIntsResponse实例，也就是AddTwoInts.srv中定义的Response类型(---下半部分)，即int64 sum。AddTwoInt.srv文件在被build时，会在动在后面加Response，也就是 --.srv >> --Response()，所以这里才能用类AddTwoIntsResponse进行实例化。

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    #初始化服务器节点的名字
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    #service = rospy.Service('service_name', serviceClassName, handler)
    #service_name 为服务名
    #serviceClassName 为定义对应的 服务.srv 文件名
    #handler 为处理函数，处理函数的传入参数为服务的请求，请求的数据即为在AddTwoInts.srv中定义的 int64 a 与 int64 b,参数a和b将传入handle_add_two_ints形参req
    print "Ready to add two ints."
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()

```
