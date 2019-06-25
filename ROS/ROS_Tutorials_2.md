# 创建ROS消息和ROS服务

- 消息(msg): msg文件就是一个描述ROS中所使用消息类型的简单文本。它们会被用来生成不同语言的源代码。
- 服务(srv): 一个srv文件描述一项服务。它包含两个部分：请求和响应。
> msg文件存放在package的msg目录下，srv文件则存放在srv目录下。

msg文件实际上就是每行声明一个数据类型和变量名。可以使用的数据类型如下： 

- int8, int16, int32, int64 (plus uint*) 
- float32, float64 
- string 
- time, duration 
- other msg files 
- variable-length array[] and fixed-length array[C] 
> ROS中的float32和float64类型都对应到Python的float类型

在ROS中有一个特殊的数据类型：Header，它含有时间戳和坐标系信息。在msg文件的第一行经常可以看到Header header的声明.
下面是一个msg文件的样例，它使用了Header，string，和其他另外两个消息类型。

```
Header header
string child_frame_id
geometry_msgs/PoseWithCovariance pose
geometry_msgs/TwistWithCovariance twist
```

srv文件分为请求和响应两部分，由'---'分隔。下面是srv的一个样例：其中 A 和 B 是请求, 而Sum 是响应。 
```
int64 A
int64 B
---
int64 Sum
```
## 使用 msg
创建一个 msg：
```shell
# 在之前创建的package里定义新的消息。
$ cd ~/catkin_ws/src/beginner_tutorials
$ mkdir msg
$ echo "int64 num" > msg/Num.msg
```

> 上面是最简单的例子——在.msg文件中只有一行数据。当然，你可以仿造上面的形式多增加几行以得到更为复杂的消息： 
```shell
string first_name
string last_name
uint8 age
uint32 score
```

接下来，还有关键的一步：我们要确保msg文件被转换成为C++，Python和其他语言的源代码：

查看package.xml, 确保它包含一下两条语句: 
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
> 在构建的时候，我们只需要"message_generation"。
> 然而，在运行的时候，我们只需要"message_runtime"。

在编辑器中打开CMakeLists.txt文件。

在 CMakeLists.txt文件中，利用find_packag函数，增加对message_generation的依赖，这样就可以生成消息了。find_package()调用末尾添加的message_generation，这样catkin就知道去啦里找message_generation包：
```
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    rospy 
    std_msgs 
    message_generation # 添加这一行
)
```
> 有时候你会发现，即使你没有调用find_package,你也可以编译通过。这是因为catkin把你所有的package都整合在一起，因此，如果其他的package调用了find_package，你的package的依赖就会是同样的配置。但是，在你单独编译时，忘记调用find_package会很容易出错。


通过在add_message_files()调用的末尾添加消息定义文件来告知catkin我们想要编译他们：
```
add_message_files(
  FILES
  Num.msg
)
```

然后需要去掉generate_message()调用的注释，并已经包含了消息所依赖的所有依赖项：
```
generate_messages(
    DEPENDENCIES
    std_msgs
)
```

最后告知catkin我们将在运行时使用消息，即在catkin_package()调用末尾添加message_runtime:
```
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```

## 使用rosmsg

下面通过rosmsg show命令，检查ROS是否能够识消息。

使用方法：` rosmsg show [message type]`

```shell
$ rosmsg show beginner_tutorials/Num

int64 num
```
> beginner_tutorials -- 消息所在的package
> Num -- 消息名Num. 

如果忘记了消息所在的package，你也可以省略掉package名。输入：

```shell
$ rosmsg show Num

[beginner_tutorials/Num]:
int64 num
```

## 创建一个srv

在package中创建一个服务：
```shell
$ roscd beginner_tutorials
$ mkdir srv
```

这次我们不再手动创建服务，而是从其他的package中复制一个服务。 roscp是一个很实用的命令行工具，它实现了将文件从一个package复制到另外一个package的功能。

使用方法：`roscp [package_name] [file_to_copy_path] [copy_path]`

从rospy_tutorials package中复制一个服务文件：

```shell
$ roscp rospy_tutorials AddTwoInts.srv srv/AddTwoInts.srv
```

AddTwoInts.srv内容如下：
```
int64 a
int64 b
---
int64 sum
```
要确保srv文件被转换成C++，Python和其他语言的源代码,还有很关键的两步：

package.xml文件
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

在CMakeLists.txt文件中增加了对message_generation的依赖。

```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation)
```
> message_generation 对msg和srv都起作用

```
add_service_files(
  FILES
  AddTwoInts.srv
)
```
```
generate_messages(
    DEPENDENCIES
    std_msgs
)
```
> 部分修改与定义msg时相同

## 使用 rossrv

下面通过rosmsg show命令，检查ROS是否能够识该服务。

用法： `rossrv show [service type]`

```shell
rossrv show beginner_tutorials/AddTwoInts

int64 a
int64 b
---
int64 sum
```

可以不指定具体的package名来查找服务文件：
```shell
$ rossrv show AddTwoInts

[beginner_tutorials/AddTwoInts]:
int64 a
int64 b
---
int64 sum

[rospy_tutorials/AddTwoInts]:
int64 a
int64 b
---
int64 sum
```

## 创建 msg和srv都需要的步骤
**疑问？ 官方教程不是不没列全**
```
generate_messages(
  DEPENDENCIES
  std_msgs
)
```

## 创建 msg或 srv 之后make

由于增加了新的消息，所以我们需要重新编译我们的package：

```shell
$ cd ~/catkin_ws
$ catkin_make
```
> 所有在msg路径下的.msg文件都将转换为ROS所支持语言的源代码。生成的C++头文件将会放置在`~/catkin_ws/devel/include/beginner_tutorials/` 
Python脚本语言会在` ~/catkin_ws/devel/lib/python2.7/dist-packages/beginner_tutorials/msg` 目录下创建。

> 所有在srv路径下的.srv文件都将转换为ROS所支持语言的源代码。生成的C++头文件将会放置在`~/catkin_ws/devel/include/beginner_tutorials/ `
Python脚本语言会在 `~/catkin_ws/devel/lib/python2.7/dist-packages/beginner_tutorials/srv `目录下创建。


# 简单的消息发布器和订阅器
## Writing the Publisher Node

```shell
$ roscd beginner_tutorials
$ mkdir scripts
$ cd scripts
# 下载教程文件
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/talker.py
$ chmod +x talker.py
```
> python 源文件默认放到scripts文件夹

talker.py 文件内容：
```python
#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

代码解释：
```python
#!/usr/bin/env python
import rospy
```
每个Python ROS节点都会在顶部声明此声明。 第一行确保此脚本作为Python脚本执行。对于ROS节点，需要导入rospy
```python
from std_msgs.msg import String
```
std_msgs.msg导入是为了我们可以重用std_msgs / String消息类型（一个简单的字符串容器）进行发布。
- python包将在下面两个库中寻找`/home/likun/catkin_ws/devel/lib/python2.7/dist-packages:/opt/ros/kinetic/lib/python2.7/dist-packages`。 python环境变量可使用 `echo $PYTHONPATH` 查看。
- std_msgs包位于`/opt/ros/kinetic/lib/python2.7/dist-packages，std_msgs`子包为msg，子包msg内包含`_String.py`模块

```python
pub = rospy.Publisher('chatter', String, queue_size=10)
rospy.init_node('talker', anonymous=True)
```
这部分代码定义了talker与ROS中其余对象的接口。
- `pub = rospy.Publisher("chatter", String, queue_size=10)` 声明节点使用消息类型String发布到chatter话题。
    - 这里的字符串实际上是std_msgs.msg.String类。
    - queue_size参数在New ROS hydro中新添加的，如果任何subscriber没有足够快地接收消息，则限制排队消息的数量。queue_size参数用于确定在删除消息之前可以存储在发布者队列中的最大数量消息。
- `rospy.init_node(NAME, ...)` 声明发布‘chatter’topic的node的名称。 在这种情况下，节点将采用名称talker。 注意：名称必须是基本名称，即它不能包含任何斜杠“/”。
    - anonymous = True通过在NAME末尾添加随机数来确保这个节点具有唯一名称。
        -ros中，如果两个节点名字相同，则先启动的那个节点会被kill
    - 必须在调用任何其他rospy包函数之前调用init_node（）
> node的名字可以与python文件名不同，比如可以命名为节点为talker123。
> ROS发布可以是同步的也可以是异步的：同步发布意味着发布者将尝试发布到话题，但如果该话题由其他发布者发布，则可能会被阻止。 在这种情况下，第二个发布者被阻止，直到第一个发布者将所有消息序列化到缓冲区，并且缓冲区已将消息写入每个主题的订阅者。 rospy.Publisher默认使用同步发布，如果未使用queue_size参数或将其设置为None。异步发布意味着发布者可以将消息存储在队列中，直到可以发送消息。 如果发布的消息数超过队列大小，则删除最旧的消息。 可以使用queue_size参数设置队列大小。

```python
rate = rospy.Rate(10) # 10hz
```
此行创建Rate类的实例化对象rate。 借助对象rate的方法sleep（）可以提供需要的速率循环。参数为10，即每秒循环10次。

```python
while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()
```

这个循环是一个相当标准的rospy构造：检查rospy.is_shutdown（）标志然后执行循环。 is_shutdown（）用于判断程序是否应该退出（例如，如果有Ctrl-C或其他）。 在这种情况下，这个循环需要做的是对pub.publish（hello_str）的调用，它将字符串发布到chatter。 

循环调用rate.sleep（），使循环保持所需的速率，即上面的10Hz。

这个循环还调用rospy.loginfo（str），它执行三重任务：将消息打印到屏幕，将消息写入Node的日志文件，将消息写入rosout。

```python
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
`if __name__ == '__main__' `的意思是：当.py文件被直接运行时，`if __name__ == '__main__'`之下的代码块将被运行；当.py文件以模块形式被导入时，`if __name__ == '__main__'`之下的代码块不被运行。

除了标准的Python`` __main__``检查之外，这还会捕获一个rospy.ROSInterruptException异常，当按下Ctrl-C或者节点关闭时，rospy.sleep（）和rospy.Rate.sleep（）方法会抛出该异常。 这样做是为了防止在sleep（）之后意外地继续执行代码。

try-except 异常处理：

Python中检测处理异常是非常重要的，这可以增加代码的健壮性，异常可以通过 try 语句来检测. 任何在 try 语句块里的代码都会被监测, 检查有无异常发生。首先尝试执行 try 子句, 如果没有错误, 忽略所有的 except 从句继续执行，如果发生异常, 解释器将在这一串处理器(except 子句)中查找匹配的异常。

[常见ROS异常](http://wiki.ros.org/rospy/Overview/Exceptions)


## Writing the Subscriber Node

```shell
$ roscd beginner_tutorials/scripts/
$ wget https://raw.github.com/ros/ros_tutorials/kinetic-devel/rospy_tutorials/001_talker_listener/listener.py
$ chmod +x listener.py
```
listener.py内容：

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

代码解释：

listener.py的代码类似于talker.py，另外引入了一个新的基于回调的机制来订阅消息。

```python
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
```
这里的data.data = String.data， callback(data)中的data是接收到的String对象。

String消息的定义为：`string data` ，data.data中第一个data为ROS String对象，第二个data为ROS String对象的一个方法，并且名字要与`string data`中的data保持一致，用来获取msg数据。

```python
rospy.init_node('listener', anonymous=True)
rospy.Subscriber("chatter", String, callback)
```
这声明节点订阅了std_msgs.msgs.String类型的chatter主题。 收到新消息时，将调用回调，并将消息作为第一个参数。

我们也稍微改变了对rospy.init_node（）的调用。 我们添加了anonymous = True关键字参数。 ROS要求每个节点都有唯一的名称。 如果出现具有相同名称的节点，则会与第一个节点冲突。  anonymous = True标志告诉rospy为节点生成一个唯一的名称，以便可以轻松运行多个listener.py节点。

rospy.spin（）只是让保持运行节点，直到节点被关闭。 与roscpp不同，rospy.spin（）不会影响订阅者回调函数，因为它们有自己的线程。

## 测试消息发布器和订阅器

此处因为只使用了默认的msg和python API,所以catkin_make 不是必须的。
但是 需要`source .zshrc`
```shell
$ source .zshrc
```

```shell
# 新终端
$ roscore

# 新终端
$ rosrun beginner_tutorials talker.py

# 新终端
$ rosrun beginner_tutorials listener.py
```

![](./res/turtlesim6.png)


# 使用自己创建的消息msg Num.msg

Num.msg：
```
int64 num
```

## 创建Node 用来发布话题
```shell
$ roscd beginner_tutorials/scripts
$ touch talkernum.py
$ chmod +x talkernum.py
```

talkernum.py 内容：
```python
#!/usr/bin/env python
import rospy
from beginner_tutorials.msg import Num

def talker():
    pub = rospy.Publisher('chatternum', Num, queue_size=10)
    rospy.init_node('talkernum', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 0
    while not rospy.is_shutdown():
        i = i + 1
        numi = i
        rospy.loginfo(numi)
        pub.publish(numi)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```
> `from beginner_tutorials.msg import Num` 从beginner_tutorials包中的子包msg中导入Num模块。

> `rospy.loginfo(numi)` `pub.publish(numi)` 也可以使用类的示例化调用。

Num.msg的另一用用法：
```python
#!/usr/bin/env python
import rospy
from beginner_tutorials.msg import Num

def talker():
    pub = rospy.Publisher('chatternum', Num, queue_size=10)
    rospy.init_node('talkernum', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    i = 0
    while not rospy.is_shutdown():
        i = i + 1
        msg = Num()
        msg.num = i
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
```

## 创建Node 用来接受话题

```shell
$ roscd beginner_tutorials/scripts
$ touch listenernum.py
$ chmod +x listenernum.py
```

listenernum.py内容：

```python
#!/usr/bin/env python
import rospy
from beginner_tutorials.msg import Num

def callback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %d", msg.num)
    
def listener():
    rospy.init_node('listenernum', anonymous=True)
    rospy.Subscriber("chatternum", Num, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
```

## 测试消息发布器和订阅器

```shell
# 新终端
$ roscore

# 新终端
$ rosrun beginner_tutorials talkernum.py

# 新终端
$ rosrun beginner_tutorials listenernum.py
```

