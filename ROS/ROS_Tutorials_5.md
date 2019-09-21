# 编写简单的服务器和客户端 (C++)

使用自定义的服务文件AddTwoInts.srv

包目录结构

```
beginner_tutorialscppsrv
├── CMakeLists.txt
├── package.xml
├── src
│   ├── add_two_ints_client.cpp
│   └── add_two_ints_server.cpp
└── srv
    └── AddTwoInts.srv
```

AddTwoInts.srv

```
int64 a
int64 b
---
int64 sum
```

如果使用catkin_make编译并生成AddTwoInts.h头文件之后，可以这样使用生成的类

```cpp
#include "beginner_tutorialscppsrv/AddTwoInts.h"
...
beginner_tutorialscppsrv::AddTwoInts ati;  //ati分为ati.request和ati.response两部分
ati.request.a = 10; //不能用ati.name或者ati.age来访问  
ati.request.b = 20;
...
```

服务文件生成的类可以和如下结构体形式类比：

```cpp
struct AddTwoInts
{
    struct Request
    {
        int a;
        int b;
    }request;
    struct Response
    {
        int sum;
    }response;
}
```

### 编写Service节点

add_two_ints_server.cpp

```cpp
#include "ros/ros.h"
#include "beginner_tutorialscppsrv/AddTwoInts.h"

bool add(beginner_tutorialscppsrv::AddTwoInts::Request  &req,
         beginner_tutorialscppsrv::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
```

#### 代码解释

```cpp
#include "ros/ros.h"
#include "beginner_tutorialscppsrv/AddTwoInts.h"
// beginner_tutorialscppsrv/AddTwoInts.h是由编译系统，
// 自动根据我们先前创建的AddTwoInts.srv文件生成的对应该srv文件的头文件。 

/*
函数的输入参数就是AddTwoInts的Request和Response两部分，而非整个AddTwoInts对象。
通常在处理函数中，我们对Requst数据进行需要的操作，将结果写入到Response中。
在roscpp中，处理函数返回值是bool型，也就是服务是否成功执行。
不要理解成输入Request，返回Response
在这里：
这个函数提供两个int值求和的服务，int值从Request里面获取，
而返回数据装入Response内，这些数据类型都定义在srv文件内部，
函数返回一个boolean值。
*/
bool add(beginner_tutorialscppsrv::AddTwoInts::Request  &req,
         beginner_tutorialscppsrv::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
    // %ld 在ROS_INFO表示long int
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server"); //初始化节点
  ros::NodeHandle n; //初始化句柄，实例化node

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
    // 写明服务的处理函数，"add_two_ints"为服务名，add为处理函数
  ROS_INFO("Ready to add two ints.");
  ros::spin(); // 阻塞循环等待服务请求，
    // 如果有请求，则调用处理函数add，然后再次进入等待，
    // 直到有新的请求,然后再调用处理函数add，然后再次进入等待，如此循环
  return 0;
}
```

### 编写Client节点

add_two_ints_client.cpp

```cpp
#include "ros/ros.h"
#include "beginner_tutorialscppsrv/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorialscppsrv::AddTwoInts>("add_two_ints");
  beginner_tutorialscppsrv::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
```

#### 代码解释

```cpp
#include "ros/ros.h"
#include "beginner_tutorialscppsrv/AddTwoInts.h"
#include <cstdlib> //提供函数atoll，Convert string to long long integer
// 函数原型 long long int atoll ( const char * str )
// 形参为 char指针

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
    // // 初始化，节点命名为"add_two_ints_client"
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
    
  /*
  建立client需要用n.serviceClient<beginner_tutorialscppsrv::AddTwoInts>("add_two_ints")，
  指明服务的类型和服务的名称
  */
  ros::ServiceClient client = n.serviceClient<beginner_tutorialscppsrv::AddTwoInts>("add_two_ints");
    // 定义service客户端，service名字为"add_two_ints"，
    // service类型为 beginner_tutorialscppsrv::AddTwoInts
    
  /*
  实例化srv，设置其request消息的内容，
  这里request包含两个变量，a和b，AddTwoInts.srv
  */
  beginner_tutorialscppsrv::AddTwoInts srv;
  srv.request.a = atoll(argv[1]); // 将char指针转换为long long int
  srv.request.b = atoll(argv[2]);
  
  /*
  调用服务直接用client.call(srv)，返回结果不是response，而是是否成功调用远程服务
  由于service的调用是阻塞过程（调用的时候占用进程阻止其他代码的执行），
  一旦调用完成，将返回调用结果。
  如果service调用成功，call()函数将返回true，
  srv.response里面的值将是合法的值，即是服务端已给其赋值
  如果调用失败，call()函数将返回false，srv.response里面的值将是非法的。 
  */
  if (client.call(srv))
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
      // 如果请求服务处理成功，则可以访问服务的 response
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }
  return 0;
}
```

#### 修改CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorialscppsrv)

find_package(catkin REQUIRED COMPONENTS 
	roscpp 
	std_msgs 
	message_generation
)

add_service_files(FILES  AddTwoInts.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package( CATKIN_DEPENDS std_msgs message_runtime)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(add_two_ints_server src/add_two_ints_server.cpp)
add_dependencies(add_two_ints_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
# add_dependencies(add_two_ints_server beginner_tutorialscppsrv_generate_messages_cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
add_dependencies(add_two_ints_client ${${PROJECT_NAME}_EXPORTED_TARGETS})
# add_dependencies(add_two_ints_client beginner_tutorialscppsrv_generate_messages_cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
# 当使用当前包生成的消息和服务文件时使用 ${${PROJECT_NAME}_EXPORTED_TARGETS}

# beginner_tutorialscppsrv_generate_messages_cpp 即为项目名+_generate_messages_cpp
# 两种方式等效
```

#### package.xml

```xml
<?xml version="1.0"?>
<package format="2">
  <name>beginner_tutorialscppsrv</name>
  <version>0.0.0</version>
  <description>The beginner_tutorialscppsrv package</description>
  <maintainer email="likun@todo.todo">likun</maintainer>
  <license>TODO</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>roscpp</build_depend>
  <build_export_depend>roscpp</build_export_depend>
  <exec_depend>roscpp</exec_depend>

  <depend>std_msgs</depend>

  <build_depend>message_generation</build_depend>
  <build_export_depend>message_runtime</build_export_depend>
  <exec_depend>message_runtime</exec_depend>

  <export>
  </export>
</package> 
```

> catkin相关参考：http://docs.ros.org/melodic/api/catkin/html/howto/format2/index.html

#### 编译

```shell
catkin_make
```

编译完成后：

```shell
devel
├── cmake.lock
├── env.sh
├── include
│   └── beginner_tutorialscppsrv # AddTwoInts.srv 生成的头文件
│       ├── AddTwoInts.h
│       ├── AddTwoIntsRequest.h
│       └── AddTwoIntsResponse.h
├── lib
│   ├── beginner_tutorialscppsrv
│   │   ├── add_two_ints_client # 节点，可执行文件
│   │   └── add_two_ints_server # 节点，可执行文件
│   ├── pkgconfig
│   │   └── beginner_tutorialscppsrv.pc
│   └── python2.7
│       └── dist-packages
│           └── beginner_tutorialscppsrv
│               ├── __init__.py
│               └── srv
│                   ├── _AddTwoInts.py
│                   └── __init__.py
......
```

#### 运行测试

```shell
roscore

# server:
$ rosrun beginner_tutorialscppsrv add_two_ints_server
[ INFO] [1569106137.231066143]: Ready to add two ints.
[ INFO] [1569106145.141608854]: request: x=1, y=2
[ INFO] [1569106145.141655649]: sending back response: [3]
[ INFO] [1569106150.778730025]: request: x=1, y=3
[ INFO] [1569106150.778770743]: sending back response: [4]

# client
$ rosrun beginner_tutorialscppsrv add_two_ints_client 1 2  
[ INFO] [1569106145.141818801]: Sum: 3
$ rosrun beginner_tutorialscppsrv add_two_ints_client 1 3
[ INFO] [1569106150.778901349]: Sum: 4

# server一直循环等待请求
```

