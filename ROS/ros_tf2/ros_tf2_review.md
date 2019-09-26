# tf2_ros 源码

tf2_ros Documentation http://docs.ros.org/kinetic/api/tf2_ros/html/c++/index.html

## tf2_ros::StaticTransformBroadcaster

static_transform_broadcaster.h

```cpp
#ifndef TF2_ROS_STATICTRANSFORMBROADCASTER_H
#define TF2_ROS_STATICTRANSFORMBROADCASTER_H

#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2_msgs/TFMessage.h"

namespace tf2_ros
{
class StaticTransformBroadcaster{
    public:
    StaticTransformBroadcaster();
    void sendTransform(const geometry_msgs::TransformStamped & transform);
    void sendTransform(const std::vector<geometry_msgs::TransformStamped> & transforms);
    private:
    ros::NodeHandle node_; //声明句柄
    ros::Publisher publisher_; //声明发布对象
    tf2_msgs::TFMessage net_message_; //tf消息
};
}
#endif
```

static_transform_broadcaster.cpp

```cpp
#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace tf2_ros {

StaticTransformBroadcaster::StaticTransformBroadcaster()
{
  publisher_ = node_.advertise<tf2_msgs::TFMessage>("/tf_static", 100, true);
};

void StaticTransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped & msgtf)
{
  std::vector<geometry_msgs::TransformStamped> v1;
  v1.push_back(msgtf);
  sendTransform(v1);
}
    
void StaticTransformBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped> & msgtf)
{
  for (std::vector<geometry_msgs::TransformStamped>::const_iterator it_in = msgtf.begin(); it_in != msgtf.end(); ++it_in)
  {
    bool match_found = false;
    for (std::vector<geometry_msgs::TransformStamped>::iterator it_msg = net_message_.transforms.begin(); it_msg != net_message_.transforms.end(); ++it_msg)
    {
      if (it_in->child_frame_id == it_msg->child_frame_id)
      {
        *it_msg = *it_in;
        match_found = true;
        break;
      }
    }
    if (! match_found)
      net_message_.transforms.push_back(*it_in);
  }

  publisher_.publish(net_message_);
}
}
```

## tf2_ros::TransformBroadcaster

transform_broadcaster.h

```cpp
#ifndef TF2_ROS_TRANSFORMBROADCASTER_H
#define TF2_ROS_TRANSFORMBROADCASTER_H

#include "ros/ros.h"
#include "geometry_msgs/TransformStamped.h"
namespace tf2_ros
{
class TransformBroadcaster{
public:
  TransformBroadcaster();
  void sendTransform(const geometry_msgs::TransformStamped & transform);
  void sendTransform(const std::vector<geometry_msgs::TransformStamped> & transforms);
private:
  ros::NodeHandle node_;
  ros::Publisher publisher_;
};
}
#endif
```

transform_broadcaster.cpp

```cpp
#include "ros/ros.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_broadcaster.h"

namespace tf2_ros {
TransformBroadcaster::TransformBroadcaster()
{
  publisher_ = node_.advertise<tf2_msgs::TFMessage>("/tf", 100);
};

void TransformBroadcaster::sendTransform(const geometry_msgs::TransformStamped & msgtf)
{
  std::vector<geometry_msgs::TransformStamped> v1;
  v1.push_back(msgtf);
  sendTransform(v1);
}

void TransformBroadcaster::sendTransform(const std::vector<geometry_msgs::TransformStamped> & msgtf)
{
  tf2_msgs::TFMessage message;
  for (std::vector<geometry_msgs::TransformStamped>::const_iterator it = msgtf.begin(); it != msgtf.end(); ++it)
  {
    message.transforms.push_back(*it);
  }
  publisher_.publish(message);
}
}
```

