# ar_track_alvar

这是一个可以识别 AR tag 的ROS 库

github源码：

https://github.com/ros-perception/ar_track_alvar/tree/kinetic-devel

## AR tag 简介（ArUco）

- 是有7x7个方块组成，每个方块有黑白两种可能，最外圈只能是黑色
- 必须保证每个 AR tag 不能通过旋转去和其他ID的 AR tag 不能匹配
- 一共提供 1024个ID，即不同的 ARtag
- ARtag-ArUco 标签可以只使用 单目相机+单个标签 实现相机和标签相对位置的确定

![ar_track_alvar/artags.png](res/artags-1568378634399.png)

## Overview

功能：

- 生成不同大小，分辨率和数据/ ID编码的AR标签
- 识别和跟踪单个AR标签的位姿，并可以整合kinect深度数据（当kinect可用时）以获得更好的位姿估计。
- 识别和跟踪由多个标签组成的“捆绑”的位姿。 这可以使位姿估计更稳定，并且提高对遮挡的鲁棒性以及支持对多边物体的跟踪。
- 使用相机图像自动计算“捆绑”标签之间的空间关系，这样用户就不必手动测量并在XML文件中输入标签位置以使用“捆绑”标签功能（貌似当前不可用）

## 安装

kinetic版本：

```shell
# 安装二进制包：
$ sudo apt-get install ros-kinetic-ar-track-alvar

# 安装源码包：
## 使用git clone 从github下载源码并编译
$ git clone https://github.com/ros-perception/ar_track_alvar.git
...
$ catkin_make
```

二进制包提供了launch文件：

```shell
➜ ar_track_alvar tree
.
├── bundles
│   ├── table_8_9_10.xml
│   ├── tags8and9.xml
│   └── truthTableLeg.xml
├── cmake
│   ├── ar_track_alvarConfig.cmake
│   └── ar_track_alvarConfig-version.cmake
├── launch
│   ├── pr2_bundle.launch
│   ├── pr2_bundle_no_kinect.launch
│   ├── pr2_indiv.launch
│   ├── pr2_indiv_no_kinect.launch
│   └── pr2_train.launch
└── package.xml
# /opt/ros/kinetic/share/ar_track_alvar/launch
```



## 生成 AR tags

