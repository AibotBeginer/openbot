# openbot

## :golf: Framework

![openbot_framework](./images/openbot.jpeg)

## :seedling:  Introduction

**<font color='green'>openbot</font>** 不依赖 **<font color='red'>ROS/ROS2</font>** 的机器人视觉定位&建图、导航系统框架，完全由C++开发。打造一个linux系统级别，支持分布式、高吞吐、低延时、易部署、易调试、高性能的机器人算法框架：

- [x] 支持CyberRT通信机制
- [ ] 支持mqtt、grpc、zmq、http网络通信机制
- [x] 支持原生ROS/ROS2通用msgs消息格式
- [ ] 支持相机（realsense2）、IMU和雷达驱动
- [ ] 支持多传感器数据融合
- [ ] 支持激光SLAM建图
- [x] 支持3D视觉SLAM建图
- [x] 支持2D/3D占用栅格地图全局路径规划，以及全局路径的动态库插件化 
- [x] 支持3D路径跟踪和物体跟随控制，以及控制器的动态库插件化 
- [ ] 支持雷达/视觉感知、预测、识别和实例分割功能
- [x] 支持数据、算法和ROS2数据生态互联与应用扩展
- [x] 支持docker (**<font color='green'>x86-64 & aarch64 系统平台</font>**)
- [ ] 支持本地化网页显示
- [x] 支持cmake、bazel系统编译
- [ ] 支持行为树调度 
- [x] 完善的说明文档和使用文档  

* :shamrock: Github Code

```bash
# openbot
git clone https://github.com/AibotBeginer/openbot.git
```

## :house_with_garden: Usage

### :tanabata_tree:详细文档

**<font color='green'>参考</font>** ：[openbot详细文档](https://openbot-doc.readthedocs.io/en/latest/)

### :cactus: 编译

```bash
cd openbot
mkdir build && cd build && cmake ..
make -j8
sudo make install
```

### :desert_island: 环境变量

```bash
# .bashrc 或者.zshrc，添加一下环境变量
export CYBER_PATH=/usr/local/share/
export GLOG_logtostderr=1
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0
```

### :four_leaf_clover: 运行

* 激活环境变量

```bash
source /opt/cyber/setup.bash或者source /opt/cyber/setup.zsh
```

* 运行模块

```bash
cd /opt/openbot/share/openbot/launch

# bridge
python3 ./openbot.bridge.launch.py

# control
python3 ./openbot.control.launch.py

# drviers
python3 ./openbot.drviers.launch.py

# planning
python3 ./openbot.planning.launch.py

# mapping
python3 ./openbot.mapping.launch.py
```

## :leaves: License

Openbot is released under the [Apache 2.0 license](https://github.com/AibotBeginer/openbot/blob/main/LICENSE).

## :turtle: Acknowledgments

In the development of Openbot, we stand on the shoulders of the following repositories:

* [Apollo](https://github.com/ApolloAuto/apollo): Apollo is a high performance, flexible architecture which accelerates the development, testing, and deployment of Autonomous Vehicles.

* [Autoware Universe](https://github.com/autowarefoundation/autoware.universe): Autoware Universe serves as a foundational pillar within the Autoware ecosystem, playing a critical role in enhancing the core functionalities of autonomous driving technologies. This repository is a pivotal element of the Autoware Core/Universe concept, managing a wide array of packages that significantly extend the capabilities of autonomous vehicles.

* [rtabmap](https://github.com/introlab/rtabmap): **RTAB-Map** (Real-Time Appearance-Based Mapping) is a RGB-D, Stereo and Lidar Graph-Based SLAM approach based on an incremental appearance-based loop closure detector. 
* [rtabmap_ros](https://github.com/introlab/rtabmap_ros): This package is a ROS wrapper of [RTAB-Map](http://introlab.github.io/rtabmap) (Real-Time Appearance-Based Mapping), a RGB-D SLAM approach based on a global loop closure detector with real-time constraints.



