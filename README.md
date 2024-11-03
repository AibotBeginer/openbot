# openbot

## :seedling:  简介

> **<font color='green'>openbot</font>** 不依赖 **<font color='red'>ROS/ROS2</font>** 的机器人视觉定位&建图、导航系统框架，完全由C++开发。打造一个linux系统级别，支持分布式、高吞吐、低延时、易部署、易调试、高性能的机器人算法框架：
>
> * 支持CyberRT通信机制
> * 支持mqtt、grpc、zmq、http网络通信机制 **（TODO）**
> * 支持原生ROS/ROS2通用msgs消息格式
> * 支持3D视觉SLAM建图 **（TODO）**
> * 支持2D/3D占用栅格地图全局路径规划，以及全局路径的动态库插件化 **（TODO）**
> * 支持3D路径跟踪和物体跟随控制，以及控制器的动态库插件化 **（TODO）**
> * 支持数据、算法和ROS2数据生态互联与应用扩展
> * 支持docker **（TODO）**
> * 支持cmake、bazel系统编译 **（TODO）**
> * 支持行为树调度 **（TODO）**
> * 完善的说明文档和使用文档 **（TODO）**

## :shamrock:工程代码

```bash
# openbot
git clone https://github.com/AibotBeginer/openbot.git
cd openbot && mkdir build && cd build && cmake ..
make -j8
```

## :leaves:  安装依赖

* :man_cartwheeling: abseil

```bash
git clone https://github.com/abseil/abseil-cpp.git
# 编辑CMakeLists.txt，添加add_compile_options(-fPIC)
cd abseil-cpp && cmake -B build && cd build && cmake ..
make -j8 
sudo make install
```

* :basketball_woman: behaviortree_cpp

```bash
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP && mkdir build && cmake ..
make -j8
sudo make install
```

* :golfing_woman: ceres-solver

```bash
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver && mkdir build && cd build && cmake ..
make -j8
sudo make install
```

* :business_suit_levitating: apt安装

```bash
sudo apt install libtinyxml2-dev \
	liblua5.3-dev \
	ninja-build \
	libompl-dev
```

* :person_fencing: G2O

```bash
git clone -b 20230223_git https://github.com/RainerKuemmerle/g2o.git
cd g2o && mkdir build && cd build && cmake ..
make -j8
sudo make install
```

* :woman_playing_handball: cyberRT

```bash
git clone git clone https://gitee.com/minhanghuang/CyberRT.git

# 安装third_party
cd CyberRT 
sudo python3 install.py --install_prefix /opt/cyber
source /opt/cyber/setup.zsh or source /opt/cyber/setup.bash

# 安装cyber
cd CyberRT && cmake -B build
cd build && cmake -DCMAKE_INSTALL_PREFIX=/opt/cyber ..
make -j8
sudo make install
```

* :woman_playing_water_polo: benchmark

```bash
git clone -b v1.9.0 https://github.com/google/benchmark.git
cd benchmark
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBENCHMARK_ENABLE_TESTING=OFF
cmake --install build
```

##  :cactus: 编译

```bash
cd openbot
mkdir build && cd build && cmake ..
make -j8
sudo make install
```

##  :desert_island: 环境变量

```bash
# .bashrc 或者.zshrc，添加一下环境变量
export CYBER_PATH=/usr/local/share/
export GLOG_logtostderr=1
```

## :four_leaf_clover: 运行

```bash
source /opt/cyber/setup.bash或者source /opt/cyber/setup.zsh
./application.system_main
```

