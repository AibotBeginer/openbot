# 2 程序运行

程序运行的方式有2种：

* 方式1：使用源码方式进行安装运行（不推荐，效率较低，安装依赖繁琐）
* 方式2：使用docker方式进行安装运行 **<font color='red'>（推荐）</font>** 

## 1 源码方式

* abseil

```bash
git clone https://github.com/abseil/abseil-cpp.git
# 编辑CMakeLists.txt，添加add_compile_options(-fPIC)
cd abseil-cpp && cmake -B build && cd build && cmake ..
make -j8 
sudo make install
```

*  behaviortree_cpp

```bash
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP && mkdir build && cmake ..
make -j8
sudo make install
```

*  ceres-solver

```bash
git clone https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver && mkdir build && cd build && cmake ..
make -j8
sudo make install
```

* apt安装

```bash
sudo apt install libtinyxml2-dev \
	liblua5.3-dev \
	ninja-build \
	libompl-dev
```

*  G2O

```bash
git clone -b 20230223_git https://github.com/RainerKuemmerle/g2o.git
cd g2o && mkdir build && cd build && cmake ..
make -j8
sudo make install
```

* cyberRT

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

*  benchmark

```bash
git clone -b v1.9.0 https://github.com/google/benchmark.git
cd benchmark
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBENCHMARK_ENABLE_TESTING=OFF
cmake --install build
```

* osqp

```bash
git clone -b release-0.6.3 https://github.com/osqp/osqp.git
cd osqp && git submodule update --init --recursive
mkdir build && cd build && cmake ..
make -j8
sudo make install
```

## 2 Docker（推荐）

### 2.1 docker安装

```bash
cd openbot/docker/scripts
sudo ./install_docker.sh
```

### 2.2 x86-64平台构造镜像

```bash
cd openbot/docker
./build_docker.x86_64.sh -f ./dockerfile/openbot.x86_64.dockerfile
```

### 2.3 aarch64平台构造镜像

```bash
./build_docker.aarch64.orin.sh -f ./dockerfile/openbot.aarch64.orin.dockerfile
```

### 2.4 运行docker

* 运行x86-64平台构造镜像

```bash
./run_openbot.sh
```

* 运行aarch64平台构造镜像

```
./run_openbot.sh aarch64
```

### 2.5 在docker容器中配置环境变量

在openbot的docker容器中配置.bashrc环境变量如下

```bash
### Openbot ###
export LD_LIBRARY_PATH=/opt/openbot/lib:$LD_LIBRARY_PATH
export CYBER_PATH=/usr/local/share/
export GLOG_logtostderr=1
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0
export OPENBOT_ENV=/home/quandy/workspace/project/openbot/ # 根据自己的目录修改

### ROS2 build cmd ###
alias openbot_build='colcon build --symlink-install --packages-up-to openbot_ros'
```

## 3 代码编译

* 进入docker

```bash
# x86-64平台
cd openbot/docker
./run_openbot.sh 

# aarch64平台
cd openbot/docker
./run_openbot.sh aarch64
```

* 相关的docker运行指令

```bash
# 运行存在的容器
docker exec -it SpaceHero /bin/bash

# 重启容器
docker restart SpaceHero

# 停止容器
docker stop SpaceHero
```

* 编译openbot

```bash
cd openbot
mkdir build && cd build && cmake ..
make -j8
sudo make install
```
