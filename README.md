# openbot

## 工程代码

```bash
# openbot
git clone https://github.com/AibotBeginer/openbot.git
cd openbot && mkdir build && cd build && cmake ..
make -j8
```

## 安装依赖

* abseil

```bash
git clone https://github.com/abseil/abseil-cpp.git
# 编辑CMakeLists.txt，添加add_compile_options(-fPIC)
cd abseil-cpp && cmake -B build && cd build && cmake ..
make -j8 
sudo make install
```

* behaviortree_cpp

```bash
git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP && mkdir build && cmake ..
make -j8
sudo make install
```

* ceres-solver

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

* benchmark

```bash
git clone -b v1.9.0 https://github.com/google/benchmark.git
cd benchmark
cmake -B build -DCMAKE_BUILD_TYPE=Release -DBENCHMARK_ENABLE_TESTING=OFF
cmake --install build
```

##  编译

```bash
cd openbot
colcon build --symlink-install --packages-up-to openbot_ros --cmake-args -G Ninja
或者
colcon build --symlink-install --packages-up-to openbot_ros
```

## 环境变量

```bash
# .bashrc 或者.zshrc，添加一下环境变量
export CYBER_PATH=/usr/local/share/
export GLOG_logtostderr=1
```

## 运行

```bash
source /opt/cyber/setup.bash或者source /opt/cyber/setup.zsh
./application.system_main
```

