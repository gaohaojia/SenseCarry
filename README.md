# SenseCarry

高颢嘉

# 下载代码

```bash
cd ~/
git clone --recursive https://github.com/gaohaojia/SenseCarry
```

# 安装驱动

以下驱动请根据实际情况按需安装，Mid 360 为必要驱动，即使不使用 Mid 360，也需要安装相关驱动。

## Mid 360 驱动（必要）

```bash
cd ~/
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
```

## Realsense 驱动

Realsense 驱动当前不支持 Jetson orin AGX。

```bash
sudo apt update
sudo apt install ros-humble-librealsense2* ros-humble-realsense2-* -y
```

## 宇树 Unilidar 驱动

```bash
cd ~/SenseCarry/src
git clone https://github.com/gaohaojia/Unitree_lidar_ros2
```

## 北科天绘 Dom 60 驱动

当前还存在部分 Bug。

# 配置环境

```bash
cd ~/SenseCarry
sudo apt update
sudo apt install ros-humble-desktop-full python3-rosdep libgflags-dev libgoogle-glog-dev ros-humble-pcl* -y
sudo rosdep init
rosdep update
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

# 参数配置

~/SenseCarry/src/bringup/launch/real_robot.launch.py 中的参数配置。

| 参数名称 | 可选参数 | 参数描述 | 默认参数 |
| --- | --- | --- | --- |
| lidar_type | mid360、unilidar、dom60 | 雷达类型。只有安装对应雷达驱动后，设置此参数才有效。 | mid360 |
| robot_type | simulated、c1 | 机器人类型，即对应的预先设定好的参数值。 | simulated |

## 雷达角度调整

若雷达并非水平向上安装，则需进行此调整。

修改 ~/SenseCarry/src/lidar_transform/config/lidar_transform_params_simulated.yaml 中带有 lidar 前缀的六个参数，此六个参数表示雷达相较于小车中心位置的偏移。

## 配置服务器（可选）

修改 ~/SenseCarry/src/robot_communication/config/robot_communication_params.yaml 的 network_ip 为服务器 ip，network_port 为服务器端口（详情请查看 [SenseServer 文档](https://www.notion.so/SenseServer-66ec20dd852645f99df47974ee3cc9cb?pvs=21) ）。

若同时启动多台机器人，请为 id 不为 0 的机器人设置偏移坐标实现全局点云配准。修改上述配置文件中带有 multi 前缀的六个参数，此六个参数表示为该机器人相对于 robot_0 机器人的偏移。

# 编译

本代码实现对 x86 和 arm 的自适应，会自动根据不同架构编译不同代码。

```bash
bash ~/SenseCarry/toBuild.sh
```

# 启动

将 [robot_id] 替换为当前机器人的 id（当前支持的范围为 0-4），若单机启动请输入 0。

```bash
bash ~/SenseCarry/run.sh [robot_id]
```

# 保存地图

先建立保存文件夹。

```bash
cd ~/SenseCarry/
mkdir save
```

之后运行代码，待扫描全部完成后，按下 ctrl+c 终止代码将自动保存地图。