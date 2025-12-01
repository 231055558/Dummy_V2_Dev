## 基于Dummy2 任同学版本设计的简单开发程序

### CLI_Tool
是原始的终端交互程序，已经经过二次开发，在pybullet_robot_controller.py是一个将现实机械臂与拟真环境中的机械臂通过urdf文件实现同步的代码

### dummy_real_workshop
基于真实的机械臂传感器电机以及数据进行python代码与机械臂交互的代码

### dummy_workshop
基于拟真环境中的机械臂关节以及相机进行python代码与机械臂交互的代码

## 环境配置

### 系统要求
ubuntu 22.04(推荐)

### ROS2安装
```bash
wget http://fishros.com/install -O fishros && . fishros
```
选择安装ROS2 Humble

### 基础依赖
```bash
# ARM交叉编译工具链
sudo apt install gcc-arm-none-eabi

# ROS2相关包
sudo apt install ros-humble-control-msgs
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-servo
```

### Intel RealSense SDK(可选)
```bash
# 添加公钥和仓库
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/librealsense.list

# 安装SDK
sudo apt-get update
sudo apt-get install librealsense2-dkms librealsense2-dev librealsense2-utils
```

### Python环境
```bash
# 安装pip
sudo apt install python3-pip

# 安装依赖包
pip install pyusb pybullet opencv-python ultralytics pyrealsense2 open3d matplotlib numpy pillow
```

### USB设备权限配置
```bash
# 1. 插入设备后查看设备信息
lsusb  # 找到ODrive设备的VID和PID(通常是1209:0d32)

# 2. 创建udev规则
sudo nano /etc/udev/rules.d/99-dummy-arm.rules
# 添加以下内容(替换为实际的VID和PID):
# SUBSYSTEM=="usb", ATTRS{idVendor}=="1209", ATTRS{idProduct}=="0d32", MODE="0666"

# 3. 重载规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 4. 重新插拔USB设备
```

## 快速启动
```bash
# 在工作空间根目录
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# 启动demo
ros2 launch dummy_moveit_config demo_real_arm.launch.py

# 新终端运行控制器
source install/setup.bash
ros2 run dummy_controller dummy_arm_controller

# 新终端运行规划器
source install/setup.bash
python3 moveit_rviz_planner.py
```

**注意**: 不建议一次性安装所有依赖，按需安装可避免潜在问题
