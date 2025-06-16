## 基于Dummy2 任同学版本设计的简单开发程序

### CLI_Tool
是原始的终端交互程序，已经经过二次开发，在pybullet_robot_controller.py是一个将现实机械臂与拟真环境中的机械臂通过urdf文件实现同步的代码

### dummy_real_workshop
基于真实的机械臂传感器电机以及数据进行python代码与机械臂交互的代码

### dummy_workshop
基于拟真环境中的机械臂关节以及相机进行python代码与机械臂交互的代码

## 快速开始
并不建议立刻按照下面内容完全安装，可以在使用时按照需要安装以避免奇怪的问题  
ubuntu 22.04(best choice)

```requirments
pybullet
cv2(opencv-python)
ultralytics
pyrealsense
open3d
matplotlib
numpy
PIL
```
