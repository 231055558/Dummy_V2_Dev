#!/usr/bin/env python3
"""
PyBullet虚拟现实统一控制器 - 精确映射版
目的：在PyBullet中操作虚拟机械臂，然后同步到真实机械臂
修复：移除复杂GUI控件，使用键盘控制避免段错误
"""

import sys
import os
import math
import time
import threading
import numpy as np

# 添加fibre模块路径
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.realpath(__file__))), "CLI-Tool", "fibre", "python"))

try:
    import pybullet as p
    import pybullet_data
    from fibre import Logger, Event
    import ref_tool
except ImportError as e:
    print(f"❌ 缺少依赖库: {e}")
    print("请安装: pip install pybullet")
    sys.exit(1)

class PyBulletRobotController:
    def __init__(self):
        self.physics_client = None
        self.robot_id = None
        self.real_robot_device = None
        self.joint_indices = []
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.initial_angles = [0, -70, 180, 0, 0, 0]  # 真实机械臂初始位置（度）
        self.initial_pose_angles = [0.0, -55.0, 150.0, -90.0, 0.0, 0.0]  # 初始位置姿态
        self.current_joint_angles = self.initial_angles.copy()
        
        # 真实角度到虚拟角度的转换参数
        # 基于测试发现：真实[0, -70, 180, 0, 0, 0] -> 虚拟[0, 70, 90, 0, 0, 0]
        self.real_to_virtual_mapping = [
            {'multiplier': -1, 'offset': 0},      # joint1: 直接映射
            {'multiplier': -1, 'offset': 5},     # joint2: 方向相反
            {'multiplier': 1, 'offset': -90},    # joint3: 偏移-90度
            {'multiplier': -1, 'offset': 0},      # joint4: 直接映射
            {'multiplier': 1, 'offset': 0},      # joint5: 直接映射
            {'multiplier': 1, 'offset': 0}       # joint6: 直接映射
        ]
        
        # 键盘控制状态
        self.selected_joint = 0
        self.angle_step = 10.0  # 每次调整的角度步长
        
        print("🤖 PyBullet虚拟现实统一控制器 - 精确映射版")
        print("=" * 60)
        
    def real_to_virtual_angles(self, real_angles):
        """将真实机械臂角度转换为虚拟环境角度"""
        virtual_angles = []
        for i, real_angle in enumerate(real_angles):
            mapping = self.real_to_virtual_mapping[i]
            virtual_angle = real_angle * mapping['multiplier'] + mapping['offset']
            virtual_angles.append(virtual_angle)
        return virtual_angles
        
    def virtual_to_real_angles(self, virtual_angles):
        """将虚拟环境角度转换为真实机械臂角度"""
        real_angles = []
        for i, virtual_angle in enumerate(virtual_angles):
            mapping = self.real_to_virtual_mapping[i]
            real_angle = (virtual_angle - mapping['offset']) / mapping['multiplier']
            real_angles.append(real_angle)
        return real_angles
        
    def start_pybullet(self):
        """启动PyBullet仿真环境"""
        print("🚀 启动PyBullet仿真环境...")
        
        try:
            # 启动PyBullet GUI
            self.physics_client = p.connect(p.GUI)
            
            # 设置额外数据路径
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            
            # 设置重力
            p.setGravity(0, 0, -9.81)
            
            # 加载地面
            p.loadURDF("plane.urdf")
            
            # 加载机器人URDF
            urdf_path = os.path.join(os.path.dirname(os.path.dirname(
                os.path.realpath(__file__))), "dummy2", "dummy2.urdf")
            
            if not os.path.exists(urdf_path):
                print(f"❌ 找不到URDF文件: {urdf_path}")
                return False
                
            # 加载机器人模型
            self.robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
            
            # 获取关节信息
            num_joints = p.getNumJoints(self.robot_id)
            print(f"📊 机器人关节数量: {num_joints}")
            
            # 找到6个主要关节的索引
            for i in range(num_joints):
                joint_info = p.getJointInfo(self.robot_id, i)
                joint_name = joint_info[1].decode('utf-8')
                if joint_name in self.joint_names:
                    self.joint_indices.append(i)
                    print(f"   关节 {joint_name}: 索引 {i}")
            
            # 设置初始关节角度
            self.set_virtual_robot_pose(self.initial_angles)
            
            # 设置相机视角
            p.resetDebugVisualizerCamera(
                cameraDistance=1.5,
                cameraYaw=45,
                cameraPitch=-30,
                cameraTargetPosition=[0, 0, 0.5]
            )
            
            print("✅ PyBullet环境启动成功")
            return True
            
        except Exception as e:
            print(f"❌ PyBullet启动失败: {e}")
            return False
        
    def connect_real_robot(self):
        """连接真实机器人"""
        print("🔍 连接真实Dummy Robot...")
        
        logger = Logger(verbose=False)
        shutdown_token = Event()
        
        try:
            self.real_robot_device = ref_tool.find_any(
                path="usb",
                serial_number=None,
                search_cancellation_token=shutdown_token,
                channel_termination_token=shutdown_token,
                timeout=10,
                logger=logger
            )
            
            if self.real_robot_device is None:
                print("⚠️  未找到真实机器人设备")
                return False
                
            print(f"✅ 真实机器人连接成功！序列号: {self.real_robot_device.serial_number:012X}")
            
            # 激活机器人
            self.real_robot_device.robot.set_enable(True)
            time.sleep(1)
            
            # 移动到初始位置
            print("🏠 移动到初始位置...")
            result = self.real_robot_device.robot.move_j(*self.initial_angles)
            print(f"   移动结果: {result}")
            
            return True
            
        except Exception as e:
            print(f"❌ 连接真实机器人失败: {e}")
            return False
            
    def set_virtual_robot_pose(self, joint_angles_deg):
        """设置虚拟机器人姿态"""
        if self.robot_id is None:
            return
            
        # 将真实角度转换为虚拟角度
        virtual_angles = self.real_to_virtual_angles(joint_angles_deg)
        
        # 转换为弧度并设置关节角度
        for i, joint_idx in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, joint_idx, math.radians(virtual_angles[i]))
            
        # 更新当前角度记录（保存的是真实角度）
        self.current_joint_angles = joint_angles_deg.copy()
        
    def sync_to_real_robot(self):
        """同步虚拟机器人状态到真实机器人"""
        if self.real_robot_device is None:
            print("⚠️  真实机器人未连接")
            return False
            
        print(f"🔄 同步关节角度到真实机器人: {[f'{a:.1f}°' for a in self.current_joint_angles]}")
        
        try:
            result = self.real_robot_device.robot.move_j(*self.current_joint_angles)
            print(f"✅ 同步完成，结果: {result}")
            return True
        except Exception as e:
            print(f"❌ 同步失败: {e}")
            return False
            
    def get_real_robot_angles(self):
        """获取真实机器人当前关节角度"""
        if self.real_robot_device is None:
            print("⚠️  真实机器人未连接")
            return None
            
        try:
            real_angles = []
            for i in range(1, 7):
                joint = getattr(self.real_robot_device.robot, f'joint_{i}')
                real_angles.append(joint.angle)
            
            print(f"📊 真实机器人当前角度: {[f'{a:.1f}°' for a in real_angles]}")
            return real_angles
        except Exception as e:
            print(f"❌ 获取角度失败: {e}")
            return None
    
    def move_to_target_pose(self, target_angles, description="目标位置"):
        """平滑运动到目标姿态"""
        print(f"🔄 运动到{description}...")
        print(f"   当前位置: {[f'{a:.1f}°' for a in self.current_joint_angles]}")
        print(f"   目标位置: {[f'{a:.1f}°' for a in target_angles]}")
        
        start_angles = self.current_joint_angles.copy()
        motion_steps = 100  # 运动步数
        motion_duration = 2.0  # 运动时长（秒）
        
        for step in range(motion_steps + 1):
            progress = step / motion_steps
            smooth_progress = progress * progress * (3.0 - 2.0 * progress)
            
            self.current_joint_angles = [
                start + (end - start) * smooth_progress 
                for start, end in zip(start_angles, target_angles)
            ]
            
            self.set_virtual_robot_pose(self.current_joint_angles)
            
            # 步进物理仿真
            for _ in range(5):
                p.stepSimulation()
                time.sleep(1/600)
            
            # 显示进度
            if step % 25 == 0 or step == motion_steps:
                print(f"   进度: {progress*100:5.1f}% - {[f'{a:.1f}°' for a in self.current_joint_angles]}")
        
        print(f"✅ 已到达{description}")
        return True
            
    def run_keyboard_control(self):
        """运行键盘控制主循环"""
        print("⌨️  启动键盘控制模式...")
        print("=" * 60)
        print("📝 控制说明:")
        print("1-6: 选择关节 (当前: joint1)")
        print("w: 增加角度 (+5度)")
        print("s: 减少角度 (-5度)")
        print("r: 重置到初始位置")
        print("go: 运动到初始位置姿态 [0°, -60°, 150°, -90°, 0°, 0°]")
        print("back: 返回初始位置 [0°, -70°, 180°, 0°, 0°, 0°]")
        print("t: 同步到真实机器人")
        print("g: 获取真实机器人角度")
        print("q: 退出程序")
        print("💡 提示: 显示的角度为真实机器人角度，虚拟环境会自动转换显示")
        print("=" * 60)
        
        try:
            while True:
                # 显示当前状态
                joint_name = self.joint_names[self.selected_joint]
                current_angle = self.current_joint_angles[self.selected_joint]
                
                print(f"\n🎯 当前选择: {joint_name} = {current_angle:.1f}°")
                print("请输入命令 (1-6/w/s/r/go/back/t/g/q): ", end='')
                
                try:
                    key = input().strip().lower()
                except EOFError:
                    break
                
                if key == 'q':
                    break
                elif key in '123456':
                    self.selected_joint = int(key) - 1
                    print(f"🎯 选择关节: {self.joint_names[self.selected_joint]}")
                elif key == 'w':
                    self.current_joint_angles[self.selected_joint] += self.angle_step
                    self.current_joint_angles[self.selected_joint] = max(-180, min(180, self.current_joint_angles[self.selected_joint]))
                    self.set_virtual_robot_pose(self.current_joint_angles)
                    print(f"⬆️  {self.joint_names[self.selected_joint]} = {self.current_joint_angles[self.selected_joint]:.1f}°")
                elif key == 's':
                    self.current_joint_angles[self.selected_joint] -= self.angle_step
                    self.current_joint_angles[self.selected_joint] = max(-180, min(180, self.current_joint_angles[self.selected_joint]))
                    self.set_virtual_robot_pose(self.current_joint_angles)
                    print(f"⬇️  {self.joint_names[self.selected_joint]} = {self.current_joint_angles[self.selected_joint]:.1f}°")
                elif key == 'r':
                    self.current_joint_angles = self.initial_angles.copy()
                    self.set_virtual_robot_pose(self.current_joint_angles)
                    print("🏠 重置到初始位置")
                elif key == 'go':
                    self.move_to_target_pose(self.initial_pose_angles, "初始位置姿态")
                elif key == 'back':
                    self.move_to_target_pose(self.initial_angles, "初始位置")
                elif key == 't':
                    print("🔄 同步到真实机器人...")
                    self.sync_to_real_robot()
                elif key == 'g':
                    print("📥 获取真实机器人角度...")
                    angles = self.get_real_robot_angles()
                    if angles:
                        self.current_joint_angles = angles
                        self.set_virtual_robot_pose(angles)
                        print("✅ 已同步真实机器人角度到虚拟环境")
                
                # 步进物理仿真
                for _ in range(10):  # 步进几次让动作更流畅
                    p.stepSimulation()
                    time.sleep(1/600)
                
        except KeyboardInterrupt:
            print("\n⚠️  用户中断程序")
        except Exception as e:
            print(f"\n❌ 控制异常: {e}")
            
    def run_simple_loop(self):
        """运行简单循环（无键盘控制）"""
        print("🔄 运行简单仿真循环...")
        print("=" * 60)
        print("📝 简单模式说明:")
        print("- 机器人将保持初始位置")
        print("- 按 Ctrl+C 退出程序")
        print("=" * 60)
        
        try:
            while True:
                p.stepSimulation()
                time.sleep(1/60)
        except KeyboardInterrupt:
            print("\n⚠️  用户中断程序")

    def cleanup(self):
        """清理资源"""
        print("🧹 清理资源...")
        
        if self.real_robot_device:
            try:
                self.real_robot_device.robot.set_enable(False)
                print("✅ 真实机器人已安全停止")
            except:
                pass
                
        if self.physics_client:
            p.disconnect()
            print("✅ PyBullet环境已关闭")
            
        print("👋 程序结束")

def main():
    """主程序"""
    controller = PyBulletRobotController()
    
    try:
        # 启动PyBullet环境
        if not controller.start_pybullet():
            return
            
        # 尝试连接真实机器人
        real_robot_connected = controller.connect_real_robot()
        if not real_robot_connected:
            print("⚠️  继续运行，但无法同步到真实机器人")
            
        # 运行主循环
        controller.run_keyboard_control()
        
    except Exception as e:
        print(f"❌ 程序出错: {e}")
    finally:
        controller.cleanup()

if __name__ == "__main__":
    main() 