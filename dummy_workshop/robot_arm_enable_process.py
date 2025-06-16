#!/usr/bin/env python3
"""
机械臂使能过程模拟器
从未使能状态 [0, 70, 90, 0, 0, 0] 平滑过渡到预备状态 [0, 60, 60, 90, 0, 0]
"""

import sys
import os
import math
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "CLI_Tool"))

try:
    import pybullet as p
    import pybullet_data
except ImportError:
    print("请安装: pip install pybullet")
    sys.exit(1)

class RobotArmEnableProcess:
    def __init__(self):
        self.physics_client = None
        self.robot_id = None
        self.joint_indices = []
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 状态定义
        self.disabled_state = [0.0, 70.0, 90.0, 0.0, 0.0, 0.0]  # 未使能状态
        self.ready_state = [0.0, 60.0, 60.0, 90.0, 0.0, 0.0]    # 使能预备状态
        self.current_joint_angles = self.disabled_state.copy()
        
        # 使能参数
        self.enable_duration = 3.0
        self.enable_steps = 120
    
    def start_pybullet(self):
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        
        # 加载机械臂
        urdf_path = os.path.join(os.path.dirname(os.path.dirname(
            os.path.realpath(__file__))), "dummy2", "dummy2.urdf")
        self.robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
        
        # 获取关节索引
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            if joint_name in self.joint_names:
                self.joint_indices.append(i)
        
        # 设置相机视角
        p.resetDebugVisualizerCamera(
            cameraDistance=1.8,
            cameraYaw=45,
            cameraPitch=-25,
            cameraTargetPosition=[0, 0, 0.4]
        )
    
    def set_robot_to_disabled_state(self):
        for i, joint_idx in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, joint_idx, math.radians(self.disabled_state[i]))
        
        for _ in range(10):
            p.stepSimulation()
            time.sleep(0.001)
    
    def smooth_step(self, t):
        """S曲线插值"""
        return t * t * (3.0 - 2.0 * t)
    
    def interpolate_joint_angles(self, start_angles, end_angles, progress):
        """平滑插值关节角度"""
        smooth_progress = self.smooth_step(progress)
        return [start + (end - start) * smooth_progress 
                for start, end in zip(start_angles, end_angles)]
    
    def execute_enable_process(self):
        print(f"🔄 使能过程: {self.disabled_state} → {self.ready_state}")
        
        start_time = time.time()
        step_duration = self.enable_duration / self.enable_steps
        
        for step in range(self.enable_steps + 1):
            progress = step / self.enable_steps
            
            # 插值计算关节角度
            self.current_joint_angles = self.interpolate_joint_angles(
                self.disabled_state, self.ready_state, progress
            )
            
            # 更新机械臂姿态
            for i, joint_idx in enumerate(self.joint_indices):
                p.resetJointState(self.robot_id, joint_idx, 
                                math.radians(self.current_joint_angles[i]))
            
            p.stepSimulation()
            
            # 显示进度
            if step % 30 == 0 or step == self.enable_steps:
                print(f"进度: {progress*100:5.1f}% - {[f'{a:.1f}°' for a in self.current_joint_angles]}")
            
            time.sleep(step_duration)
        
        print(f"✅ 使能完成，用时: {time.time() - start_time:.2f}秒")
    
    def run(self):
        print("🤖 机械臂使能过程模拟器")
        
        self.start_pybullet()
        self.set_robot_to_disabled_state()
        
        print(f"未使能状态: {self.disabled_state}")
        print(f"预备状态: {self.ready_state}")
        input("按 Enter 开始使能...")
        
        self.execute_enable_process()
        
        print("观察最终姿态，按 Enter 退出...")
        input()
        
        p.disconnect()

def main():
    simulator = RobotArmEnableProcess()
    try:
        simulator.run()
    except KeyboardInterrupt:
        p.disconnect()

if __name__ == "__main__":
    main() 