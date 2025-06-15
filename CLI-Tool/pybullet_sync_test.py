#!/usr/bin/env python3
"""
PyBullet同步测试程序
用于验证虚拟环境和真实机械臂的角度映射是否正确
"""

import sys
import os
import math
import time

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
    sys.exit(1)

def test_angle_mapping():
    """测试角度映射的正确性"""
    print("🔍 PyBullet角度映射测试程序")
    print("=" * 50)
    
    # 初始化PyBullet
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    
    # 加载机器人URDF
    urdf_path = os.path.join(os.path.dirname(os.path.dirname(
        os.path.realpath(__file__))), "dummy2", "dummy2.urdf")
    
    if not os.path.exists(urdf_path):
        print(f"❌ 找不到URDF文件: {urdf_path}")
        return
        
    robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
    
    # 连接真实机器人
    print("🔍 连接真实Dummy Robot...")
    logger = Logger(verbose=False)
    shutdown_token = Event()
    
    try:
        real_robot = ref_tool.find_any(
            path="usb", serial_number=None, search_cancellation_token=shutdown_token,
            channel_termination_token=shutdown_token, timeout=10, logger=logger
        )
        
        if real_robot is None:
            print("⚠️  未找到真实机器人，仅测试虚拟环境")
            test_virtual_only(robot_id)
            return
            
        print(f"✅ 真实机器人连接成功！序列号: {real_robot.serial_number:012X}")
        real_robot.robot.set_enable(True)
        time.sleep(1)
        
        # 测试各个关节
        test_joint_sync(robot_id, real_robot)
        
    except Exception as e:
        print(f"❌ 连接失败: {e}")
    finally:
        if 'real_robot' in locals() and real_robot:
            real_robot.robot.set_enable(False)
        p.disconnect()

def test_virtual_only(robot_id):
    """仅测试虚拟环境"""
    print("🎮 仅测试虚拟环境角度设置...")
    
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    joint_indices = []
    
    # 获取关节索引
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        if joint_name in joint_names:
            joint_indices.append(i)
    
    # 测试角度
    test_angles = [0, -70, 180, 0, 0, 0]
    direction_multipliers = [-1, 1, -1, -1, -1, -1]
    
    print(f"📊 设置测试角度: {test_angles}")
    
    for i, angle in enumerate(test_angles):
        corrected_angle = angle * direction_multipliers[i]
        p.resetJointState(robot_id, joint_indices[i], math.radians(corrected_angle))
    
    print("✅ 虚拟机器人已设置为测试姿态")
    print("按 Ctrl+C 退出...")
    
    try:
        while True:
            p.stepSimulation()
            time.sleep(1/60)
    except KeyboardInterrupt:
        print("\n👋 测试结束")

def test_joint_sync(robot_id, real_robot):
    """测试关节同步"""
    print("🔄 测试关节同步...")
    
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
    joint_indices = []
    direction_multipliers = [-1, 1, -1, -1, -1, -1]
    
    # 获取关节索引
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        joint_name = joint_info[1].decode('utf-8')
        if joint_name in joint_names:
            joint_indices.append(i)
    
    # 测试序列
    test_sequences = [
        ([0, -70, 180, 0, 0, 0], "初始位置"),
        ([10, -70, 180, 0, 0, 0], "joint1 +10度"),
        ([0, -60, 180, 0, 0, 0], "joint2 -60度"),
        ([0, -70, 170, 0, 0, 0], "joint3 170度"),
        ([0, -70, 180, 10, 0, 0], "joint4 +10度"),
        ([0, -70, 180, 0, 10, 0], "joint5 +10度"),
        ([0, -70, 180, 0, 0, 10], "joint6 +10度"),
    ]
    
    for test_angles, description in test_sequences:
        print(f"\n📊 测试: {description}")
        print(f"   目标角度: {[f'{a:.1f}°' for a in test_angles]}")
        
        # 设置虚拟机器人
        for i, angle in enumerate(test_angles):
            corrected_angle = angle * direction_multipliers[i]
            p.resetJointState(robot_id, joint_indices[i], math.radians(corrected_angle))
        
        # 同步到真实机器人
        result = real_robot.robot.move_j(*test_angles)
        print(f"   同步结果: {result}")
        
        time.sleep(2)  # 等待动作完成
        
        # 获取真实机器人当前角度
        real_angles = []
        for i in range(1, 7):
            joint = getattr(real_robot.robot, f'joint_{i}')
            raw_angle = joint.angle
            corrected_angle = raw_angle * direction_multipliers[i-1]
            real_angles.append(corrected_angle)
        
        print(f"   真实角度: {[f'{a:.1f}°' for a in real_angles]}")
        print(f"   角度差异: {[f'{abs(t-r):.1f}°' for t, r in zip(test_angles, real_angles)]}")
        
        input("按回车继续下一个测试...")
    
    print("\n✅ 所有测试完成！")

if __name__ == "__main__":
    test_angle_mapping() 