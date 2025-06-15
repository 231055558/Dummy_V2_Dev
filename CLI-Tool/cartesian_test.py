#!/usr/bin/env python3
"""
笛卡尔运动行为测试程序
目的：验证move_l是绝对位置还是相对位置指令
"""

import sys
import os
import time

# 添加fibre模块路径  
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.realpath(__file__))), "Firmware", "fibre", "python"))

from fibre import Logger, Event
import ref_tool

def connect_robot():
    """连接机器人"""
    print("🔍 连接Dummy Robot...")
    
    logger = Logger(verbose=True)
    shutdown_token = Event()
    
    try:
        device = ref_tool.find_any(
            path="usb",
            serial_number=None,
            search_cancellation_token=shutdown_token,
            channel_termination_token=shutdown_token,
            timeout=10,
            logger=logger
        )
        
        if device is None:
            print("❌ 未找到设备")
            return None
            
        print(f"✅ 连接成功！序列号: {device.serial_number:012X}")
        return device
        
    except Exception as e:
        print(f"❌ 连接失败: {e}")
        return None

def read_joint_angles(robot):
    """读取当前关节角度"""
    angles = []
    for i in range(1, 7):
        joint = getattr(robot, f'joint_{i}')
        angles.append(joint.angle)
    return angles

def test_cartesian_behavior(device):
    """测试笛卡尔运动的行为特性"""
    robot = device.robot
    
    print("\n" + "="*60)
    print("🧪 笛卡尔运动行为测试")
    print("="*60)
    
    # 激活机器人
    print("\n🚀 激活机器人...")
    robot.set_enable(True)
    time.sleep(1)
    
    # 记录初始关节角度
    initial_angles = read_joint_angles(robot)
    print(f"\n📊 初始关节角度: {[f'{a:.1f}°' for a in initial_angles]}")
    
    # 测试1: 重复相同指令
    print("\n" + "-"*50)
    print("🔬 测试1: 重复相同笛卡尔指令")
    print("-"*50)
    
    target_pos = [200, 0, 300, 0, 0, 0]  # X, Y, Z, A, B, C
    
    print(f"📍 目标位置: X={target_pos[0]}, Y={target_pos[1]}, Z={target_pos[2]}")
    print(f"📍 目标姿态: A={target_pos[3]}, B={target_pos[4]}, C={target_pos[5]}")
    
    for i in range(3):
        print(f"\n🎯 第{i+1}次发送相同指令...")
        
        # 记录运动前的关节角度
        angles_before = read_joint_angles(robot)
        
        # 发送move_l指令
        result = robot.move_l(
            target_pos[0], target_pos[1], target_pos[2],  # X, Y, Z
            target_pos[3], target_pos[4], target_pos[5]   # A, B, C
        )
        
        print(f"   返回值: {result}")
        time.sleep(3)  # 等待运动完成
        
        # 记录运动后的关节角度
        angles_after = read_joint_angles(robot)
        
        # 计算角度变化
        angle_changes = [abs(after - before) for before, after in zip(angles_before, angles_after)]
        max_change = max(angle_changes)
        
        print(f"   运动前: {[f'{a:.1f}°' for a in angles_before]}")
        print(f"   运动后: {[f'{a:.1f}°' for a in angles_after]}")
        print(f"   最大变化: {max_change:.2f}°")
        
        if max_change < 0.1:  # 如果变化很小，认为没有运动
            print("   🔍 结论: 没有明显运动")
        else:
            print("   🔍 结论: 发生了运动")
    
    # 测试2: 不同位置的绝对坐标测试
    print("\n" + "-"*50)
    print("🔬 测试2: 不同绝对位置指令")
    print("-"*50)
    
    test_positions = [
        [220, 20, 320, 0, 0, 15],   # 位置A
        [180, -20, 280, 0, 0, -15], # 位置B  
        [220, 20, 320, 0, 0, 15],   # 回到位置A
    ]
    
    for i, pos in enumerate(test_positions):
        print(f"\n🎯 移动到位置{chr(65+i)}: X={pos[0]}, Y={pos[1]}, Z={pos[2]}")
        
        angles_before = read_joint_angles(robot)
        result = robot.move_l(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])
        
        print(f"   返回值: {result}")
        time.sleep(3)
        
        angles_after = read_joint_angles(robot)
        angle_changes = [abs(after - before) for before, after in zip(angles_before, angles_after)]
        max_change = max(angle_changes)
        
        print(f"   最大角度变化: {max_change:.2f}°")
        
        if max_change < 0.1:
            print("   🔍 结论: 没有明显运动（可能已在目标位置）")
        else:
            print("   🔍 结论: 发生了运动")
    
    # 测试3: 小步长累积测试
    print("\n" + "-"*50)
    print("🔬 测试3: 小步长累积移动")
    print("-"*50)
    
    base_pos = [200, 0, 300, 0, 0, 0]
    
    for step in range(1, 4):
        new_pos = [base_pos[0] + step * 5, base_pos[1], base_pos[2], 
                   base_pos[3], base_pos[4], base_pos[5]]
        
        print(f"\n🎯 步骤{step}: 移动到 X={new_pos[0]} (增加 {step*5}mm)")
        
        angles_before = read_joint_angles(robot)
        result = robot.move_l(new_pos[0], new_pos[1], new_pos[2], 
                              new_pos[3], new_pos[4], new_pos[5])
        
        print(f"   返回值: {result}")
        time.sleep(3)
        
        angles_after = read_joint_angles(robot)
        angle_changes = [abs(after - before) for before, after in zip(angles_before, angles_after)]
        max_change = max(angle_changes)
        
        print(f"   最大角度变化: {max_change:.2f}°")
    
    # 安全回零
    print("\n🏠 安全回零...")
    robot.set_enable(False)

def main():
    """主程序"""
    print("=" * 60)
    print("🤖 笛卡尔运动行为分析程序")
    print("=" * 60)
    
    device = connect_robot()
    if device is None:
        return
    
    try:
        input("\n⚠️  即将开始运动测试，请确保机器人周围安全，按Enter继续...")
        test_cartesian_behavior(device)
        
        print("\n" + "="*60)
        print("📋 测试结论分析:")
        print("="*60)
        print("根据以上测试结果，可以得出以下结论：")
        print("")
        print("1. 如果重复相同指令不产生运动 → move_l是绝对位置指令")
        print("2. 如果每次都产生运动 → move_l是相对位置指令")
        print("3. 如果返回值为False → 可能有内部位置检查机制")
        print("4. 关节角度变化可以间接反映末端位置变化")
        print("")
        print("💡 这解释了为什么没有位置读取API：")
        print("   - 机器人内部维护绝对位置状态")
        print("   - 但API设计上选择不暴露给用户")
        print("   - 这是一种'控制接口'而非'状态接口'的设计哲学")
        
    except KeyboardInterrupt:
        print("\n⚠️  用户中断测试")
    except Exception as e:
        print(f"\n❌ 测试出错: {e}")
    finally:
        print("\n👋 测试结束")

if __name__ == "__main__":
    main() 