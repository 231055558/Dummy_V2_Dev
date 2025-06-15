#!/usr/bin/env python3
"""
简单的Dummy Robot连接测试
目的：验证连接、读取状态、理解API结构
"""

import sys
import os
import time

# 添加fibre模块路径  
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.realpath(__file__))), "Firmware", "fibre", "python"))

# 导入必要的模块
from fibre import Logger, Event     # 通信日志和事件管理
import ref_tool                     # 设备发现工具

def connect_and_explore():
    """连接设备并探索其结构"""
    print("🔍 连接Dummy Robot...")
    
    # 创建通信对象
    logger = Logger(verbose=True)
    shutdown_token = Event()
    
    try:
        # 连接设备
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

def explore_device_structure(device):
    """探索设备的完整结构"""
    print("\n📋 探索设备结构...")
    
    # 获取设备的所有属性
    print("\n🔍 根级属性:")
    try:
        # 通过_remote_attributes查看所有可用属性
        if hasattr(device, '_remote_attributes'):
            attrs = device._remote_attributes
            for name, attr in attrs.items():
                print(f"   {name}: {type(attr).__name__}")
        
        # 尝试访问robot对象
        if hasattr(device, 'robot'):
            print("\n🤖 机器人对象存在，探索robot子属性...")
            robot = device.robot
            
            if hasattr(robot, '_remote_attributes'):
                robot_attrs = robot._remote_attributes
                for name, attr in robot_attrs.items():
                    print(f"   robot.{name}: {type(attr).__name__}")
                    
                    # 如果是关节，进一步探索
                    if 'joint' in name.lower():
                        try:
                            joint = getattr(robot, name)
                            if hasattr(joint, '_remote_attributes'):
                                joint_attrs = joint._remote_attributes
                                print(f"      └─ {name}属性: {list(joint_attrs.keys())}")
                        except:
                            pass
        
    except Exception as e:
        print(f"❌ 结构探索失败: {e}")

def test_basic_functions(device):
    """测试基本功能"""
    print("\n🧪 测试基本功能...")
    
    try:
        # 测试系统信息读取
        print(f"📟 设备序列号: {device.serial_number}")
        print(f"📟 设备序列号(hex): {device.serial_number:012X}")
        
        # 尝试读取温度
        try:
            temp = device.get_temperature()
            print(f"🌡️  系统温度: {temp}°C")
        except Exception as e:
            print(f"🌡️  温度读取: 不支持或失败 ({e})")
        
        # 尝试访问robot对象
        if hasattr(device, 'robot'):
            robot = device.robot
            print("🤖 机器人对象: 可访问")
            
            # 尝试读取关节状态
            joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            print("\n🔧 关节状态测试:")
            
            for i, joint_name in enumerate(joint_names, 1):
                try:
                    if hasattr(robot, joint_name):
                        joint = getattr(robot, joint_name)
                        print(f"   {joint_name}: 对象存在")
                        
                        # 尝试读取角度
                        if hasattr(joint, 'angle'):
                            angle = joint.angle
                            print(f"      └─ 角度: {angle:.2f}°")
                        
                        # 列出其他属性
                        if hasattr(joint, '_remote_attributes'):
                            attrs = list(joint._remote_attributes.keys())
                            print(f"      └─ 可用属性: {attrs}")
                            
                    else:
                        print(f"   {joint_name}: 不存在")
                        
                except Exception as e:
                    print(f"   {joint_name}: 读取失败 ({e})")
            
            # 探索运动控制API
            print("\n🎯 运动控制API探索:")
            motion_apis = [
                'move_j',           # 关节空间运动
                'move_l',           # 笛卡尔空间运动  
                'set_joint_speed',  # 设置关节速度
                'set_joint_acc',    # 设置关节加速度
                'set_command_mode', # 设置命令模式
                'homing',           # 回零
                'resting',          # 休息位置
                'calibrate_home_offset'  # 标定回零偏移
            ]
            
            for api_name in motion_apis:
                try:
                    if hasattr(robot, api_name):
                        api_obj = getattr(robot, api_name)
                        print(f"   {api_name}: ✅ 可用 - {type(api_obj).__name__}")
                    else:
                        print(f"   {api_name}: ❌ 不可用")
                except Exception as e:
                    print(f"   {api_name}: ⚠️  查询失败 ({e})")
            
            # 显示API设计说明
            print("\n💡 API设计特点:")
            print("   - 该机器人支持发送笛卡尔运动指令(move_l)")
            print("   - 但不提供当前末端位置的读取功能")
            print("   - 只能读取关节角度，末端位置需要正运动学计算")
        
    except Exception as e:
        print(f"❌ 基本功能测试失败: {e}")

def test_simple_commands(device):
    """测试简单的命令执行"""
    print("\n⚙️ 测试简单命令...")
    
    try:
        if hasattr(device, 'robot'):
            robot = device.robot
            
            # 测试使能
            print("🔋 测试机器人使能...")
            try:
                robot.set_enable(True)
                print("✅ 使能命令发送成功")
                time.sleep(1)
                
                # 禁用
                robot.set_enable(False)
                print("✅ 禁用命令发送成功")
                
            except Exception as e:
                print(f"❌ 使能控制失败: {e}")
            
            # 测试move_j命令（如果存在）
            try:
                if hasattr(robot, 'move_j'):
                    print("🎯 move_j命令: 可用")
                else:
                    print("🎯 move_j命令: 不可用")
                    
                if hasattr(robot, 'move_l'):
                    print("🌐 move_l命令: 可用")
                else:
                    print("🌐 move_l命令: 不可用")
                    
            except Exception as e:
                print(f"❌ 运动命令测试失败: {e}")
        
    except Exception as e:
        print(f"❌ 命令测试失败: {e}")

def main():
    """主程序"""
    print("=" * 50)
    print("🤖 Dummy Robot 简单连接测试")
    print("=" * 50)
    
    # 连接设备
    device = connect_and_explore()
    if device is None:
        return
    
    try:
        # 探索设备结构
        explore_device_structure(device)
        
        # 测试基本功能
        test_basic_functions(device)
        
        # 测试简单命令
        test_simple_commands(device)
        
        print("\n🎉 测试完成！")
        
    except KeyboardInterrupt:
        print("\n⚠️  用户中断")
    except Exception as e:
        print(f"\n❌ 测试出错: {e}")
    finally:
        print("\n👋 断开连接")

if __name__ == "__main__":
    main() 