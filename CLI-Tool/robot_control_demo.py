#!/usr/bin/env python3
"""
Dummy Robot 简单控制演示程序
功能：激活机械臂 → 读取状态 → 关节运动 → 空间运动
作者：基于Fibre通信框架
"""

import sys
import os
import time

# 添加本地模块路径
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.realpath(__file__))), "Firmware", "fibre", "python"))

# 导入设备通信模块
from fibre import Logger, Event                    # 日志和事件管理
import ref_tool                                    # 设备发现和连接工具
from ref_tool.configuration import OperationAbortedException  # 异常处理


class DummyRobotController:
    """Dummy Robot 控制器类 - 封装所有机械臂操作"""
    
    def __init__(self):
        self.robot_device = None
        self.robot = None
        self.logger = Logger(verbose=True)
        self.shutdown_token = Event()
        
    def connect_robot(self, timeout=10):
        """连接机械臂设备"""
        print("🔍 正在搜索Dummy Robot设备...")
        
        try:
            # 使用ref_tool自动发现并连接设备
            self.robot_device = ref_tool.find_any(
                path="usb",                           # USB连接方式
                serial_number=None,                   # 接受任何序列号
                search_cancellation_token=self.shutdown_token,
                channel_termination_token=self.shutdown_token,
                timeout=timeout,
                logger=self.logger
            )
            
            if self.robot_device is None:
                raise Exception("未找到Dummy Robot设备！请检查USB连接和设备电源。")
            
            # 获取机器人对象
            self.robot = self.robot_device.robot
            
            print(f"✅ 设备连接成功！序列号: {self.robot_device.serial_number:012X}")
            return True
            
        except Exception as e:
            print(f"❌ 设备连接失败: {e}")
            return False
    
    def activate_robot(self):
        """激活机械臂 - 使能所有关节"""
        print("\n🚀 正在激活机械臂...")
        
        try:
            # 启用机器人系统
            self.robot.set_enable(True)
            time.sleep(1)  # 等待系统响应
            
            print("✅ 机械臂激活成功！")
            return True
            
        except Exception as e:
            print(f"❌ 机械臂激活失败: {e}")
            return False
    
    def read_current_state(self):
        """读取机械臂当前状态"""
        print("\n📊 读取机械臂当前状态...")
        
        try:
            # 读取6轴关节角度
            joint_angles = []
            joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            
            print("🔧 当前关节角度:")
            for i, joint_name in enumerate(joint_names, 1):
                joint = getattr(self.robot, joint_name)
                angle = joint.angle
                joint_angles.append(angle)
                print(f"   关节{i}: {angle:8.2f}°")
            
            # 末端位置和姿态说明
            print("\n🌐 末端位置和姿态:")
            print("   ℹ️  注意: 该机器人固件不提供末端位置读取API")
            print("   ℹ️  可用功能:")
            print("      - ✅ 发送笛卡尔运动指令 (move_l)")
            print("      - ✅ 读取关节角度 (joint.angle)")
            print("      - ❌ 读取当前末端位置 (需要正运动学计算)")
            
            # 读取系统温度
            try:
                temperature = self.robot_device.get_temperature()
                print(f"\n🌡️  系统温度: {temperature:.1f}°C")
            except:
                print("\n🌡️  系统温度: 读取失败")
            
            return joint_angles
            
        except Exception as e:
            print(f"❌ 状态读取失败: {e}")
            return None
    
    def move_joints(self, target_angles, description="关节运动"):
        """使用关节角度进行运动"""
        print(f"\n🎯 执行{description}...")
        print(f"   目标角度: {[f'{a:6.1f}°' for a in target_angles]}")
        
        try:
            # 使用move_j方法进行关节空间运动
            self.robot.move_j(
                target_angles[0],  # J1
                target_angles[1],  # J2  
                target_angles[2],  # J3
                target_angles[3],  # J4
                target_angles[4],  # J5
                target_angles[5]   # J6
            )
            
            print("✅ 关节运动指令发送成功！")
            return True
            
        except Exception as e:
            print(f"❌ 关节运动失败: {e}")
            return False
    
    def move_cartesian(self, x, y, z, a, b, c, description="空间运动"):
        """使用笛卡尔坐标进行运动"""
        print(f"\n🌐 执行{description}...")
        print(f"   目标位置: X={x:6.1f}, Y={y:6.1f}, Z={z:6.1f}mm")
        print(f"   目标姿态: A={a:6.1f}°, B={b:6.1f}°, C={c:6.1f}°")
        
        try:
            # 使用move_l方法进行笛卡尔空间运动
            self.robot.move_l(x, y, z, a, b, c)
            
            print("✅ 空间运动指令发送成功！")
            return True
            
        except Exception as e:
            print(f"❌ 空间运动失败: {e}")
            return False
    
    def wait_for_motion_complete(self, timeout=10):
        """等待运动完成"""
        print("⏳ 等待运动完成...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                # 检查是否还在运动中（这个方法可能需要根据实际API调整）
                # is_moving = self.robot.is_moving()  # 如果有这个方法的话
                # if not is_moving:
                #     break
                time.sleep(0.5)  # 简单等待，实际可根据反馈优化
            except:
                break
        
        print("✅ 运动执行完成")
    
    def disconnect(self):
        """断开连接并清理资源"""
        print("\n🔌 断开设备连接...")
        try:
            if self.robot:
                self.robot.set_enable(False)  # 禁用机器人
            self.shutdown_token.set()
            print("✅ 设备安全断开")
        except Exception as e:
            print(f"⚠️  断开过程中出现警告: {e}")


def main():
    """主程序入口"""
    print("=" * 60)
    print("🤖 Dummy Robot 控制演示程序")
    print("=" * 60)
    
    # 创建控制器实例
    controller = DummyRobotController()
    
    try:
        # 步骤1: 连接设备
        if not controller.connect_robot():
            return
        
        # 步骤2: 激活机械臂
        if not controller.activate_robot():
            return
        
        # 步骤3: 读取当前状态
        current_angles = controller.read_current_state()
        if current_angles is None:
            return
        
        # 等待用户确认
        input("\n按Enter键继续执行运动演示...")
        
        # # 步骤4: 关节空间运动演示
        # print("\n" + "="*50)
        # print("📍 演示1: 关节空间运动")
        # print("="*50)
        
        # 小幅度关节运动 - 安全的测试运动
        test_angles = [
            0,0,0,0,0,0
        ]
        
        controller.move_joints(test_angles, "小幅度关节测试")
        controller.wait_for_motion_complete()
        
        # time.sleep(2)
        
        # # 回到原位
        # controller.move_joints(current_angles, "返回原始位置")
        # controller.wait_for_motion_complete()
        
        # # # 步骤5: 笛卡尔空间运动演示
        # print("\n" + "="*50)
        # print("📍 演示2: 笛卡尔空间运动")
        # print("="*50)
        
        # # 简单的空间位置运动
        # controller.move_cartesian(
        #     x=100,   y=0,     z=300,     # 位置 (mm)
        #     a=0,     b=90,     c=0,       # 姿态 (度)
        #     description="移动到安全测试位置"
        # )
        # controller.wait_for_motion_complete()
        
        # time.sleep(2)
        
        # # 小幅移动
        # controller.move_cartesian(
        #     x=220,   y=20,    z=320,     # 小幅位置变化
        #     a=0,     b=0,     c=15,      # 小幅姿态变化
        #     description="小幅度空间移动"
        # )
        # controller.wait_for_motion_complete()

        # # 回到原位
        # controller.move_joints(current_angles, "返回原始位置")
        # controller.wait_for_motion_complete()
        
        print("\n🎉 所有演示完成！")
        
    except OperationAbortedException:
        print("\n⚠️  操作被用户中断")
    except KeyboardInterrupt:
        print("\n⚠️  程序被用户中断 (Ctrl+C)")
    except Exception as e:
        print(f"\n❌ 程序执行出错: {e}")
    finally:
        # 确保安全断开连接
        controller.disconnect()
        print("\n👋 程序结束")


if __name__ == "__main__":
    main() 