import sys
import os
import time

# 获取项目根目录的绝对路径
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

# 导入并设置CLI_Tool环境
from CLI_Tool.dummy_robot_integrated import setup_environment
setup_environment()

# 现在可以安全地导入DummyRobotIntegrated
from CLI_Tool.dummy_robot_integrated import DummyRobotIntegrated

def main():
    """示例用法"""
    # 创建机器人控制器实例（选择是否连接真实机械臂）
    robot = DummyRobotIntegrated(connect_real_robot=True)
    time.sleep(5)  # 等待初始化完成
    
    try:
        # 测试back和go功能
        print("\n测试回到初始位置...")
        robot.back()
        time.sleep(5)  # 等待运动完成

        result = robot.current_joint_angles
        kine = robot.forward_kinematics(result)
        print(kine)
        
        print("\n测试移动到工作位置...")

        time.sleep(5)  # 等待运动完成

        result = robot.current_joint_angles

        kine = robot.forward_kinematics(result)
        print(kine)

        robot.back()
        time.sleep(5)
        
        
    finally:
        robot.cleanup()

if __name__ == "__main__":
    main() 