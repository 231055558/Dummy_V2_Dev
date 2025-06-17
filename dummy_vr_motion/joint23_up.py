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

def joint23_updown(robot, target_height):
    """
    通过调整J2和J3关节角度来达到目标高度
    
    Args:
        robot: DummyRobotIntegrated实例
        target_height: 目标高度（末端法兰z轴坐标）
    """
    # 获取当前关节角度
    current_angles = robot.current_joint_angles
    # 获取当前末端位置
    current_pose = robot.forward_kinematics(current_angles)
    current_height = current_pose['position'][2]
    
    print(f"当前高度: {current_height:.3f}")
    print(f"目标高度: {target_height:.3f}")
    
    # 计算总高度差
    height_diff = abs(target_height - current_height)
    
    # 确定运动方向
    if target_height > current_height:
        # 需要上升：J2增大，J3减小
        direction = 1
        print("需要上升")
    else:
        # 需要下降：J2减小，J3增大
        direction = -1
        print("需要下降")
    
    while True:
        # 计算当前到目标的距离
        current_pose = robot.forward_kinematics(current_angles)
        current_height = current_pose['position'][2]
        current_diff = abs(target_height - current_height)
        
        # 自适应步进值计算
        if current_diff > height_diff * 0.5:  # 距离目标还很远（>50%）
            step = 0.2  # 大步进
        elif current_diff > height_diff * 0.2:  # 距离目标中等（20%-50%）
            step = 0.1  # 中等步进
        elif current_diff > height_diff * 0.05:  # 接近目标（5%-20%）
            step = 0.05  # 小步进
        else:  # 非常接近目标（<5%）
            step = 0.01  # 最小步进
        
        # 同时调整J2和J3（注意索引从0开始，所以是1和2）
        current_angles[1] += direction * step  # J2
        current_angles[2] -= direction * step  # J3
        
        # 计算新的位置
        new_pose = robot.forward_kinematics(current_angles)
        new_height = new_pose['position'][2]
        
        print(f"调整后高度: {new_height:.3f}, 步进值: {step:.3f}")
        
        # 检查是否达到目标
        if direction > 0:  # 上升
            if new_height >= target_height:
                print("达到或超过目标高度")
                break
        else:  # 下降
            if new_height <= target_height:
                print("达到或低于目标高度")
                break
        
        # 执行实际的关节运动
        robot.move_j(current_angles)
        
    print(f"最终高度: {new_height:.3f}")
    return current_angles

def main():
    """示例用法"""
    # 创建机器人控制器实例（选择是否连接真实机械臂）
    robot = DummyRobotIntegrated(connect_real_robot=True)

    # 获取当前位置
    result = robot.current_joint_angles
    kine = robot.forward_kinematics(result)
    current_height = kine['position'][2]
    print("当前高度 = " + str(current_height))
    
    # 测试上升10cm
    target_height = current_height + 0.1
    print(f"\n尝试上升到: {target_height:.3f}")
    joint23_updown(robot, target_height)

    robot.sync_to_real_robot()
    
    time.sleep(5)
    
    # 测试回到原始高度
    print(f"\n尝试回到原始高度: {current_height:.3f}")
    joint23_updown(robot, current_height)
    robot.sync_to_real_robot()
    time.sleep(5)
    
    # 清理
    robot.cleanup()

if __name__ == "__main__":
    main()