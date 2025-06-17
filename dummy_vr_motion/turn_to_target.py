import sys
import os
import time
import math
import numpy as np

# 获取项目根目录的绝对路径
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.append(ROOT_DIR)

# 导入并设置CLI_Tool环境
from CLI_Tool.dummy_robot_integrated import setup_environment, DummyRobotIntegrated
setup_environment()

def calculate_angle_to_target(current_pos, target_axis):
    """
    计算当前位置到目标轴的角度
    Args:
        current_pos: 当前末端位置 [x, y, z]
        target_axis: 目标轴在xy平面的坐标 [x, y]
    Returns:
        angle_diff: 需要旋转的角度（弧度）
        is_facing: 是否朝向目标（而不是背对）
    """
    # 计算末端到目标轴的向量（在xy平面上）
    dx = target_axis[0] - current_pos[0]
    dy = target_axis[1] - current_pos[1]
    
    # 计算目标方向角度（相对于x轴）
    target_angle = math.atan2(dy, dx)
    
    # 获取当前朝向角度（相对于x轴）
    current_angle = math.atan2(current_pos[1], current_pos[0])
    
    # 计算角度差
    angle_diff = target_angle - current_angle
    
    # 标准化到 [-pi, pi]
    while angle_diff > math.pi:
        angle_diff -= 2 * math.pi
    while angle_diff < -math.pi:
        angle_diff += 2 * math.pi
    
    # 判断是否朝向目标（角度差的绝对值小于90度）
    is_facing = abs(angle_diff) < math.pi/2
    
    return angle_diff, is_facing

def turn_to_target(robot, target_axis):
    """
    通过调整J1使末端法兰朝向目标轴
    Args:
        robot: DummyRobotIntegrated实例
        target_axis: 目标轴在xy平面的坐标 [x, y]
    Returns:
        bool: 是否成功对准目标
    """
    print(f"开始转向目标轴 [{target_axis[0]:.3f}, {target_axis[1]:.3f}]")
    
    # 获取当前关节角度
    current_angles = robot.current_joint_angles
    
    # 获取当前末端位置
    current_pose = robot.forward_kinematics(current_angles)
    current_pos = current_pose['position']
    
    # 计算需要旋转的角度
    angle_diff, is_facing = calculate_angle_to_target(current_pos, target_axis)
    
    # 如果是背对目标，需要调整角度差
    if not is_facing:
        if angle_diff > 0:
            angle_diff -= math.pi
        else:
            angle_diff += math.pi
    
    # 将弧度转换为度
    angle_diff_deg = math.degrees(angle_diff)
    
    # 计算目标J1角度
    target_j1 = current_angles[0] + angle_diff_deg
    
    # 确保目标角度在限位范围内 (-170~170)
    if target_j1 > 170:
        target_j1 = 170
    elif target_j1 < -170:
        target_j1 = -170
    
    print(f"当前J1角度: {current_angles[0]:.2f}°")
    print(f"目标J1角度: {target_j1:.2f}°")
    
    # 设置搜索步进值
    step = 0.5  # 每次旋转0.5度
    
    # 记录最佳角度和最小误差
    best_angle = current_angles[0]
    min_error = float('inf')
    
    # 根据旋转方向设置搜索范围
    if target_j1 > current_angles[0]:
        search_range = np.arange(current_angles[0], min(target_j1 + step, 170), step)
    else:
        search_range = np.arange(current_angles[0], max(target_j1 - step, -170), -step)
    
    # 开始搜索
    for j1_angle in search_range:
        # 更新J1角度
        new_angles = current_angles.copy()
        new_angles[0] = j1_angle
        
        # 移动机器人
        robot.move_j(new_angles)
        
        # 计算新的位置和误差
        new_pose = robot.forward_kinematics(new_angles)
        new_pos = new_pose['position']
        _, is_facing = calculate_angle_to_target(new_pos, target_axis)
        
        # 计算到目标轴的距离误差
        error = abs((target_axis[0] - new_pos[0])**2 + (target_axis[1] - new_pos[1])**2)
        
        # 如果朝向正确且误差更小，更新最佳角度
        if is_facing and error < min_error:
            min_error = error
            best_angle = j1_angle
            print(f"找到更好的角度: {j1_angle:.2f}°, 误差: {error:.6f}")
    
    # 移动到最佳角度
    final_angles = current_angles.copy()
    final_angles[0] = best_angle
    robot.move_j(final_angles)
    
    print(f"完成转向，最终J1角度: {best_angle:.2f}°")
    return True

def main():
    """示例用法"""
    # 创建机器人控制器实例（选择是否连接真实机械臂）
    robot = DummyRobotIntegrated(connect_real_robot=True)
    time.sleep(5)  # 等待初始化完成
    
    try:
        # 测试用的目标轴（在xy平面上）
        target_axis = [0.3, 0.3]  # 例如：目标轴通过点(0.3, 0.3)
        
        # 执行转向
        turn_to_target(robot, target_axis)

        robot.sync_to_real_robot()
        time.sleep(5)

        robot.back()
        time.sleep(5)
        
    finally:
        robot.cleanup()

if __name__ == "__main__":
    main()