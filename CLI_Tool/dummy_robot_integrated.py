#!/usr/bin/env python3
"""
Dummy Robot 集成控制器
整合了以下功能：
1. PyBullet仿真与真实机械臂的连接和同步
2. 正运动学计算
3. 逆运动学计算
4. 关节空间运动控制
5. 笛卡尔空间运动控制
"""

import sys
import os
import math
import time
import numpy as np
import threading

def setup_environment():
    """设置环境变量和依赖路径"""
    # 获取当前文件所在目录的绝对路径
    current_dir = os.path.dirname(os.path.abspath(__file__))
    # 获取项目根目录
    root_dir = os.path.dirname(current_dir)
    
    # 添加 CLI_Tool 目录到路径中（对于 ref_tool 模块）
    if current_dir not in sys.path:
        sys.path.append(current_dir)
    
    # 添加 fibre 模块路径
    fibre_python_path = os.path.join(current_dir, "fibre")
    if os.path.exists(fibre_python_path) and fibre_python_path not in sys.path:
        sys.path.append(fibre_python_path)
    
    # 添加项目根目录到路径
    if root_dir not in sys.path:
        sys.path.append(root_dir)

def import_dependencies():
    """导入所需的依赖，并处理可能的导入错误"""
    missing_deps = []
    
    try:
        import pybullet as p
        import pybullet_data
    except ImportError:
        missing_deps.append("pybullet")
    
    try:
        from fibre import Logger, Event
    except ImportError:
        missing_deps.append("fibre")
    
    try:
        import ref_tool
    except ImportError:
        missing_deps.append("ref_tool")
    
    if missing_deps:
        print("❌ 缺少以下依赖库:")
        for dep in missing_deps:
            print(f"  - {dep}")
        print("\n请安装必要的依赖:")
        if "pybullet" in missing_deps:
            print("pip install pybullet")
        if "fibre" in missing_deps or "ref_tool" in missing_deps:
            print("请确保 fibre 和 ref_tool 模块在正确的路径下")
        sys.exit(1)
    
    return p, pybullet_data, Logger, Event, ref_tool

# 设置环境
setup_environment()

# 导入依赖
p, pybullet_data, Logger, Event, ref_tool = import_dependencies()

class DummyRobotIntegrated:
    def __init__(self, connect_real_robot=False):
        """
        初始化集成机器人控制器
        Args:
            connect_real_robot: 是否连接真实机械臂
        """
        # 基本属性
        self.physics_client = None
        self.robot_id = None
        self.real_robot_device = None
        self.joint_indices = []
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.initial_angles = [0, -70, 180, 0, 0, 0]  # 真实机械臂初始位置（度）
        self.initial_pose_angles_1 = [0.0, -55.0, 150.0, 0.0, 0.0, 0.0]  # 初始位置姿态
        self.initial_pose_angles_2 = [0.0, -55.0, 150.0, -90.0, 0.0, 0.0]
        self.current_joint_angles = self.initial_angles.copy()
        self.base_position = [0, 0, 0]
        self.visualization_lines = []
        self.visualization_texts = []
        
        # 真实角度到虚拟角度的转换参数
        # 基于测试发现：真实[0, -70, 180, 0, 0, 0] -> 虚拟[0, 70, 90, 0, 0, 0]
        self.real_to_virtual_mapping = [
            {'multiplier': -1, 'offset': 0},      # joint1: 基座旋转 - 方向相反
            {'multiplier': -1, 'offset': 5},      # joint2: 肩部 - 方向相反，偏移+5度
            {'multiplier': 1, 'offset': -90},     # joint3: 肘部 - 方向相同，偏移-90度
            {'multiplier': -1, 'offset': 0},      # joint4: 腕部roll - 方向相反
            {'multiplier': 1, 'offset': 0},       # joint5: 腕部pitch - 方向相同
            {'multiplier': 1, 'offset': 0}        # joint6: 腕部yaw - 方向相同
        ]
        
        # 逆运动学求解参数
        self.max_iterations = 1000
        self.tolerance = 1e-4
        self.step_size = 0.1
        
        print("🤖 Dummy Robot 集成控制器")
        print("=" * 60)
        
        # 初始化PyBullet环境
        if not self.start_pybullet():
            raise Exception("PyBullet环境初始化失败")
        
        # 如果需要，连接真实机械臂
        self.real_robot_connected = False
        if connect_real_robot:
            self.real_robot_connected = self.connect_real_robot()
            if not self.real_robot_connected:
                print("⚠️ 真实机械臂连接失败，将只在仿真环境中运行")
    
    def start_pybullet(self):
        """启动PyBullet仿真环境"""
        try:
            self.physics_client = p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0, 0, -9.81)
            p.loadURDF("plane.urdf")
            
            # 加载机器人URDF
            urdf_path = os.path.join(os.path.dirname(os.path.dirname(
                os.path.realpath(__file__))), "dummy2", "dummy2.urdf")
            
            if not os.path.exists(urdf_path):
                print(f"❌ 找不到URDF文件: {urdf_path}")
                return False
            
            self.robot_id = p.loadURDF(urdf_path, self.base_position, useFixedBase=True)
            
            # 获取关节信息
            num_joints = p.getNumJoints(self.robot_id)
            for i in range(num_joints):
                joint_info = p.getJointInfo(self.robot_id, i)
                joint_name = joint_info[1].decode('utf-8')
                if joint_name in self.joint_names:
                    self.joint_indices.append(i)
            
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
            if result:
                print("✅ 初始化位置成功")
                # 更新虚拟机械臂位置以保持同步
                self.set_virtual_robot_pose(self.initial_angles)
            else:
                print("❌ 初始化位置失败")
            
            return True
            
        except Exception as e:
            print(f"❌ 连接真实机器人失败: {e}")
            return False

    def back(self):
        """
        回到初始位置 [0, -70, 180, 0, 0, 0]
        Returns:
            bool: 是否成功
        """
        print("🏠 移动到初始位置...")
        success = self.move_j(self.initial_angles)
        if success and self.real_robot_connected:
            self.sync_to_real_robot()
        return success

    def go_1(self):
        """
        移动到工作位置 [0.0, -55.0, 150.0, 0.0, 0.0, 0.0]
        Returns:
            bool: 是否成功
        """
        print("🎯 移动到工作位置...")
        success = self.move_j(self.initial_pose_angles_1)
        if success and self.real_robot_connected:
            self.sync_to_real_robot()
        return success

    def go_2(self):
        """
        移动到工作位置 [0.0, -55.0, 150.0, -90, 0.0, 0.0]
        Returns:
            bool: 是否成功
        """
        print("🎯 移动到工作位置...")
        success = self.move_j(self.initial_pose_angles_2)
        if success and self.real_robot_connected:
            self.sync_to_real_robot()
        return success

    
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
    
    def forward_kinematics(self, joint_angles_deg):
        """
        正运动学计算
        Args:
            joint_angles_deg: 真实机械臂的关节角度列表（度）
        Returns:
            end_effector_pose: 末端位姿字典，包含position和euler_angles
        """
        # 首先将真实机械臂角度转换为虚拟环境角度
        virtual_angles = self.real_to_virtual_angles(joint_angles_deg)
        
        joint_angles_rad = [math.radians(angle) for angle in virtual_angles]
        T_accumulated = np.eye(4)
        
        # Joint1: 基座旋转 (绕Z轴) - 修正方向
        T1 = np.array([
            [math.cos(-joint_angles_rad[0]), -math.sin(-joint_angles_rad[0]), 0, 0],
            [math.sin(-joint_angles_rad[0]),  math.cos(-joint_angles_rad[0]), 0, 0],
            [0, 0, 1, 0.096],
            [0, 0, 0, 1]
        ])
        T_accumulated = np.dot(T_accumulated, T1)
        
        # Joint2: 肩部关节 (绕X轴)
        T2_offset = np.array([
            [1, 0, 0, -0.011639],
            [0, 1, 0,  0.034477],
            [0, 0, 1,  0.0285],
            [0, 0, 0, 1]
        ])
        T2_rot = np.array([
            [1, 0, 0, 0],
            [0, math.cos(joint_angles_rad[1]), -math.sin(joint_angles_rad[1]), 0],
            [0, math.sin(joint_angles_rad[1]),  math.cos(joint_angles_rad[1]), 0],
            [0, 0, 0, 1]
        ])
        T2 = np.dot(T2_offset, T2_rot)
        T_accumulated = np.dot(T_accumulated, T2)
        
        # Joint3: 肘关节 (绕X轴，反向)
        T3_offset = np.array([
            [1, 0, 0, 0.03615],
            [0, 1, 0, 0.0],
            [0, 0, 1, 0.168],
            [0, 0, 0, 1]
        ])
        T3_rot = np.array([
            [1, 0, 0, 0],
            [0, math.cos(-joint_angles_rad[2]), -math.sin(-joint_angles_rad[2]), 0],
            [0, math.sin(-joint_angles_rad[2]),  math.cos(-joint_angles_rad[2]), 0],
            [0, 0, 0, 1]
        ])
        T3 = np.dot(T3_offset, T3_rot)
        T_accumulated = np.dot(T_accumulated, T3)
        
        # Joint4: 腕部roll (绕Y轴)
        T4_offset = np.array([
            [1, 0, 0, -0.013162],
            [0, 1, 0,  0.0041],
            [0, 0, 1,  0.062467],
            [0, 0, 0, 1]
        ])
        T4_rot = np.array([
            [math.cos(-joint_angles_rad[3]), 0, math.sin(-joint_angles_rad[3]), 0],
            [0, 1, 0, 0],
            [-math.sin(-joint_angles_rad[3]), 0, math.cos(-joint_angles_rad[3]), 0],
            [0, 0, 0, 1]
        ])
        T4 = np.dot(T4_offset, T4_rot)
        T_accumulated = np.dot(T_accumulated, T4)
        
        # Joint5: 腕部pitch (绕X轴，反向)
        T5_offset = np.array([
            [1, 0, 0, 0.0213],
            [0, 1, 0, 0.11],
            [0, 0, 1, 0.0],
            [0, 0, 0, 1]
        ])
        T5_rot = np.array([
            [1, 0, 0, 0],
            [0, math.cos(-joint_angles_rad[4]), -math.sin(-joint_angles_rad[4]), 0],
            [0, math.sin(-joint_angles_rad[4]),  math.cos(-joint_angles_rad[4]), 0],
            [0, 0, 0, 1]
        ])
        T5 = np.dot(T5_offset, T5_rot)
        T_accumulated = np.dot(T_accumulated, T5)
        
        # Joint6: 腕部yaw (绕Y轴)
        T6_offset = np.array([
            [1, 0, 0, -0.019822],
            [0, 1, 0,  0.1195],
            [0, 0, 1, -0.001226],
            [0, 0, 0, 1]
        ])
        T6_rot = np.array([
            [math.cos(-joint_angles_rad[5]), 0, math.sin(-joint_angles_rad[5]), 0],
            [0, 1, 0, 0],
            [-math.sin(-joint_angles_rad[5]), 0, math.cos(-joint_angles_rad[5]), 0],
            [0, 0, 0, 1]
        ])
        T6 = np.dot(T6_offset, T6_rot)
        T_accumulated = np.dot(T_accumulated, T6)
        
        return self.extract_pose_from_matrix(T_accumulated)
    
    def extract_pose_from_matrix(self, transform_matrix):
        """从变换矩阵中提取位姿信息"""
        position = transform_matrix[:3, 3]
        rotation_matrix = transform_matrix[:3, :3]
        euler_angles = self.rotation_matrix_to_euler(rotation_matrix)
        
        pose = {
            'position': position.tolist(),
            'euler_angles': euler_angles,
            'rotation_matrix': rotation_matrix
        }
        
        return pose
    
    def rotation_matrix_to_euler(self, R):
        """旋转矩阵转欧拉角"""
        sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
        singular = sy < 1e-6
        
        if not singular:
            x = math.atan2(R[2,1], R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else:
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        
        return [x, y, z]
    
    def compute_jacobian(self, joint_angles):
        """计算雅可比矩阵"""
        jacobian = np.zeros((6, 6))
        h = 1e-6
        
        current_pose = self.forward_kinematics(joint_angles)
        current_pos = np.array(current_pose['position'])
        current_euler = np.array(current_pose['euler_angles'])
        
        for i in range(6):
            joint_angles_plus = joint_angles.copy()
            joint_angles_plus[i] += math.degrees(h)
            pose_plus = self.forward_kinematics(joint_angles_plus)
            pos_plus = np.array(pose_plus['position'])
            euler_plus = np.array(pose_plus['euler_angles'])
            
            joint_angles_minus = joint_angles.copy()
            joint_angles_minus[i] -= math.degrees(h)
            pose_minus = self.forward_kinematics(joint_angles_minus)
            pos_minus = np.array(pose_minus['position'])
            euler_minus = np.array(pose_minus['euler_angles'])
            
            jacobian[0:3, i] = (pos_plus - pos_minus) / (2 * h)
            jacobian[3:6, i] = (euler_plus - euler_minus) / (2 * h)
        
        return jacobian
    
    def inverse_kinematics(self, target_position, target_euler_angles):
        """
        逆运动学计算
        Args:
            target_position: 目标位置 [x, y, z]
            target_euler_angles: 目标欧拉角 [roll, pitch, yaw]（弧度）
        Returns:
            (joint_angles, success): (关节角度列表, 是否成功)
        """
        joint_angles = self.current_joint_angles.copy()
        target_pos = np.array(target_position)
        target_euler = np.array(target_euler_angles)
        target_vector = np.concatenate([target_pos, target_euler])
        
        for iteration in range(self.max_iterations):
            current_pose = self.forward_kinematics(joint_angles)
            current_pos = np.array(current_pose['position'])
            current_euler = np.array(current_pose['euler_angles'])
            current_vector = np.concatenate([current_pos, current_euler])
            
            error_vector = target_vector - current_vector
            error_norm = np.linalg.norm(error_vector)
            
            if error_norm < self.tolerance:
                return joint_angles, True
            
            jacobian = self.compute_jacobian(joint_angles)
            
            try:
                jacobian_pinv = np.linalg.pinv(jacobian)
                delta_joints = jacobian_pinv.dot(error_vector) * self.step_size
                
                joint_angles += np.degrees(delta_joints)
                joint_angles = np.clip(joint_angles, -180, 180)
                
            except np.linalg.LinAlgError:
                return joint_angles, False
        
        return joint_angles, False
    
    def move_j(self, target_angles):
        """
        关节空间运动
        Args:
            target_angles: 目标关节角度列表（度）
        Returns:
            bool: 是否成功
        """
        try:
            # 设置虚拟机器人姿态
            self.set_virtual_robot_pose(target_angles)
            return True
        except Exception as e:
            print(f"❌ 关节运动失败: {e}")
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
        
        # 步进仿真
        for _ in range(10):
            p.stepSimulation()
            time.sleep(1/1000)
    
    def move_p(self, target_position, target_euler_angles):
        """
        笛卡尔空间运动
        Args:
            target_position: 目标位置 [x, y, z]
            target_euler_angles: 目标欧拉角 [roll, pitch, yaw]（弧度）
        Returns:
            bool: 是否成功
        """
        # 计算逆运动学
        joint_angles, success = self.inverse_kinematics(target_position, target_euler_angles)
        
        if not success:
            print("❌ 逆运动学求解失败")
            return False
        
        # 执行关节运动
        return self.move_j(joint_angles)
    
    def sync_to_real_robot(self):
        """同步仿真状态到真实机械臂"""
        if not self.real_robot_connected:
            print("⚠️ 未连接真实机械臂")
            return False
            
        print(f"🔄 同步关节角度到真实机器人: {[f'{a:.1f}°' for a in self.current_joint_angles]}")
        
        try:
            result = self.real_robot_device.robot.move_j(*self.current_joint_angles)
            if result:
                print("✅ 同步到真实机械臂成功")
            else:
                print("❌ 同步到真实机械臂失败")
            return result
        except Exception as e:
            print(f"❌ 同步失败: {e}")
            return False
    
    def cleanup(self):
        """清理资源"""
        if self.real_robot_connected and self.real_robot_device:
            try:
                self.real_robot_device.robot.set_enable(False)
                print("✅ 真实机械臂已安全停止")
            except:
                pass
        
        if self.physics_client:
            p.disconnect()
            print("✅ PyBullet环境已关闭")
    
    def verify_angle_mapping(self, real_angles):
        """
        验证真实角度和虚拟角度的映射关系
        Args:
            real_angles: 真实机械臂的关节角度列表
        """
        virtual_angles = self.real_to_virtual_angles(real_angles)
        real_angles_back = self.virtual_to_real_angles(virtual_angles)
        
        print("\n角度映射验证:")
        print("="*50)
        print("关节  真实角度  ->  虚拟角度  ->  转换回真实角度")
        print("-"*50)
        for i in range(6):
            print(f"J{i+1}:  {real_angles[i]:8.2f}  ->  {virtual_angles[i]:8.2f}  ->  {real_angles_back[i]:8.2f}")
        print("="*50)
        
        # 验证转换精度
        conversion_error = np.array(real_angles) - np.array(real_angles_back)
        if np.max(np.abs(conversion_error)) < 1e-10:
            print("✅ 角度映射验证通过！转换精度在误差范围内")
        else:
            print("⚠️ 角度映射可能存在问题，最大误差: {:.2e}".format(np.max(np.abs(conversion_error))))

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