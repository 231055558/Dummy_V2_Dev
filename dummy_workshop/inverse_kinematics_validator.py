#!/usr/bin/env python3

import sys
import os
import math
import time
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "CLI_Tool"))

import pybullet as p
import pybullet_data

class InverseKinematicsValidator:
    def __init__(self):
        self.physics_client = None
        self.robot_id = None
        self.joint_indices = []
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.base_position = [0, 0, 0]
        self.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.visualization_lines = []
        self.visualization_texts = []
        
        # 逆运动学求解参数
        self.max_iterations = 1000
        self.tolerance = 1e-4
        self.step_size = 0.1
        
    def forward_kinematics(self, joint_angles_deg):
        """基于原始程序的正向运动学计算"""
        joint_angles_rad = [math.radians(angle) for angle in joint_angles_deg]
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
    
    def euler_to_rotation_matrix(self, euler_angles):
        roll, pitch, yaw = euler_angles
        
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll), math.cos(roll)]
        ])
        
        R_y = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])
        
        R_z = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw), math.cos(yaw), 0],
            [0, 0, 1]
        ])
        
        return np.dot(R_z, np.dot(R_y, R_x))
    
    def compute_jacobian(self, joint_angles):
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
    
    def inverse_kinematics(self, target_position, target_euler_angles, initial_guess=None):
        if initial_guess is None:
            joint_angles = self.current_joint_angles.copy()
        else:
            joint_angles = initial_guess.copy()
        
        target_pos = np.array(target_position)
        target_euler = np.array(target_euler_angles)
        target_vector = np.concatenate([target_pos, target_euler])
        
        print(f"逆运动学求解中...")
        print(f"目标位置: {target_pos}")
        print(f"目标姿态 (度): {np.degrees(target_euler)}")
        
        for iteration in range(self.max_iterations):
            current_pose = self.forward_kinematics(joint_angles)
            current_pos = np.array(current_pose['position'])
            current_euler = np.array(current_pose['euler_angles'])
            current_vector = np.concatenate([current_pos, current_euler])
            
            error_vector = target_vector - current_vector
            error_norm = np.linalg.norm(error_vector)
            
            if error_norm < self.tolerance:
                print(f"收敛成功！迭代次数: {iteration}")
                print(f"最终误差: {error_norm:.6f}")
                print(f"计算得到的关节角度: {joint_angles}")
                return joint_angles, True
            
            jacobian = self.compute_jacobian(joint_angles)
            
            try:
                jacobian_pinv = np.linalg.pinv(jacobian)
                delta_joints = jacobian_pinv.dot(error_vector) * self.step_size
                
                joint_angles += np.degrees(delta_joints)
                joint_angles = np.clip(joint_angles, -180, 180)
                
            except np.linalg.LinAlgError:
                print("雅可比矩阵奇异，无法求解")
                return joint_angles, False
            
            if iteration % 100 == 0:
                print(f"迭代 {iteration}: 误差 = {error_norm:.6f}")
        
        print(f"未收敛，最大迭代次数达到。最终误差: {error_norm:.6f}")
        return joint_angles, False
    
    def start_pybullet(self):
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        
        urdf_path = os.path.join(os.path.dirname(os.path.dirname(
            os.path.realpath(__file__))), "dummy2", "dummy2.urdf")
        
        if not os.path.exists(urdf_path):
            print(f"URDF文件未找到: {urdf_path}")
            return False
            
        self.robot_id = p.loadURDF(urdf_path, self.base_position, useFixedBase=True)
        
        num_joints = p.getNumJoints(self.robot_id)
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            if joint_name in self.joint_names:
                self.joint_indices.append(i)
        
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.3]
        )
        
        self.add_coordinate_frame([0, 0, 0], "World Frame")
        return True
    
    def add_coordinate_frame(self, position, label="", size=0.1):
        p.addUserDebugLine(
            position, 
            [position[0] + size, position[1], position[2]], 
            lineColorRGB=[1, 0, 0], 
            lineWidth=3,
            lifeTime=0
        )
        
        p.addUserDebugLine(
            position, 
            [position[0], position[1] + size, position[2]], 
            lineColorRGB=[0, 1, 0], 
            lineWidth=3,
            lifeTime=0
        )
        
        p.addUserDebugLine(
            position, 
            [position[0], position[1], position[2] + size], 
            lineColorRGB=[0, 0, 1], 
            lineWidth=3,
            lifeTime=0
        )
        
        if label:
            p.addUserDebugText(
                label, 
                [position[0], position[1], position[2] + size + 0.02],
                textColorRGB=[0, 0, 0],
                textSize=1.2,
                lifeTime=0
            )
    
    def visualize_target_pose(self, position, euler_angles):
        self.clear_visualization()
        
        # 目标位置标记（红色球体）
        sphere_size = 0.015
        for i in range(12):
            angle1 = i * math.pi / 6
            angle2 = (i + 1) * math.pi / 6
            
            line_id = p.addUserDebugLine(
                [position[0] + sphere_size * math.cos(angle1), 
                 position[1] + sphere_size * math.sin(angle1), 
                 position[2]],
                [position[0] + sphere_size * math.cos(angle2), 
                 position[1] + sphere_size * math.sin(angle2), 
                 position[2]],
                lineColorRGB=[1, 0, 0],
                lineWidth=6,
                lifeTime=0
            )
            self.visualization_lines.append(line_id)
        
        # 目标姿态坐标系
        R = self.euler_to_rotation_matrix(euler_angles)
        frame_size = 0.1
        
        x_axis = position + R[:, 0] * frame_size
        line_id = p.addUserDebugLine(position, x_axis, lineColorRGB=[1, 0.5, 0.5], lineWidth=8, lifeTime=0)
        self.visualization_lines.append(line_id)
        
        y_axis = position + R[:, 1] * frame_size
        line_id = p.addUserDebugLine(position, y_axis, lineColorRGB=[0.5, 1, 0.5], lineWidth=8, lifeTime=0)
        self.visualization_lines.append(line_id)
        
        z_axis = position + R[:, 2] * frame_size
        line_id = p.addUserDebugLine(position, z_axis, lineColorRGB=[0.5, 0.5, 1], lineWidth=8, lifeTime=0)
        self.visualization_lines.append(line_id)
        
        text_id = p.addUserDebugText(
            f"TARGET POSE\nPos: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})\nRPY: ({math.degrees(euler_angles[0]):.1f}, {math.degrees(euler_angles[1]):.1f}, {math.degrees(euler_angles[2]):.1f})", 
            [position[0] + 0.05, position[1] + 0.05, position[2] + 0.15],
            textColorRGB=[1, 0, 0],
            textSize=1.5,
            lifeTime=0
        )
        self.visualization_texts.append(text_id)
    
    def visualize_actual_pose(self, pose):
        position = pose['position']
        
        # 实际位置标记（绿色球体）
        sphere_size = 0.012
        for i in range(8):
            angle1 = i * math.pi / 4
            angle2 = (i + 1) * math.pi / 4
            
            line_id = p.addUserDebugLine(
                [position[0] + sphere_size * math.cos(angle1), 
                 position[1] + sphere_size * math.sin(angle1), 
                 position[2]],
                [position[0] + sphere_size * math.cos(angle2), 
                 position[1] + sphere_size * math.sin(angle2), 
                 position[2]],
                lineColorRGB=[0, 1, 0],
                lineWidth=5,
                lifeTime=0
            )
            self.visualization_lines.append(line_id)
        
        # 实际姿态坐标系
        R = pose['rotation_matrix']
        frame_size = 0.08
        
        x_axis = np.array(position) + R[:, 0] * frame_size
        line_id = p.addUserDebugLine(position, x_axis, lineColorRGB=[1, 0, 0], lineWidth=5, lifeTime=0)
        self.visualization_lines.append(line_id)
        
        y_axis = np.array(position) + R[:, 1] * frame_size
        line_id = p.addUserDebugLine(position, y_axis, lineColorRGB=[0, 1, 0], lineWidth=5, lifeTime=0)
        self.visualization_lines.append(line_id)
        
        z_axis = np.array(position) + R[:, 2] * frame_size
        line_id = p.addUserDebugLine(position, z_axis, lineColorRGB=[0, 0, 1], lineWidth=5, lifeTime=0)
        self.visualization_lines.append(line_id)
        
        euler = pose['euler_angles']
        text_id = p.addUserDebugText(
            f"ACTUAL POSE\nPos: ({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})\nRPY: ({math.degrees(euler[0]):.1f}, {math.degrees(euler[1]):.1f}, {math.degrees(euler[2]):.1f})", 
            [position[0] - 0.15, position[1] + 0.05, position[2] + 0.15],
            textColorRGB=[0, 1, 0],
            textSize=1.5,
            lifeTime=0
        )
        self.visualization_texts.append(text_id)
    
    def clear_visualization(self):
        for line_id in self.visualization_lines:
            p.removeUserDebugItem(line_id)
        self.visualization_lines.clear()
        
        for text_id in self.visualization_texts:
            p.removeUserDebugItem(text_id)
        self.visualization_texts.clear()
    
    def update_robot_pose(self, joint_angles):
        for i, joint_idx in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, joint_idx, math.radians(joint_angles[i]))
        
        for _ in range(10):
            p.stepSimulation()
            time.sleep(1/1000)
    
    def compare_poses(self, target_pos, target_euler, actual_pose):
        actual_pos = actual_pose['position']
        actual_euler = actual_pose['euler_angles']
        
        pos_error = np.linalg.norm(np.array(target_pos) - np.array(actual_pos))
        euler_error = np.linalg.norm(np.array(target_euler) - np.array(actual_euler))
        
        print("\n" + "="*60)
        print("位姿对比结果")
        print("="*60)
        print(f"目标位置: ({target_pos[0]:.4f}, {target_pos[1]:.4f}, {target_pos[2]:.4f})")
        print(f"实际位置: ({actual_pos[0]:.4f}, {actual_pos[1]:.4f}, {actual_pos[2]:.4f})")
        print(f"位置误差: {pos_error:.4f} m")
        print()
        print(f"目标姿态 (度): ({math.degrees(target_euler[0]):.2f}, {math.degrees(target_euler[1]):.2f}, {math.degrees(target_euler[2]):.2f})")
        print(f"实际姿态 (度): ({math.degrees(actual_euler[0]):.2f}, {math.degrees(actual_euler[1]):.2f}, {math.degrees(actual_euler[2]):.2f})")
        print(f"姿态误差: {math.degrees(euler_error):.2f} 度")
        print("="*60)
        
        return pos_error, euler_error
    
    def run_test(self, target_position, target_euler_angles):
        print(f"\n开始逆运动学验证测试...")
        print(f"目标位置: {target_position}")
        print(f"目标姿态 (弧度): {target_euler_angles}")
        print(f"目标姿态 (度): {[math.degrees(angle) for angle in target_euler_angles]}")
        
        # 可视化目标位姿
        self.visualize_target_pose(target_position, target_euler_angles)
        
        # 逆运动学求解
        joint_angles, success = self.inverse_kinematics(target_position, target_euler_angles)
        
        if success:
            print(f"\n逆运动学求解成功！")
            print(f"计算得到的关节角度: {joint_angles}")
            
            # 更新机器人位姿
            self.update_robot_pose(joint_angles)
            self.current_joint_angles = joint_angles
            
            # 计算实际位姿
            actual_pose = self.forward_kinematics(joint_angles)
            
            # 可视化实际位姿
            self.visualize_actual_pose(actual_pose)
            
            # 对比结果
            pos_error, euler_error = self.compare_poses(target_position, target_euler_angles, actual_pose)
            
            # 判断精度
            if pos_error < 0.01 and euler_error < math.radians(5):
                print("\n✅ 验证成功！位姿匹配精度良好")
            else:
                print("\n⚠️  验证完成，但精度可能需要改进")
            
            return True
        else:
            print("\n❌ 逆运动学求解失败")
            return False
    
    def interactive_test(self):
        while True:
            print("\n" + "="*50)
            print("逆运动学验证器 - 交互式测试")
            print("="*50)
            print("1. 输入目标位姿进行测试")
            print("2. 使用预设测试案例")
            print("3. 退出")
            
            choice = input("请选择 (1-3): ").strip()
            
            if choice == '1':
                try:
                    print("\n请输入目标位姿:")
                    x = float(input("X坐标 (m): "))
                    y = float(input("Y坐标 (m): "))
                    z = float(input("Z坐标 (m): "))
                    
                    roll = math.radians(float(input("Roll角度 (度): ")))
                    pitch = math.radians(float(input("Pitch角度 (度): ")))
                    yaw = math.radians(float(input("Yaw角度 (度): ")))
                    
                    self.run_test([x, y, z], [roll, pitch, yaw])
                    
                except ValueError:
                    print("输入格式错误，请输入数字")
                    
            elif choice == '2':
                test_cases = [
                    ([0.2, 0.1, 0.3], [0, 0, 0]),
                    ([0.15, -0.15, 0.25], [math.radians(30), 0, math.radians(45)]),
                    ([0.0, 0.2, 0.4], [0, math.radians(-45), 0]),
                    ([-0.1, 0.15, 0.35], [math.radians(15), math.radians(30), math.radians(-30)])
                ]
                
                print("\n预设测试案例:")
                for i, (pos, euler) in enumerate(test_cases):
                    print(f"{i+1}. 位置: {pos}, 姿态(度): {[math.degrees(a) for a in euler]}")
                
                try:
                    case_idx = int(input("选择测试案例 (1-4): ")) - 1
                    if 0 <= case_idx < len(test_cases):
                        pos, euler = test_cases[case_idx]
                        self.run_test(pos, euler)
                    else:
                        print("无效选择")
                except ValueError:
                    print("输入格式错误")
                    
            elif choice == '3':
                break
            else:
                print("无效选择，请重新输入")
            
            input("\n按 Enter 继续...")
    
    def cleanup(self):
        self.clear_visualization()
        if self.physics_client:
            p.disconnect()

def main():
    validator = InverseKinematicsValidator()
    
    if not validator.start_pybullet():
        print("启动PyBullet失败")
        return
    
    print("逆运动学验证器已启动")
    print("红色坐标系和球体表示目标位姿")
    print("绿色坐标系和球体表示实际位姿")
    print("请观察两者是否重合来验证精度")
    
    validator.interactive_test()
    validator.cleanup()

if __name__ == "__main__":
    main() 