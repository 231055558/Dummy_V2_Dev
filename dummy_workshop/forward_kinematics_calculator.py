#!/usr/bin/env python3

import sys
import os
import math
import time
import numpy as np
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "CLI_Tool"))

import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'SimHei', 'Microsoft YaHei']
plt.rcParams['axes.unicode_minus'] = False

class ForwardKinematicsCalculator:
    def __init__(self):
        self.physics_client = None
        self.robot_id = None
        self.joint_indices = []
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.base_position = [0, 0, 0]
        self.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.dh_params = []
        self.link_transforms = []
        self.end_effector_pose = None
        self.gripper_tip_pose = None
        self.visualization_lines = []
        self.visualization_texts = []
    
    def parse_urdf_parameters(self):
        urdf_path = os.path.join(os.path.dirname(os.path.dirname(
            os.path.realpath(__file__))), "dummy2", "dummy2.urdf")
        
        if not os.path.exists(urdf_path):
            return False
        
        tree = ET.parse(urdf_path)
        root = tree.getroot()
        joint_info = {}
        
        for joint in root.findall('joint'):
            joint_name = joint.get('name')
            if joint_name in self.joint_names:
                origin = joint.find('origin')
                if origin is not None:
                    xyz = origin.get('xyz', '0 0 0').split()
                    rpy = origin.get('rpy', '0 0 0').split()
                    translation = [float(x) for x in xyz]
                    rotation = [float(x) for x in rpy]
                else:
                    translation = [0, 0, 0]
                    rotation = [0, 0, 0]
                
                axis = joint.find('axis')
                if axis is not None:
                    axis_xyz = axis.get('xyz', '0 0 1').split()
                    axis_direction = [float(x) for x in axis_xyz]
                else:
                    axis_direction = [0, 0, 1]
                
                joint_info[joint_name] = {
                    'translation': translation,
                    'rotation': rotation,
                    'axis': axis_direction
                }
        
        self.build_dh_parameters(joint_info)
        return True
    
    def build_dh_parameters(self, joint_info):
        self.dh_params = [
            {'a': 0.0,      'alpha': 0.0,        'd': 0.096,    'theta_offset': 0.0},
            {'a': 0.0,      'alpha': math.pi/2,  'd': 0.0,      'theta_offset': 0.0},
            {'a': 0.168,    'alpha': 0.0,        'd': 0.0,      'theta_offset': 0.0},
            {'a': 0.062,    'alpha': math.pi/2,  'd': 0.0,      'theta_offset': 0.0},
            {'a': 0.0,      'alpha': -math.pi/2, 'd': 0.11,     'theta_offset': 0.0},
            {'a': 0.0,      'alpha': 0.0,        'd': 0.074,    'theta_offset': 0.0}
        ]
        
        self.gripper_offset = {
            'translation': [0.0, 0.030, 0.0],
            'rotation': [0.0, 0.0, 0.0]
        }
    
    def calculate_transformation_matrix(self, a, alpha, d, theta):
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)
        
        T = np.array([
            [ct,    -st*ca,  st*sa,   a*ct],
            [st,     ct*ca, -ct*sa,   a*st],
            [0,      sa,     ca,      d],
            [0,      0,      0,       1]
        ])
        
        return T
    
    def forward_kinematics(self, joint_angles_deg):
        joint_angles_rad = [math.radians(angle) for angle in joint_angles_deg]
        T_accumulated = np.eye(4)
        self.link_transforms = [T_accumulated.copy()]
        
        # Joint1: 基座旋转 (绕Z轴) - 修正方向
        T1 = np.array([
            [math.cos(-joint_angles_rad[0]), -math.sin(-joint_angles_rad[0]), 0, 0],
            [math.sin(-joint_angles_rad[0]),  math.cos(-joint_angles_rad[0]), 0, 0],
            [0, 0, 1, 0.096],
            [0, 0, 0, 1]
        ])
        T_accumulated = np.dot(T_accumulated, T1)
        self.link_transforms.append(T_accumulated.copy())
        
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
        self.link_transforms.append(T_accumulated.copy())
        
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
        self.link_transforms.append(T_accumulated.copy())
        
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
        self.link_transforms.append(T_accumulated.copy())
        
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
        self.link_transforms.append(T_accumulated.copy())
        
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
        self.link_transforms.append(T_accumulated.copy())
        
        self.end_effector_pose = self.extract_pose_from_matrix(T_accumulated)
        
        gripper_transform = np.eye(4)
        gripper_transform[:3, 3] = self.gripper_offset['translation']
        T_gripper = np.dot(T_accumulated, gripper_transform)
        self.gripper_tip_pose = self.extract_pose_from_matrix(T_gripper)
        
        return self.end_effector_pose, self.gripper_tip_pose
    
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
    
    def start_pybullet(self):
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")
        
        urdf_path = os.path.join(os.path.dirname(os.path.dirname(
            os.path.realpath(__file__))), "dummy2", "dummy2.urdf")
        
        if not os.path.exists(urdf_path):
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
    
    def clear_visualization(self):
        for line_id in self.visualization_lines:
            p.removeUserDebugItem(line_id)
        self.visualization_lines.clear()
        
        for text_id in self.visualization_texts:
            p.removeUserDebugItem(text_id)
        self.visualization_texts.clear()
    
    def visualize_end_effector_position(self):
        if not self.end_effector_pose:
            return
        
        self.clear_visualization()
        
        pos = self.end_effector_pose['position']
        x, y, z = pos
        
        # 末端位置点（紫色圆圈）
        sphere_size = 0.01
        for i in range(8):
            angle1 = i * math.pi / 4
            angle2 = (i + 1) * math.pi / 4
            
            line_id = p.addUserDebugLine(
                [x + sphere_size * math.cos(angle1), y + sphere_size * math.sin(angle1), z],
                [x + sphere_size * math.cos(angle2), y + sphere_size * math.sin(angle2), z],
                lineColorRGB=[1, 0, 1],
                lineWidth=5,
                lifeTime=0
            )
            self.visualization_lines.append(line_id)
            
            line_id = p.addUserDebugLine(
                [x + sphere_size * math.cos(angle1), y, z + sphere_size * math.sin(angle1)],
                [x + sphere_size * math.cos(angle2), y, z + sphere_size * math.sin(angle2)],
                lineColorRGB=[1, 0, 1],
                lineWidth=5,
                lifeTime=0
            )
            self.visualization_lines.append(line_id)
        
        # 投影线
        z_axis_point = [0, 0, z]
        line_id = p.addUserDebugLine(pos, z_axis_point, lineColorRGB=[0, 0, 1], lineWidth=3, lifeTime=0)
        self.visualization_lines.append(line_id)
        
        xy_plane_point = [x, y, 0]
        line_id = p.addUserDebugLine(pos, xy_plane_point, lineColorRGB=[0.5, 0.5, 0.5], lineWidth=3, lifeTime=0)
        self.visualization_lines.append(line_id)
        
        x_axis_point = [x, 0, 0]
        line_id = p.addUserDebugLine(xy_plane_point, x_axis_point, lineColorRGB=[1, 0, 0], lineWidth=3, lifeTime=0)
        self.visualization_lines.append(line_id)
        
        y_axis_point = [0, y, 0]
        line_id = p.addUserDebugLine(xy_plane_point, y_axis_point, lineColorRGB=[0, 1, 0], lineWidth=3, lifeTime=0)
        self.visualization_lines.append(line_id)
        
        # 坐标标注
        text_id = p.addUserDebugText(f"Z-axis: (0,0,{z:.3f})", [0.05, 0, z], textColorRGB=[0, 0, 1], textSize=1.0, lifeTime=0)
        self.visualization_texts.append(text_id)
        
        text_id = p.addUserDebugText(f"XY-plane: ({x:.3f},{y:.3f},0)", [x, y, 0.05], textColorRGB=[0.5, 0.5, 0.5], textSize=1.0, lifeTime=0)
        self.visualization_texts.append(text_id)
        
        text_id = p.addUserDebugText(f"X-axis: ({x:.3f},0,0)", [x, 0.05, 0], textColorRGB=[1, 0, 0], textSize=1.0, lifeTime=0)
        self.visualization_texts.append(text_id)
        
        text_id = p.addUserDebugText(f"Y-axis: (0,{y:.3f},0)", [0.05, y, 0], textColorRGB=[0, 1, 0], textSize=1.0, lifeTime=0)
        self.visualization_texts.append(text_id)
        
        text_id = p.addUserDebugText(f"End Effector: ({x:.3f},{y:.3f},{z:.3f})", [x + 0.05, y + 0.05, z + 0.05], textColorRGB=[1, 0, 1], textSize=1.2, lifeTime=0)
        self.visualization_texts.append(text_id)
        
        self.add_end_effector_coordinate_frame(pos)
    
    def add_end_effector_coordinate_frame(self, position, size=0.08):
        if not self.end_effector_pose:
            return
        
        R = self.end_effector_pose['rotation_matrix']
        x_axis = R[:, 0] * size
        y_axis = R[:, 1] * size
        z_axis = R[:, 2] * size
        
        line_id = p.addUserDebugLine(position, [position[0] + x_axis[0], position[1] + x_axis[1], position[2] + x_axis[2]], lineColorRGB=[1, 0, 0], lineWidth=5, lifeTime=0)
        self.visualization_lines.append(line_id)
        
        line_id = p.addUserDebugLine(position, [position[0] + y_axis[0], position[1] + y_axis[1], position[2] + y_axis[2]], lineColorRGB=[0, 1, 0], lineWidth=5, lifeTime=0)
        self.visualization_lines.append(line_id)
        
        line_id = p.addUserDebugLine(position, [position[0] + z_axis[0], position[1] + z_axis[1], position[2] + z_axis[2]], lineColorRGB=[0, 0, 1], lineWidth=5, lifeTime=0)
        self.visualization_lines.append(line_id)
        
        text_id = p.addUserDebugText("End Frame", [position[0] + 0.02, position[1] + 0.02, position[2] + size + 0.02], textColorRGB=[0, 0, 0], textSize=1.0, lifeTime=0)
        self.visualization_texts.append(text_id)
    
    def update_robot_pose(self):
        for i, joint_idx in enumerate(self.joint_indices):
            p.resetJointState(self.robot_id, joint_idx, math.radians(self.current_joint_angles[i]))
        
        for _ in range(5):
            p.stepSimulation()
            time.sleep(1/1000)
        
        self.visualize_end_effector_position()
    
    def display_poses(self):
        print("\n" + "="*50)
        print("末端位置和姿态信息")
        print("="*50)
        
        if self.end_effector_pose:
            pos = self.end_effector_pose['position']
            euler = self.end_effector_pose['euler_angles']
            distance = math.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
            
            print("末端法兰:")
            print(f"  位置 (X, Y, Z): ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m")
            print(f"  姿态 (Roll, Pitch, Yaw): ({math.degrees(euler[0]):.2f}°, {math.degrees(euler[1]):.2f}°, {math.degrees(euler[2]):.2f}°)")
            print(f"  到原点距离: {distance:.4f} m")
        
        if self.gripper_tip_pose:
            pos = self.gripper_tip_pose['position']
            euler = self.gripper_tip_pose['euler_angles']
            distance = math.sqrt(pos[0]**2 + pos[1]**2 + pos[2]**2)
            
            print("\n夹爪尖:")
            print(f"  位置 (X, Y, Z): ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}) m")
            print(f"  姿态 (Roll, Pitch, Yaw): ({math.degrees(euler[0]):.2f}°, {math.degrees(euler[1]):.2f}°, {math.degrees(euler[2]):.2f}°)")
            print(f"  到原点距离: {distance:.4f} m")
        
        print("\n当前关节角度:")
        for i, angle in enumerate(self.current_joint_angles):
            print(f"  关节{i+1}: {angle:.2f}°")
    
    def run_interactive_control(self):
        print("\n交互式控制:")
        print("1-6: 选择关节")
        print("w/s: 增加/减少角度 (5度)")
        print("W/S: 增加/减少角度 (1度)")
        print("r: 重置关节")
        print("c: 显示坐标")
        print("q: 退出")
        
        selected_joint = 0 
        
        while True:
            self.forward_kinematics(self.current_joint_angles)
            self.update_robot_pose()
            
            print(f"\n当前选择: joint{selected_joint+1} = {self.current_joint_angles[selected_joint]:.2f}°")
            print("命令: ", end='')
            
            command = input().strip().lower()
            
            if command == 'q':
                break
            elif command in '123456':
                selected_joint = int(command) - 1
            elif command == 'w':
                self.current_joint_angles[selected_joint] += 5.0
                self.current_joint_angles[selected_joint] = max(-180, min(180, self.current_joint_angles[selected_joint]))
            elif command == 's':
                self.current_joint_angles[selected_joint] -= 5.0
                self.current_joint_angles[selected_joint] = max(-180, min(180, self.current_joint_angles[selected_joint]))
            elif command == 'W':
                self.current_joint_angles[selected_joint] += 1.0
                self.current_joint_angles[selected_joint] = max(-180, min(180, self.current_joint_angles[selected_joint]))
            elif command == 'S':
                self.current_joint_angles[selected_joint] -= 1.0
                self.current_joint_angles[selected_joint] = max(-180, min(180, self.current_joint_angles[selected_joint]))
            elif command == 'r':
                self.current_joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            elif command == 'c':
                self.display_poses()
    
    def cleanup(self):
        self.clear_visualization()
        if self.physics_client:
            p.disconnect()

def main():
    calculator = ForwardKinematicsCalculator()
    
    if not calculator.parse_urdf_parameters():
        return
    
    if not calculator.start_pybullet():
        return
    
    print("正向运动学计算器已就绪")
    input("按 Enter 开始...")
    
    calculator.run_interactive_control()
    calculator.cleanup()

if __name__ == "__main__":
    main() 