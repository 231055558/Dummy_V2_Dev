#!/usr/bin/env python3
"""
简化版机械臂虚拟相机点云生成器
思路：获取RGB图像和深度图像，根据深度给每个像素赋予3D位置，颜色来自RGB图像
"""

import sys
import os
import math
import time
import threading
import numpy as np
import cv2

# 添加CLI-Tool路径
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "CLI_Tool"))


import pybullet as p
import pybullet_data
import open3d as o3d


class SimplePointCloudCamera:
    """
    简化版虚拟相机点云生成器
    """
    
    def __init__(self):
        # PyBullet仿真环境相关
        self.physics_client = None
        self.robot_id = None
        self.joint_indices = []
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 机械臂状态
        self.current_joint_angles = [0.0, 60.0, 90.0, 90.0, 0.0, 0.0]  # 使能状态
        
        self.camera_offset = [0.0, 0.06, 0.0]  # 相机偏移
        self.camera_fov = 120.0 
        self.camera_near = 0.01
        self.camera_far = 2.0
        self.image_width = 640
        self.image_height = 480
        
        # 点云参数
        self.downsample = 1  # 每3个像素取一个点
        self.depth_min = 0.05
        self.depth_max = 1.0
        
        # 末端执行器连杆索引
        self.end_effector_link_index = 7
        
        # 控制
        self.running = True
        self.selected_joint = 0
        self.angle_step = 5.0
        
        print("📹 简化版虚拟相机点云生成器")
        print("思路：像素→3D点→Open3D显示")
        print("=" * 50)
    
    def start_pybullet(self):
        """启动PyBullet"""
        print("🔧 启动PyBullet...")
        
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # 加载地面和机械臂
        p.loadURDF("plane.urdf", [0, 0, 0])
        
        urdf_path = os.path.join(os.path.dirname(os.path.dirname(
            os.path.realpath(__file__))), "dummy2", "dummy2.urdf")
        self.robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
        
        # 获取关节信息
        self.get_joint_info()
        
        # 添加一些彩色物体
        self.add_colorful_objects()
        
        print("✅ PyBullet启动成功")
        return True
    
    def get_joint_info(self):
        """获取关节信息"""
        num_joints = p.getNumJoints(self.robot_id)
        self.joint_indices = []
        
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            
            if joint_name in self.joint_names:
                joint_index = self.joint_names.index(joint_name)
                while len(self.joint_indices) <= joint_index:
                    self.joint_indices.append(-1)
                self.joint_indices[joint_index] = i
    
    def add_colorful_objects(self):
        """添加一些彩色物体用于测试"""
        print("🎨 添加彩色测试物体...")
        
        # 红绿蓝黄四个立方体
        colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1], [1, 1, 0, 1]]
        positions = [[0.3, 0.3, 0.05], [0.3, -0.3, 0.05], [-0.3, 0.3, 0.05], [-0.3, -0.3, 0.05]]
        
        for pos, color in zip(positions, colors):
            p.createMultiBody(
                baseMass=0.1,
                baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05]),
                baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.05, 0.05, 0.05], rgbaColor=color),
                basePosition=pos
            )
        
        # 添加一个货架
        shelf_pos = [0.0, 0.5, 0.0]
        shelf_color = [0.8, 0.8, 0.8, 1.0]
        
        # 货架底板
        p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.2, 0.05, 0.01]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.2, 0.05, 0.01], rgbaColor=shelf_color),
            basePosition=[shelf_pos[0], shelf_pos[1], shelf_pos[2] + 0.01]
        )
        
        # 货架上的物品
        item_colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
        item_positions = [
            [shelf_pos[0] - 0.1, shelf_pos[1], shelf_pos[2] + 0.05],
            [shelf_pos[0], shelf_pos[1], shelf_pos[2] + 0.05],
            [shelf_pos[0] + 0.1, shelf_pos[1], shelf_pos[2] + 0.05]
        ]
        
        for pos, color in zip(item_positions, item_colors):
            p.createMultiBody(
                baseMass=0.05,
                baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03]),
                baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03], rgbaColor=color),
                basePosition=pos
            )
    
    def update_robot_pose(self):
        """更新机械臂姿态"""
        for i, joint_idx in enumerate(self.joint_indices):
            if joint_idx != -1:
                angle_rad = math.radians(self.current_joint_angles[i])
                p.resetJointState(self.robot_id, joint_idx, angle_rad)
    
    def get_camera_pose(self):
        """获取相机位置和朝向"""
        # 获取末端法兰位置姿态
        link_state = p.getLinkState(self.robot_id, self.end_effector_link_index, computeForwardKinematics=True)
        flange_position = list(link_state[0])
        flange_orientation = list(link_state[1])
        
        # 计算相机位置
        rotation_matrix = np.array(p.getMatrixFromQuaternion(flange_orientation)).reshape(3, 3)
        camera_offset_global = rotation_matrix @ np.array(self.camera_offset)
        camera_position = np.array(flange_position) + camera_offset_global
        
        # 相机朝向（Y轴正方向为前方）
        forward_direction = rotation_matrix @ np.array([0, 1, 0])
        camera_target = camera_position + forward_direction * 0.3
        
        return camera_position.tolist(), camera_target.tolist()
    
    def capture_rgbd(self):
        """捕获RGB图像和深度图像"""
        camera_position, camera_target = self.get_camera_pose()
        
        # 计算视图矩阵和投影矩阵
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=camera_target,
            cameraUpVector=[0, 0, 1]
        )
        
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=self.camera_fov,
            aspect=1.0,
            nearVal=self.camera_near,
            farVal=self.camera_far
        )
        
        # 渲染图像
        width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
            width=self.image_width,
            height=self.image_height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        
        # 转换数据格式
        rgb_array = np.array(rgb_img, dtype=np.uint8)
        rgb_array = rgb_array[:, :, :3]  # 去掉alpha通道
        
        depth_array = np.array(depth_img).reshape(height, width)
        # 转换为实际深度值
        depth_real = self.camera_far * self.camera_near / (self.camera_far - (self.camera_far - self.camera_near) * depth_array)
        
        return rgb_array, depth_real, camera_position, camera_target
    
    def pixels_to_pointcloud(self, rgb_image, depth_image, camera_position, camera_target):
        """
        将像素转换为点云
        核心思路：每个像素 → 根据深度值 → 3D空间位置 + RGB颜色
        """
        points = []
        colors = []
        
        height, width = depth_image.shape
        
        # 计算相机内参（从FOV推导焦距）
        fov_rad = math.radians(self.camera_fov)
        focal_length = (height / 2.0) / math.tan(fov_rad / 2.0)
        cx, cy = width / 2.0, height / 2.0
        
        # 计算相机坐标系到世界坐标系的变换
        camera_pos = np.array(camera_position)
        target_pos = np.array(camera_target)
        
        # 相机坐标系方向向量
        forward = target_pos - camera_pos
        forward = forward / np.linalg.norm(forward)
        up = np.array([0, 0, 1])
        right = np.cross(forward, up)
        right = right / np.linalg.norm(right)
        up = np.cross(right, forward)
        
        # 旋转矩阵：相机坐标系 → 世界坐标系
        R = np.column_stack([right, -up, forward])
        
        # 遍历像素（下采样）
        for v in range(0, height, self.downsample):
            for u in range(0, width, self.downsample):
                depth = depth_image[v, u]
                
                # 过滤无效深度
                if depth < self.depth_min or depth > self.depth_max:
                    continue
                
                # 像素坐标 → 相机坐标系
                x_cam = (u - cx) * depth / focal_length
                y_cam = (v - cy) * depth / focal_length
                z_cam = depth
                
                # 相机坐标系 → 世界坐标系
                point_cam = np.array([x_cam, y_cam, z_cam])
                point_world = camera_pos + R @ point_cam
                
                # 获取像素颜色
                pixel_color = rgb_image[v, u] / 255.0  # 归一化到[0,1]
                
                points.append(point_world)
                colors.append(pixel_color)
        
        return np.array(points), np.array(colors)
    
    def show_pointcloud(self, points, colors):
        """在Open3D中显示点云（圆形点）"""
        # 创建点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.colors = o3d.utility.Vector3dVector(colors)
        
        # 创建坐标轴
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        
        # 创建可视化器并设置圆形点渲染
        vis = o3d.visualization.Visualizer()
        vis.create_window(window_name="简化版点云（圆形点）", width=800, height=600)
        
        # 添加几何体
        vis.add_geometry(pcd)
        vis.add_geometry(coordinate_frame)
        
        # 获取渲染选项并设置点为圆形
        render_option = vis.get_render_option()
        render_option.point_show_normal = False
        render_option.point_size = 3.0  # 设置点的大小
        render_option.point_color_option = o3d.visualization.PointColorOption.Color
        
        # 设置背景颜色
        render_option.background_color = np.array([0.1, 0.1, 0.1])  # 深灰色背景
        
        print("🎯 点云显示说明:")
        print("   - 鼠标左键拖拽: 旋转视角")
        print("   - 鼠标右键拖拽: 平移视角") 
        print("   - 滚轮: 缩放")
        print("   - 按Q或关闭窗口: 退出显示")
        
        # 运行可视化
        vis.run()
        vis.destroy_window()
    
    def run_once(self):
        """运行一次点云生成和显示"""
        print("🔄 更新机械臂姿态...")
        self.update_robot_pose()
        p.stepSimulation()
        
        print("📷 捕获RGB-D图像...")
        rgb_image, depth_image, camera_position, camera_target = self.capture_rgbd()
        
        print(f"📊 图像信息 - RGB形状: {rgb_image.shape}, 深度范围: {depth_image.min():.3f}-{depth_image.max():.3f}")
        
        print("🔄 转换像素到点云...")
        points, colors = self.pixels_to_pointcloud(rgb_image, depth_image, camera_position, camera_target)
        
        print(f"🎨 点云信息 - 点数: {len(points)}, 相机位置: {[f'{p:.3f}' for p in camera_position]}")
        
        if len(points) > 0:
            print("🎯 显示点云...")
            self.show_pointcloud(points, colors)
        else:
            print("❌ 没有生成有效点云")
    
    def interactive_control(self):
        """交互控制"""
        print("\n💬 交互控制模式")
        print("命令: joint <1-6>, up, down, show, quit")
        
        while self.running:
            try:
                command = input("请输入命令: ").strip().lower()
                
                if command == 'quit':
                    break
                elif command.startswith('joint '):
                    joint_num = int(command.split()[1])
                    if 1 <= joint_num <= 6:
                        self.selected_joint = joint_num - 1
                        print(f"🎯 选中关节 J{joint_num}")
                elif command == 'up':
                    self.current_joint_angles[self.selected_joint] += self.angle_step
                    print(f"⬆️ J{self.selected_joint + 1}: {self.current_joint_angles[self.selected_joint]:.1f}°")
                elif command == 'down':
                    self.current_joint_angles[self.selected_joint] -= self.angle_step
                    print(f"⬇️ J{self.selected_joint + 1}: {self.current_joint_angles[self.selected_joint]:.1f}°")
                elif command == 'show':
                    self.run_once()
                elif command == 'help':
                    print("命令说明:")
                    print("  joint <1-6> - 选择关节")
                    print("  up/down     - 调整关节角度")
                    print("  show        - 生成并显示点云")
                    print("  quit        - 退出")
                else:
                    print("❌ 未知命令，输入 'help' 查看帮助")
                    
            except Exception as e:
                print(f"❌ 命令执行错误: {e}")
    
    def cleanup(self):
        """清理资源"""
        if self.physics_client is not None:
            p.disconnect()

def main():
    print("🚀 启动简化版虚拟相机点云生成器")
    
    camera = SimplePointCloudCamera()
    
    try:
        # 启动PyBullet
        if not camera.start_pybullet():
            return
        
        # 首次演示
        print("\n🎬 首次演示...")
        camera.run_once()
        
        # 交互控制
        camera.interactive_control()
        
    except Exception as e:
        print(f"❌ 程序错误: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        camera.cleanup()
        print("🔚 程序结束")

if __name__ == "__main__":
    main() 