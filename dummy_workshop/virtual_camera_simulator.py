#!/usr/bin/env python3
"""
机械臂虚拟相机模拟器（集成使能过程+点云生成版本）
目的：
- 从未使能状态平滑过渡到使能状态
- 在机械臂末端法兰y轴方向（绿色轴）安装虚拟相机
- 显示相机视角画面
- 使用Open3D生成实时点云
- 支持终端交互控制
"""

import sys
import os
import math
import time
import threading
import numpy as np
import cv2

# 添加CLI-Tool路径
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "CLI-Tool"))

try:
    import pybullet as p
    import pybullet_data
    import open3d as o3d
except ImportError as e:
    print(f"❌ 缺少依赖库: {e}")
    print("请安装: pip install pybullet opencv-python open3d")
    sys.exit(1)

class VirtualCameraWithPointCloud:
    """
    集成使能过程和点云生成的虚拟相机模拟器
    """
    
    def __init__(self):
        # PyBullet仿真环境相关
        self.physics_client = None
        self.robot_id = None
        self.joint_indices = []
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 机械臂状态定义
        self.disabled_state = [0.0, 70.0, 90.0, 0.0, 0.0, 0.0]  # 未使能状态
        self.ready_state = [0.0, 60.0, 60.0, 90.0, 0.0, 0.0]    # 使能预备状态
        self.current_joint_angles = self.disabled_state.copy()
        
        # 使能参数
        self.enable_duration = 3.0
        self.enable_steps = 120
        self.is_enabled = False
        
        # 相机相关参数（调整到y轴方向，更靠近）
        self.camera_offset = [0.0, 0.06, 0.0]  # 相机相对于末端法兰的偏移（y轴前方6cm）
        self.camera_fov = 120.0      # 视野角度（度）
        self.camera_aspect = 1.0    # 画面宽高比
        self.camera_near = 0.01     # 近截面距离
        self.camera_far = 2.0       # 远截面距离
        self.image_width = 640      # 图像宽度
        self.image_height = 480     # 图像高度
        
        # 点云相关参数
        self.point_cloud_enabled = True  # 是否启用点云生成
        self.point_cloud_downsample = 5  # 点云下采样倍数（每n个像素取一个点）
        self.depth_threshold_min = 0.01  # 深度最小阈值
        self.depth_threshold_max = 1.5   # 深度最大阈值
        self.pointcloud_update_interval = 10  # 点云更新间隔（帧数）
        
        # 控制参数
        self.angle_step = 5.0       # 角度调整步长（度）
        self.fov_step = 5.0         # FOV调整步长（度）
        self.selected_joint = 0     # 当前选中的关节
        
        # 末端执行器连杆索引
        self.end_effector_link_index = 7  # link6_1在URDF中的索引
        
        # 运行控制
        self.running = True
        self.camera_running = False
        self.pointcloud_running = False
        
        # Open3D可视化器
        self.vis = None
        self.point_cloud = None
        self.coordinate_frame = None
        
        print("📹 机械臂虚拟相机模拟器（集成使能过程+点云生成版本）")
        print("=" * 60)
        print("相机安装位置：末端法兰y轴方向（绿色轴前方6cm）")
        print("视线方向：与法兰盘朝向平行")
        print("初始视野角度：120°")
        print("点云生成：像素→3D点→Open3D显示（简化思路）")
        print("=" * 60)
    
    def start_pybullet(self):
        """启动PyBullet仿真环境"""
        # 连接到PyBullet GUI
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(enableFileCaching=0)
        
        # 加载地面
        p.loadURDF("plane.urdf", [0, 0, 0])
        
        # 加载机械臂
        urdf_path = os.path.join(os.path.dirname(os.path.dirname(
            os.path.realpath(__file__))), "dummy2", "dummy2.urdf")
        
        if not os.path.exists(urdf_path):
            print(f"❌ 找不到URDF文件: {urdf_path}")
            return False
        
        self.robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
        
        # 获取关节信息
        self.get_joint_info()
        
        # 设置相机视角
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.5]
        )
        
        # 添加测试物体
        self.add_test_objects()
        
        return True
    
    def get_joint_info(self):
        """获取机械臂关节信息"""
        num_joints = p.getNumJoints(self.robot_id)
        self.joint_indices = []
        
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_name = joint_info[1].decode('utf-8')
            
            if joint_name in self.joint_names:
                joint_index = self.joint_names.index(joint_name)
                if joint_index < len(self.joint_indices):
                    self.joint_indices[joint_index] = i
                else:
                    while len(self.joint_indices) <= joint_index:
                        self.joint_indices.append(-1)
                    self.joint_indices[joint_index] = i
    
    def add_test_objects(self):
        """在环境中添加测试物体"""
        # 在机械臂前方添加货架结构
        self.add_shelf_structure()
        
        # 添加一些额外的测试物体
        colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
        positions = [[0.4, 0.4, 0.02], [0.4, -0.4, 0.02], [-0.4, 0.0, 0.02]]
        
        for i, (pos, color) in enumerate(zip(positions, colors)):
            p.createMultiBody(
                baseMass=0.1,
                baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02]),
                baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02], rgbaColor=color),
                basePosition=pos
            )
    
    def add_shelf_structure(self):
        """在机械臂前方添加货架结构"""
        # 货架位置：在机械臂前方y轴正方向，距离约50cm
        shelf_base_pos = [0.0, 0.4, 0.0]
        
        # 货架颜色：浅灰色
        shelf_color = [0.8, 0.8, 0.8, 1.0]
        
        # 货架底座
        shelf_base = p.createMultiBody(
            baseMass=0,  # 静态物体
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.15, 0.05, 0.01]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.15, 0.05, 0.01], rgbaColor=shelf_color),
            basePosition=[shelf_base_pos[0], shelf_base_pos[1], shelf_base_pos[2] + 0.01]
        )
        
        # 货架左立柱
        left_column = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.01, 0.2]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.01, 0.2], rgbaColor=shelf_color),
            basePosition=[shelf_base_pos[0] - 0.14, shelf_base_pos[1], shelf_base_pos[2] + 0.2]
        )
        
        # 货架右立柱
        right_column = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.01, 0.01, 0.2]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.01, 0.01, 0.2], rgbaColor=shelf_color),
            basePosition=[shelf_base_pos[0] + 0.14, shelf_base_pos[1], shelf_base_pos[2] + 0.2]
        )
        
        # 货架第一层板
        shelf_1 = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.14, 0.04, 0.005]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.14, 0.04, 0.005], rgbaColor=shelf_color),
            basePosition=[shelf_base_pos[0], shelf_base_pos[1], shelf_base_pos[2] + 0.15]
        )
        
        # 货架第二层板
        shelf_2 = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.14, 0.04, 0.005]),
            baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.14, 0.04, 0.005], rgbaColor=shelf_color),
            basePosition=[shelf_base_pos[0], shelf_base_pos[1], shelf_base_pos[2] + 0.3]
        )
        
        # 在货架上放置一些物品
        # 第一层物品
        item_colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
        item_positions = [
            [shelf_base_pos[0] - 0.08, shelf_base_pos[1], shelf_base_pos[2] + 0.17],
            [shelf_base_pos[0], shelf_base_pos[1], shelf_base_pos[2] + 0.17],
            [shelf_base_pos[0] + 0.08, shelf_base_pos[1], shelf_base_pos[2] + 0.17]
        ]
        
        for i, (pos, color) in enumerate(zip(item_positions, item_colors)):
            p.createMultiBody(
                baseMass=0.05,
                baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02]),
                baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02], rgbaColor=color),
                basePosition=pos
            )
        
        # 第二层物品
        item_positions_2 = [
            [shelf_base_pos[0] - 0.06, shelf_base_pos[1], shelf_base_pos[2] + 0.32],
            [shelf_base_pos[0] + 0.06, shelf_base_pos[1], shelf_base_pos[2] + 0.32]
        ]
        
        for i, pos in enumerate(item_positions_2):
            p.createMultiBody(
                baseMass=0.05,
                baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_CYLINDER, radius=0.02, height=0.04),
                baseVisualShapeIndex=p.createVisualShape(p.GEOM_CYLINDER, radius=0.02, length=0.04, rgbaColor=[1, 1, 0, 1]),
                basePosition=pos
            )
        
        print("✅ 货架结构添加完成")
        print(f"   货架位置: {shelf_base_pos}")
        print(f"   货架尺寸: 30cm×10cm×40cm")
        print(f"   距离机械臂: 50cm")
    
    def set_robot_to_disabled_state(self):
        """设置机械臂到未使能状态"""
        for i, joint_idx in enumerate(self.joint_indices):
            if joint_idx != -1:
                p.resetJointState(self.robot_id, joint_idx, math.radians(self.disabled_state[i]))
        
        for _ in range(10):
            p.stepSimulation()
            time.sleep(0.001)
    
    def smooth_step(self, t):
        """S曲线插值"""
        return t * t * (3.0 - 2.0 * t)
    
    def interpolate_joint_angles(self, start_angles, end_angles, progress):
        """平滑插值关节角度"""
        smooth_progress = self.smooth_step(progress)
        return [start + (end - start) * smooth_progress 
                for start, end in zip(start_angles, end_angles)]
    
    def execute_enable_process(self):
        """执行使能过程"""
        print(f"🔄 使能过程: {self.disabled_state} → {self.ready_state}")
        
        start_time = time.time()
        step_duration = self.enable_duration / self.enable_steps
        
        for step in range(self.enable_steps + 1):
            if not self.running:
                break
                
            progress = step / self.enable_steps
            
            # 插值计算关节角度
            self.current_joint_angles = self.interpolate_joint_angles(
                self.disabled_state, self.ready_state, progress
            )
            
            # 更新机械臂姿态
            self.update_robot_pose()
            p.stepSimulation()
            
            # 显示进度
            if step % 30 == 0 or step == self.enable_steps:
                print(f"进度: {progress*100:5.1f}% - {[f'{a:.1f}°' for a in self.current_joint_angles]}")
            
            time.sleep(step_duration)
        
        self.is_enabled = True
        print(f"✅ 使能完成，用时: {time.time() - start_time:.2f}秒")
    
    def update_robot_pose(self):
        """更新机械臂的关节角度"""
        for i, joint_idx in enumerate(self.joint_indices):
            if joint_idx != -1:
                angle_rad = math.radians(self.current_joint_angles[i])
                p.resetJointState(self.robot_id, joint_idx, angle_rad)
    
    def get_end_effector_pose(self):
        """获取末端法兰的位置和姿态"""
        link_state = p.getLinkState(self.robot_id, self.end_effector_link_index, computeForwardKinematics=True)
        position = list(link_state[0])
        orientation = list(link_state[1])  # 四元数 [x, y, z, w]
        return position, orientation
    
    def calculate_camera_pose(self, flange_position, flange_orientation):
        """基于末端法兰位置和姿态，计算相机位置和姿态"""
        # 将四元数转换为旋转矩阵
        rotation_matrix = np.array(p.getMatrixFromQuaternion(flange_orientation)).reshape(3, 3)
        
        # 相机相对于法兰的偏移向量（y轴前方）
        camera_offset_local = np.array(self.camera_offset)
        
        # 将局部偏移转换到全局坐标系
        camera_offset_global = rotation_matrix @ camera_offset_local
        
        # 计算相机位置
        camera_position = np.array(flange_position) + camera_offset_global
        
        # 相机姿态与法兰姿态相同
        camera_orientation = flange_orientation
        
        # 计算相机朝向的目标点（法兰朝向前方0.02米处）
        forward_direction = rotation_matrix @ np.array([0, 1, 0])  # Y轴正方向为前方
        camera_target = camera_position + forward_direction * 0.02
        
        return camera_position.tolist(), camera_orientation, camera_target.tolist()
    

    def capture_camera_image_with_depth(self, camera_position, camera_target, camera_up=[0, 0, 1]):
        """从指定位置和朝向捕获相机图像和深度图"""
        # 计算视图矩阵
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=camera_target,
            cameraUpVector=camera_up
        )
        
        # 计算投影矩阵
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=self.camera_fov,
            aspect=self.camera_aspect,
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
        
        # 转换RGB图像 - 保持RGB格式用于点云
        # PyBullet已经返回了正确形状的数组 (height, width, 4)
        rgb_array = np.array(rgb_img, dtype=np.uint8)
        rgb_array = rgb_array[:, :, :3]  # 去掉alpha通道，保持RGB顺序
        
        # 为2D显示创建BGR格式
        bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
        
        # 转换深度图像
        depth_array = np.array(depth_img).reshape(height, width)
        
        # 将深度缓冲区值转换为实际深度（米）
        # PyBullet深度缓冲区使用非线性深度，需要转换
        depth_real = self.camera_far * self.camera_near / (self.camera_far - (self.camera_far - self.camera_near) * depth_array)
        
        return bgr_array, rgb_array, depth_real, view_matrix, projection_matrix
    
    def depth_image_to_point_cloud(self, rgb_image, depth_image, camera_position, camera_target, camera_up=[0, 0, 1]):
        """
        将深度图像转换为点云
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
        
        # 下采样以提高性能（使用类成员变量）
        step = self.point_cloud_downsample
        
        # 遍历像素（下采样）
        for v in range(0, height, step):
            for u in range(0, width, step):
                depth = depth_image[v, u]
                
                # 严格的深度值检查
                if (not np.isfinite(depth) or 
                    depth < self.depth_threshold_min or 
                    depth > self.depth_threshold_max or
                    depth <= 0):
                    continue
                
                # 像素坐标 → 相机坐标系
                x_cam = (u - cx) * depth / focal_length
                y_cam = (v - cy) * depth / focal_length
                z_cam = depth
                
                # 相机坐标系 → 世界坐标系
                point_cam = np.array([x_cam, y_cam, z_cam])
                point_world = camera_pos + R @ point_cam
                
                # 检查生成的3D点是否有效
                if not np.all(np.isfinite(point_world)):
                    continue
                
                # 获取像素颜色（RGB格式，确保在有效范围内）
                pixel_color = rgb_image[v, u].astype(np.float32) / 255.0
                pixel_color = np.clip(pixel_color, 0.0, 1.0)  # 确保在[0,1]范围内
                
                # 检查颜色是否有效
                if not np.all(np.isfinite(pixel_color)):
                    continue
                
                points.append(point_world)
                colors.append(pixel_color)
        
        if len(points) == 0:
            return np.empty((0, 3)), np.empty((0, 3))
        
        points_array = np.array(points)
        colors_array = np.array(colors)
        
        # 最终数据验证
        valid_indices = np.all(np.isfinite(points_array), axis=1) & np.all(np.isfinite(colors_array), axis=1)
        
        return points_array[valid_indices], colors_array[valid_indices]
    
    def capture_camera_image(self, camera_position, camera_target, camera_up=[0, 0, 1]):
        """从指定位置和朝向捕获相机图像（仅RGB，用于2D显示）"""
        # 计算视图矩阵
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=camera_target,
            cameraUpVector=camera_up
        )
        
        # 计算投影矩阵
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=self.camera_fov,
            aspect=self.camera_aspect,
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
        
        # 转换为OpenCV格式（BGR）
        rgb_array = np.array(rgb_img).reshape(height, width, 4)
        rgb_array = rgb_array[:, :, :3]  # 去掉alpha通道
        bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
        
        return bgr_array
    
    def init_open3d_visualizer(self):
        """初始化Open3D可视化器"""
        # 创建可视化器
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="实时点云视图", width=1000, height=800)
        
        # 创建点云对象
        self.point_cloud = o3d.geometry.PointCloud()
        
        # 创建坐标系
        self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        
        # 添加到可视化器
        self.vis.add_geometry(self.point_cloud)
        self.vis.add_geometry(self.coordinate_frame)
        
        # 获取渲染选项并优化设置
        render_option = self.vis.get_render_option()
        render_option.point_show_normal = False
        render_option.point_size = 5.0
        render_option.point_color_option = o3d.visualization.PointColorOption.Color
        render_option.light_on = True
        render_option.show_coordinate_frame = True
        render_option.background_color = np.array([0.05, 0.05, 0.05])
        
        # 设置视角
        ctr = self.vis.get_view_control()
        ctr.set_front([0, -1, 0])
        ctr.set_lookat([0, 0, 0.5])
        ctr.set_up([0, 0, 1])
        ctr.set_zoom(0.8)
    
    def update_point_cloud(self, points, colors):
        """更新点云数据"""
        if self.point_cloud is None or self.vis is None or len(points) == 0:
            return
        
        # 确保点和颜色数量匹配
        if len(points) != len(colors):
            min_len = min(len(points), len(colors))
            points = points[:min_len]
            colors = colors[:min_len]
        
        # 更新点云数据
        self.point_cloud.points = o3d.utility.Vector3dVector(points)
        self.point_cloud.colors = o3d.utility.Vector3dVector(colors)
        
        # 更新可视化
        self.vis.update_geometry(self.point_cloud)
        self.vis.poll_events()
        self.vis.update_renderer()
    
    def pointcloud_thread(self):
        """点云生成和显示线程"""
        # 初始化Open3D可视化器
        self.init_open3d_visualizer()
        
        frame_count = 0
        last_successful_points = None
        last_successful_colors = None
        
        while self.running and self.pointcloud_running:
            # 更新机械臂姿态
            if frame_count % 2 == 0:
                self.update_robot_pose()
                p.stepSimulation()
            
            # 生成点云
            if frame_count % self.pointcloud_update_interval == 0:
                # 获取末端法兰位置和姿态
                flange_position, flange_orientation = self.get_end_effector_pose()
                
                # 计算相机位置和姿态
                camera_position, camera_orientation, camera_target = self.calculate_camera_pose(
                    flange_position, flange_orientation)
                
                # 捕获相机图像和深度图
                bgr_image, rgb_image, depth_image, view_matrix, projection_matrix = self.capture_camera_image_with_depth(
                    camera_position, camera_target)
                
                # 生成点云
                points, colors = self.depth_image_to_point_cloud(
                    rgb_image, depth_image, camera_position, camera_target)
                
                # 检查点云是否有效
                if len(points) > 100:
                    last_successful_points = points
                    last_successful_colors = colors
                    
                    # 更新Open3D点云显示
                    self.update_point_cloud(points, colors)
                    
                elif last_successful_points is not None:
                    # 如果当前帧点云无效，使用上一次成功的点云
                    self.update_point_cloud(last_successful_points, last_successful_colors)
            
            # 更新Open3D显示
            self.vis.poll_events()
            self.vis.update_renderer()
            
            frame_count += 1
            time.sleep(0.03)
        
        # 关闭Open3D窗口
        if self.vis:
            self.vis.destroy_window()
    
    def camera_thread(self):
        """相机显示线程"""
        cv2.namedWindow('Virtual Camera View', cv2.WINDOW_AUTOSIZE)
        
        while self.running and self.camera_running:
            # 更新机械臂姿态
            self.update_robot_pose()
            p.stepSimulation()
            
            # 获取末端法兰位置和姿态
            flange_position, flange_orientation = self.get_end_effector_pose()
            
            # 计算相机位置和姿态
            camera_position, camera_orientation, camera_target = self.calculate_camera_pose(
                flange_position, flange_orientation)
            
            # 捕获相机图像
            camera_image = self.capture_camera_image(camera_position, camera_target)
            
            # 在图像上添加信息
            display_image = self.display_info_on_image(camera_image)
            
            # 显示图像
            cv2.imshow('Virtual Camera View', display_image)
            cv2.waitKey(1)
            
            time.sleep(0.03)
        
        cv2.destroyAllWindows()
    
    def display_info_on_image(self, image):
        """在图像上显示信息"""
        img_with_info = image.copy()
        
        # 添加文本信息
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (0, 255, 0)  # 绿色
        thickness = 2
        
        # 显示相机参数和状态
        info_lines = [
            f"状态: {'已使能' if self.is_enabled else '未使能'}",
            f"FOV: {self.camera_fov:.1f}°",
            f"选中关节: J{self.selected_joint + 1}",
            f"关节角度: {[f'{a:.1f}°' for a in self.current_joint_angles]}"
        ]
        
        y_offset = 30
        for i, line in enumerate(info_lines):
            cv2.putText(img_with_info, line, (10, y_offset + i * 25), 
                       font, font_scale, color, thickness)
        
        # 显示控制说明
        control_info = [
            "终端控制命令:",
            "joint <1-6>: 选择关节",
            "up/down: 关节角度 +/-",
            "fov <angle>: 设置视野角度",
            "downsample <倍数>: 设置点云下采样倍数",
            "toggle_pointcloud: 开启/关闭点云生成",
            "reset: 重置到使能状态",
            "quit: 退出程序"
        ]
        
        y_start = img_with_info.shape[0] - len(control_info) * 20 - 10
        for i, line in enumerate(control_info):
            cv2.putText(img_with_info, line, (10, y_start + i * 18), 
                       font, 0.4, (255, 255, 0), 1)  # 青色小字
        
        return img_with_info
    
    def handle_terminal_input(self, command):
        """处理终端输入命令"""
        command = command.strip().lower()
        
        if command == 'quit' or command == 'exit':
            return False
        
        elif command.startswith('joint '):
            joint_num = int(command.split()[1])
            if 1 <= joint_num <= 6:
                self.selected_joint = joint_num - 1
                print(f"🎯 选中关节 J{joint_num}")
            else:
                print("❌ 关节编号必须在1-6之间")
        
        elif command == 'up':
            if self.is_enabled:
                self.current_joint_angles[self.selected_joint] += self.angle_step
                print(f"⬆️ J{self.selected_joint + 1}: {self.current_joint_angles[self.selected_joint]:.1f}°")
            else:
                print("❌ 请先完成使能过程")
        
        elif command == 'down':
            if self.is_enabled:
                self.current_joint_angles[self.selected_joint] -= self.angle_step
                print(f"⬇️ J{self.selected_joint + 1}: {self.current_joint_angles[self.selected_joint]:.1f}°")
            else:
                print("❌ 请先完成使能过程")
        
        elif command.startswith('fov '):
            fov_value = float(command.split()[1])
            if 10.0 <= fov_value <= 180.0:
                self.camera_fov = fov_value
                print(f"🔍 FOV设置为: {self.camera_fov:.1f}°")
            else:
                print("❌ FOV必须在10-180度之间")
        
        elif command.startswith('downsample '):
            downsample_value = int(command.split()[1])
            if 1 <= downsample_value <= 20:
                self.point_cloud_downsample = downsample_value
                print(f"🔽 点云下采样设置为: {self.point_cloud_downsample}")
            else:
                print("❌ 下采样倍数必须在1-20之间")
        
        elif command == 'toggle_pointcloud':
            self.point_cloud_enabled = not self.point_cloud_enabled
            print(f"🎨 点云生成: {'开启' if self.point_cloud_enabled else '关闭'}")
        
        elif command.startswith('interval '):
            interval_value = int(command.split()[1])
            if 1 <= interval_value <= 60:
                self.pointcloud_update_interval = interval_value
                print(f"⏱️ 点云更新间隔设置为: {self.pointcloud_update_interval} 帧")
            else:
                print("❌ 更新间隔必须在1-60帧之间")
        
        elif command.startswith('pointsize '):
            point_size = float(command.split()[1])
            if 1.0 <= point_size <= 20.0:
                if self.vis:
                    render_option = self.vis.get_render_option()
                    render_option.point_size = point_size
                    print(f"🔍 点云大小设置为: {point_size}")
                else:
                    print("❌ 点云可视化器未初始化")
            else:
                print("❌ 点大小必须在1.0-20.0之间")
        
        elif command.startswith('depth_range '):
            parts = command.split()
            if len(parts) == 3:
                min_depth = float(parts[1])
                max_depth = float(parts[2])
                if 0.001 <= min_depth < max_depth <= 5.0:
                    self.depth_threshold_min = min_depth
                    self.depth_threshold_max = max_depth
                    print(f"📏 深度范围设置为: {min_depth:.3f}m - {max_depth:.3f}m")
                else:
                    print("❌ 深度范围必须: 0.001 <= min < max <= 5.0")
            else:
                print("❌ 用法: depth_range <最小深度> <最大深度>")
        
        elif command == 'reset':
            if self.is_enabled:
                self.current_joint_angles = self.ready_state.copy()
                print("🔄 重置到使能状态")
            else:
                print("❌ 请先完成使能过程")
        
        elif command == 'status':
            print(f"📊 当前状态:")
            print(f"   使能状态: {'是' if self.is_enabled else '否'}")
            print(f"   选中关节: J{self.selected_joint + 1}")
            print(f"   关节角度: {[f'{a:.1f}°' for a in self.current_joint_angles]}")
            print(f"   相机FOV: {self.camera_fov:.1f}°")
            print(f"   点云生成: {'开启' if self.point_cloud_enabled else '关闭'}")
        
        elif command == 'help':
            print("📖 可用命令:")
            print("   joint <1-6>      - 选择关节")
            print("   up               - 增加选中关节角度")
            print("   down             - 减少选中关节角度")
            print("   fov <角度>       - 设置相机视野角度")
            print("   downsample <倍数> - 设置点云下采样倍数(1-20)")
            print("   interval <帧数>   - 设置点云更新间隔(1-60)")
            print("   pointsize <大小>  - 设置点云点大小(1.0-20.0)")
            print("   depth_range <min> <max> - 设置深度范围")
            print("   toggle_pointcloud - 开启/关闭点云生成")
            print("   reset            - 重置到使能状态")
            print("   status           - 显示当前状态")
            print("   help             - 显示帮助")
            print("   quit             - 退出程序")
        
        else:
            print(f"❌ 未知命令: {command}")
            print("输入 'help' 查看可用命令")
        
        return True
    
    def run_simulation(self):
        """运行主模拟循环"""
        print("\n🚀 启动虚拟相机模拟器...")
        print("=" * 60)
        
        # 设置初始状态
        self.set_robot_to_disabled_state()
        print(f"未使能状态: {self.disabled_state}")
        print(f"目标使能状态: {self.ready_state}")
        
        # 询问是否开始使能
        input("按 Enter 开始使能过程...")
        
        # 执行使能过程
        self.execute_enable_process()
        
        if not self.running:
            return
        
        # 启动相机显示
        print("\n📹 启动虚拟相机...")
        self.camera_running = True
        camera_thread = threading.Thread(target=self.camera_thread)
        camera_thread.daemon = True
        camera_thread.start()
        
        # 启动点云生成
        if self.point_cloud_enabled:
            print("\n🎨 启动点云生成...")
            self.pointcloud_running = True
            pointcloud_thread = threading.Thread(target=self.pointcloud_thread)
            pointcloud_thread.daemon = True
            pointcloud_thread.start()
        
        # 终端交互循环
        print("\n💬 终端交互模式启动")
        print("输入 'help' 查看可用命令，输入 'quit' 退出程序")
        print("=" * 60)
        
        while self.running:
            command = input("请输入命令: ")
            if not self.handle_terminal_input(command):
                break
        
        print("🔚 程序结束")
    
    def cleanup(self):
        """清理资源"""
        self.running = False
        self.camera_running = False
        self.pointcloud_running = False
        
        if self.physics_client is not None:
            p.disconnect()

def main():
    """主函数"""
    simulator = VirtualCameraWithPointCloud()
    
    # 启动PyBullet环境
    if not simulator.start_pybullet():
        return
    
    # 运行仿真
    simulator.run_simulation()
    
    # 清理资源
    simulator.cleanup()

if __name__ == "__main__":
    main() 