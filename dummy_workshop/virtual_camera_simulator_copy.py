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

class VirtualCameraWithPointCloud:
    def __init__(self):

        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        self.disabled_state = [0.0, 70.0, 90.0, 0.0, 0.0, 0.0]
        self.ready_state = [0.0, 60.0, 60.0, 90.0, 0.0, 0.0]

        self.enable_duration = 3.0
        self.enable_steps = 120

        self.end_effector_link_index = 7

        self.camera_offset = [0.0, 0.06, 0.0]
        self.camera_fov = 120.0
        self.camera_aspect = 1.0
        self.camera_near = 0.01
        self.camera_far = 2.0
        self.camera_width = 640
        self.camera_height = 480

        self.point_cloud_enabled = True
        self.pointcloud_running = False
        self.pointcloud_update_interval = 10 # 点云更新间隔（帧数）


    def start_pybullet(self):
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(enableFileCaching=0)

        p.loadURDF("plane.urdf", [0, 0, 0])

        urdf_path = os.path.join(os.path.dirname(os.path.dirname(
            os.path.realpath(__file__))), "dummy2", "dummy2.urdf")
        
        self.robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)

        self.get_joint_info()

        p.resetDebugVisualizerCamera(
            cameraDistance=1.5,
            cameraYaw=45,
            cameraPitch=-30,
            cameraTargetPosition=[0, 0, 0.5]
        )

        return True
        

    def get_joint_info(self):
        num_joints = p.getNumJoints(self.robot_id)
        self.joint_indices = []
        
        # 把关节名和urdf的模型文件进行关联，便于控制机械臂运动
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
        return True

    def run_simulation(self):
        self.set_robot_to_disabled_state()

        self.execute_enable_process()

        camera_thread = threading.Thread(target=self.camera_thread)
        camera_thread.daemon = True
        camera_thread.start()

        if self.point_cloud_enabled:
            self.pointcloud_running = True
            pointcloud_thread = threading.Thread(target=self.pointcloud_thread)
            pointcloud_thread.daemon = True
            pointcloud_thread.start()

        while True:
            command = input("请输入命令: ")
            if not self.handle_terminal_input(command):
                break
        
        print("🔚 程序结束")



    def set_robot_to_disabled_state(self):
        for i, joint_idx in enumerate(self.joint_indices):
            if joint_idx != -1:
                p.resetJointState(self.robot_id, joint_idx, math.radians(self.disabled_state[i])) # math.radians()将角度从度转换为弧度
        
        for _ in range(10):
            p.stepSimulation()
            time.sleep(0.001)

    def execute_enable_process(self):

        step_duation = self.enable_duration / self.enable_steps

        for step in range(self.enable_steps + 1):
            progress = step / self.enable_steps

            self.current_joint_angles = self.interpolate_joint_angles(
                self.disabled_state, self.ready_state, progress
            )

            self.update_robot_pose()
            p.stepSimulation()

            time.sleep(step_duation)
        

    def interpolate_joint_angles(self, start_angles, end_angles, progress):
        smooth_progress = self.smooth_step(progress)
        return [start + (end - start) * smooth_progress 
                for start, end in zip(start_angles, end_angles)]

    def smooth_step(self, t):
        return t * t * (3.0 - 2.0 * t)

    def update_robot_pose(self):
        for i, joint_idx in enumerate(self.joint_indices):
            if joint_idx != -1:
                angle_rad = math.radians(self.current_joint_angles[i])
                p.resetJointState(self.robot_id, joint_idx, angle_rad)

    def camera_thread(self):
        cv2.namedWindow('VCam', cv2.WINDOW_AUTOSIZE)
        while True:
            self.update_robot_pose()
            p.stepSimulation()

            flange_position, flange_orientation = self.get_end_effector_pose()

            camera_position, camera_orientation, camera_target = self.calculate_camera_pose(
                flange_position, flange_orientation)
            
            camera_image = self.capture_camera_image(camera_position, camera_target)

            cv2.imshow('VCam', camera_image)
            cv2.waitKey(1)

            time.sleep(0.03)

        cv2.destroyAllWindows()


    def get_end_effector_pose(self):
        link_state = p.getLinkState(self.robot_id, self.end_effector_link_index, computeForwardKinematics=True)
        position = list(link_state[0])
        orientation = list(link_state[1])

        return position, orientation
    
    def calculate_camera_pose(self, flange_position, flange_orientation):
        rotation_matrix = np.array(p.getMatrixFromQuaternion(flange_orientation)).reshape(3, 3)

        camera_offset_local = np.array(self.camera_offset)

        camera_offset_global = rotation_matrix @ camera_offset_local

        camera_position = np.array(flange_position) + camera_offset_global

        camera_orientation = flange_orientation

        forward_direction = rotation_matrix @ np.array([0, 1, 0])
        camera_target = camera_position + forward_direction * 0.02

        return camera_position.tolist(), camera_orientation, camera_target.tolist()
    
    def capture_camera_image(self, camera_position, camera_target, camera_up=[0, 0, 1]):
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=camera_position,
            cameraTargetPosition=camera_target,
            cameraUpVector=camera_up
        )

        projection_matrix = p.computeProjectionMatrixFOV(
            fov=self.camera_fov,
            aspect=self.camera_aspect,
            nearVal=self.camera_near,
            farVal=self.camera_far
        )

        width, height, rgb_img, depth_img, seg_img= p.getCameraImage(
            width=self.camera_width,
            height=self.camera_height,
            viewMatrix=view_matrix,
            projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )

        rgb_array = np.array(rgb_img).reshape(height, width, 4)
        rgb_array = rgb_array[:, :, :3]
        bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
        return bgr_array
    
    def pointcloud_thread(self):
        self.init_open3d_visualizer()

        frame_count = 0
        last_successful_points = None
        last_successful_colors = None

        while self.pointcloud_running:
            if frame_count % 2 == 0:
                self.update_robot_pose()
                p.stepSimulation()

            if frame_count % self.pointcloud_update_interval == 0:
                flange_position, flange_orientation = self.get_end_effector_pose()

                camera_position, camera_orientation, camera_target = self.calculate_camera_pose(
                    flange_position, flange_orientation)
                
                bgr_image, rgb_image, depth_image, view_matrix, projection_matrix = self.capture_camera_image_with_depth(
                    camera_position, camera_target)
                
                points, colors = self.depth_image_to_point_cloud(
                    rgb_image, depth_image, camera_position, camera_target)
                
                if len(points) > 100:
                    last_successful_points = points
                    last_successful_colors = colors
                    
                    self.update_point_cloud(points, colors)
                    
                elif last_successful_points is not None:
                    self.update_point_cloud(last_successful_points, last_successful_colors)

            self.vis.poll_events()
            self.vis.update_renderer()
            
            frame_count += 1
            time.sleep(0.03)

        self.vis.destroy_window()

            



    def init_open3d_visualizer(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="实时点云视图", width=1000, height=800)
        
        self.point_cloud = o3d.geometry.PointCloud()
        
        self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)

        self.vis.add_geometry(self.point_cloud)
        self.vis.add_geometry(self.coordinate_frame)

        render_option = self.vis.get_render_option()
        render_option.point_show_normal = False
        render_option.point_size = 5.0
        render_option.point_color_option = o3d.visualization.PointColorOption.Color
        render_option.light_on = True
        render_option.show_coordinate_frame = True
        render_option.background_color = np.array([0.05, 0.05, 0.05])
        
        ctr = self.vis.get_view_control()
        ctr.set_front([0, -1, 0])
        ctr.set_lookat([0, 0, 0.5])
        ctr.set_up([0, 0, 1])
        ctr.set_zoom(0.8)
    
    def depth_image_to_point_cloud(self, rgb_image, depth_image, camera_position, camera_target, camera_up=[0, 0, 1]):
        points = []
        colors = []

        height, width = depth_image.shape
        
        # 计算相机内参（从FOV推导焦距）
        fov_rad = math.radians(self.camera_fov)
        focal_length = (height / 2.0) / math.tan(fov_rad / 2.0)
        cx, cy = width / 2.0, height / 2.0

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

        for v in range(0, height, step):
            for u in range(0, width, step):
                depth = depth_image[v, u]

                if (not np.isfinite(depth) or 
                    depth < self.depth_threshold_min or 
                    depth > self.depth_threshold_max or
                    depth <= 0):
                    continue
                
                x_cam = (u - cx) * depth / focal_length
                y_cam = (v - cy) * depth / focal_length
                z_cam = depth

                point_cam = np.array([x_cam, y_cam, z_cam])
                point_world = camera_pos + R @ point_cam
                
                if not np.all(np.isfinite(point_world)):
                    continue
                
                pixel_color = rgb_image[v, u].astype(np.float32) / 255.0
                pixel_color = np.clip(pixel_color, 0.0, 1.0)

                if not np.all(np.isfinite(pixel_color)):
                    continue
                
                points.append(point_world)
                colors.append(pixel_color)

        if len(points) == 0:
            return np.empty((0, 3)), np.empty((0, 3))
        
        points_array = np.array(points)
        colors_array = np.array(colors)
        
        valid_indices = np.all(np.isfinite(points_array), axis=1) & np.all(np.isfinite(colors_array), axis=1)
        
        return points_array[valid_indices], colors_array[valid_indices]
                
    def update_point_cloud(self, points, colors):
        if self.point_cloud is None or self.vis is None or len(points) == 0:
            return
        
        if len(points) != len(colors):
            min_len = min(len(points), len(colors))
            points = points[:min_len]
            colors = colors[:min_len]
        
        self.point_cloud.points = o3d.utility.Vector3dVector(points)
        self.point_cloud.colors = o3d.utility.Vector3dVector(colors)
        
        self.vis.update_geometry(self.point_cloud)
        self.vis.poll_events()
        self.vis.update_renderer()
        






def main():

    simulator = VirtualCameraWithPointCloud()

    simulator.start_pybullet()

    simulator.run_simulation()

if __name__ == "__main__":
    main()
