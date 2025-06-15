#!/usr/bin/env python3
"""
机械臂虚拟相机 + YOLO-E 目标检测集成程序
功能：
- 集成虚拟相机模拟器的所有功能
- 添加基于命令的目标检测功能
- 支持用户输入文本描述进行目标检测
- 显示检测结果和掩码可视化
"""

import sys
import os
import math
import time
import threading
import numpy as np
import cv2
from PIL import Image
import matplotlib
matplotlib.use('Agg')  # 使用非交互式后端
import matplotlib.pyplot as plt

# 添加CLI-Tool路径
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "CLI-Tool"))

import pybullet as p
import pybullet_data
from ultralytics import YOLO

class VirtualCameraYOLODetector:
    """虚拟相机 + YOLO-E 目标检测器"""
    
    def __init__(self):
        # PyBullet仿真环境相关
        self.physics_client = None
        self.robot_id = None
        self.joint_indices = []
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 机械臂状态定义
        self.disabled_state = [0.0, 70.0, 90.0, 0.0, 0.0, 0.0]
        self.ready_state = [0.0, 60.0, 60.0, 90.0, 0.0, 0.0]
        self.current_joint_angles = self.disabled_state.copy()
        
        # 使能参数
        self.enable_duration = 3.0
        self.enable_steps = 120
        self.is_enabled = False
        
        # 相机参数
        self.camera_offset = [0.0, 0.06, 0.0]
        self.camera_fov = 120.0
        self.camera_aspect = 1.0
        self.camera_near = 0.01
        self.camera_far = 2.0
        self.image_width = 640
        self.image_height = 480
        
        # YOLO模型
        self.yolo_model = None
        self.model_path = "yoloe-11s-seg.pt"
        
        # 控制参数
        self.angle_step = 5.0
        self.selected_joint = 0
        self.end_effector_link_index = 7
        
        # 运行控制
        self.running = True
        self.camera_running = False
        
        print("🤖 机械臂虚拟相机 + YOLO-E 目标检测器")
        print("=" * 60)
    
    def load_yolo_model(self):
        """加载YOLO-E模型"""
        try:
            model_full_path = os.path.join(os.path.dirname(__file__), self.model_path)
            if not os.path.exists(model_full_path):
                print(f"❌ 找不到YOLO模型文件: {model_full_path}")
                return False
            
            print(f"🔄 加载YOLO-E模型: {model_full_path}")
            self.yolo_model = YOLO(model_full_path, task='segment')
            print("✅ YOLO-E模型加载成功")
            return True
        except Exception as e:
            print(f"❌ YOLO模型加载失败: {e}")
            return False
    
    def start_pybullet(self):
        """启动PyBullet仿真环境"""
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(enableFileCaching=0)
        
        # 加载地面和机械臂
        p.loadURDF("plane.urdf", [0, 0, 0])
        
        urdf_path = os.path.join(os.path.dirname(os.path.dirname(
            os.path.realpath(__file__))), "dummy2", "dummy2.urdf")
        
        if not os.path.exists(urdf_path):
            print(f"❌ 找不到URDF文件: {urdf_path}")
            return False
        
        self.robot_id = p.loadURDF(urdf_path, [0, 0, 0], useFixedBase=True)
        self.get_joint_info()
        
        # 设置视角
        p.resetDebugVisualizerCamera(
            cameraDistance=1.5, cameraYaw=45, cameraPitch=-30, cameraTargetPosition=[0, 0, 0.5])
        
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
        """添加测试物体"""
        # 货架结构
        shelf_base_pos = [0.0, 0.4, 0.0]
        shelf_color = [0.8, 0.8, 0.8, 1.0]
        
        # 货架组件
        components = [
            ([0.15, 0.05, 0.01], [0, 0, 0.01]),  # 底座
            ([0.01, 0.01, 0.2], [-0.14, 0, 0.2]),  # 左立柱
            ([0.01, 0.01, 0.2], [0.14, 0, 0.2]),   # 右立柱
            ([0.14, 0.04, 0.005], [0, 0, 0.15]),   # 第一层
            ([0.14, 0.04, 0.005], [0, 0, 0.3])     # 第二层
        ]
        
        for extents, offset in components:
            pos = [shelf_base_pos[0] + offset[0], shelf_base_pos[1] + offset[1], shelf_base_pos[2] + offset[2]]
            p.createMultiBody(
                baseMass=0,
                baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=extents),
                baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=extents, rgbaColor=shelf_color),
                basePosition=pos
            )
        
        # 货架上的物品
        item_colors = [[1, 0, 0, 1], [0, 1, 0, 1], [0, 0, 1, 1]]
        item_positions = [
            [shelf_base_pos[0] - 0.08, shelf_base_pos[1], shelf_base_pos[2] + 0.17],
            [shelf_base_pos[0], shelf_base_pos[1], shelf_base_pos[2] + 0.17],
            [shelf_base_pos[0] + 0.08, shelf_base_pos[1], shelf_base_pos[2] + 0.17]
        ]
        
        for pos, color in zip(item_positions, item_colors):
            p.createMultiBody(
                baseMass=0.05,
                baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02]),
                baseVisualShapeIndex=p.createVisualShape(p.GEOM_BOX, halfExtents=[0.02, 0.02, 0.02], rgbaColor=color),
                basePosition=pos
            )
        
        print("✅ 测试场景添加完成")
    
    def execute_enable_process(self):
        """执行使能过程"""
        print(f"🔄 使能过程开始...")
        
        for step in range(self.enable_steps + 1):
            if not self.running:
                break
                
            progress = step / self.enable_steps
            smooth_progress = progress * progress * (3.0 - 2.0 * progress)
            
            self.current_joint_angles = [
                start + (end - start) * smooth_progress 
                for start, end in zip(self.disabled_state, self.ready_state)
            ]
            
            self.update_robot_pose()
            p.stepSimulation()
            time.sleep(self.enable_duration / self.enable_steps)
        
        self.is_enabled = True
        print("✅ 使能完成")
    
    def update_robot_pose(self):
        """更新机械臂姿态"""
        for i, joint_idx in enumerate(self.joint_indices):
            if joint_idx != -1:
                p.resetJointState(self.robot_id, joint_idx, math.radians(self.current_joint_angles[i]))
    
    def capture_current_frame(self):
        """捕获当前相机画面"""
        if not self.is_enabled:
            print("❌ 请先完成使能过程")
            return None, None
        
        self.update_robot_pose()
        p.stepSimulation()
        
        # 获取末端位置
        link_state = p.getLinkState(self.robot_id, self.end_effector_link_index, computeForwardKinematics=True)
        flange_position = list(link_state[0])
        flange_orientation = list(link_state[1])
        
        # 计算相机位置
        rotation_matrix = np.array(p.getMatrixFromQuaternion(flange_orientation)).reshape(3, 3)
        camera_offset_global = rotation_matrix @ np.array(self.camera_offset)
        camera_position = np.array(flange_position) + camera_offset_global
        forward_direction = rotation_matrix @ np.array([0, 1, 0])
        camera_target = camera_position + forward_direction * 0.2
        
        # 渲染图像
        view_matrix = p.computeViewMatrix(camera_position, camera_target, [0, 0, 1])
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=self.camera_fov, aspect=self.camera_aspect, 
            nearVal=self.camera_near, farVal=self.camera_far
        )
        
        width, height, rgb_img, depth_img, seg_img = p.getCameraImage(
            width=self.image_width, height=self.image_height,
            viewMatrix=view_matrix, projectionMatrix=projection_matrix,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
        )
        
        # 转换格式
        rgb_array = np.array(rgb_img).reshape(height, width, 4)[:, :, :3]
        bgr_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
        
        return bgr_array, rgb_array
    
    def perform_yolo_detection(self, rgb_image, text_prompt):
        """执行YOLO检测"""
        if self.yolo_model is None:
            print("❌ YOLO模型未加载")
            return None
        
        try:
            print(f"🔍 正在检测: '{text_prompt}'")
            pil_image = Image.fromarray(rgb_image)
            
            # 设置检测类别
            self.yolo_model.set_classes([text_prompt], self.yolo_model.get_text_pe([text_prompt]))
            results = self.yolo_model.predict(pil_image, conf=0.5)
            
            detected_count = len(results[0].boxes) if results[0].boxes is not None else 0
            print(f"✅ 检测完成，发现 {detected_count} 个目标")
            return results
            
        except Exception as e:
            print(f"❌ YOLO检测失败: {e}")
            return None
    
    def visualize_detection_results(self, image, results, text_prompt):
        """可视化检测结果"""
        if not results or results[0].boxes is None:
            print("❌ 没有检测到目标")
            return
        
        # 检测结果处理
        result_image = image.copy()
        boxes = results[0].boxes.xyxy.cpu().numpy()
        confidences = results[0].boxes.conf.cpu().numpy()
        
        print(f"📊 检测统计:")
        print(f"   发现目标数量: {len(boxes)}")
        
        # 处理掩码
        if hasattr(results[0], 'masks') and results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()
            print(f"   包含分割掩码: 是")
            
            for i, (box, conf, mask) in enumerate(zip(boxes, confidences, masks)):
                x1, y1, x2, y2 = box.astype(int)
                
                # 调整掩码尺寸
                mask_resized = cv2.resize(mask, (image.shape[1], image.shape[0]))
                mask_binary = (mask_resized > 0.5).astype(np.uint8)
                
                # 创建彩色掩码
                color = np.random.randint(0, 255, 3).tolist()
                colored_mask = np.zeros_like(result_image)
                colored_mask[mask_binary == 1] = color
                
                # 叠加掩码
                result_image = cv2.addWeighted(result_image, 0.7, colored_mask, 0.3, 0)
                
                # 绘制检测框和标签
                cv2.rectangle(result_image, (x1, y1), (x2, y2), color, 2)
                label = f"{text_prompt} {conf:.2f}"
                cv2.putText(result_image, label, (x1, y1 - 5), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                print(f"   目标 {i+1}: 置信度 {conf:.3f}, 位置 ({x1},{y1})-({x2},{y2})")
        else:
            print(f"   包含分割掩码: 否")
            # 仅检测框
            for i, (box, conf) in enumerate(zip(boxes, confidences)):
                x1, y1, x2, y2 = box.astype(int)
                color = (0, 255, 0)
                cv2.rectangle(result_image, (x1, y1), (x2, y2), color, 2)
                label = f"{text_prompt} {conf:.2f}"
                cv2.putText(result_image, label, (x1, y1 - 5), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
                print(f"   目标 {i+1}: 置信度 {conf:.3f}, 位置 ({x1},{y1})-({x2},{y2})")
        
        # 保存结果图像
        timestamp = int(time.time())
        result_filename = f"detection_result_{timestamp}.png"
        result_path = os.path.join(os.path.dirname(__file__), result_filename)
        cv2.imwrite(result_path, result_image)
        print(f"📸 检测结果已保存: {result_path}")
        
        # 使用OpenCV显示结果（更稳定）
        try:
            # 创建对比图像
            original_resized = cv2.resize(image, (320, 240))
            result_resized = cv2.resize(result_image, (320, 240))
            
            # 水平拼接
            comparison = np.hstack([original_resized, result_resized])
            
            # 添加标题
            cv2.putText(comparison, "Original", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(comparison, f"Detection: {len(boxes)} targets", (330, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 显示对比图像
            cv2.imshow('Detection Results', comparison)
            print("🖼️  检测结果显示窗口已打开，按任意键关闭")
            cv2.waitKey(0)
            cv2.destroyWindow('Detection Results')
            
        except Exception as e:
            print(f"⚠️  图像显示失败: {e}")
            print("📁 请查看保存的结果图像文件")
    
    def camera_thread(self):
        """相机显示线程"""
        cv2.namedWindow('Virtual Camera View', cv2.WINDOW_AUTOSIZE)
        
        while self.running and self.camera_running:
            self.update_robot_pose()
            p.stepSimulation()
            
            bgr_image, _ = self.capture_current_frame()
            if bgr_image is not None:
                # 添加状态信息
                display_image = bgr_image.copy()
                cv2.putText(display_image, f"状态: {'已使能' if self.is_enabled else '未使能'}", 
                          (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(display_image, f"输入 'detect <描述>' 进行检测", 
                          (10, display_image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                
                cv2.imshow('Virtual Camera View', display_image)
            
            cv2.waitKey(1)
            time.sleep(0.03)
        
        cv2.destroyAllWindows()
    
    def handle_terminal_input(self, command):
        """处理终端命令"""
        command = command.strip()
        
        if command.lower() in ['quit', 'exit']:
            return False
        
        elif command.lower().startswith('joint '):
            try:
                joint_num = int(command.split()[1])
                if 1 <= joint_num <= 6:
                    self.selected_joint = joint_num - 1
                    print(f"🎯 选中关节 J{joint_num}")
                else:
                    print("❌ 关节编号必须在1-6之间")
            except:
                print("❌ 请输入正确的关节编号")
        
        elif command.lower() == 'up':
            if self.is_enabled:
                self.current_joint_angles[self.selected_joint] += self.angle_step
                print(f"⬆️ J{self.selected_joint + 1}: {self.current_joint_angles[self.selected_joint]:.1f}°")
            else:
                print("❌ 请先完成使能过程")
        
        elif command.lower() == 'down':
            if self.is_enabled:
                self.current_joint_angles[self.selected_joint] -= self.angle_step
                print(f"⬇️ J{self.selected_joint + 1}: {self.current_joint_angles[self.selected_joint]:.1f}°")
            else:
                print("❌ 请先完成使能过程")
        
        elif command.lower().startswith('detect '):
            text_prompt = command[7:].strip()
            if not text_prompt:
                print("❌ 请提供检测目标的文本描述")
                return True
            
            print("📹 捕获当前画面...")
            bgr_image, rgb_image = self.capture_current_frame()
            
            if bgr_image is None:
                return True
            
            print("🤖 开始YOLO-E检测...")
            results = self.perform_yolo_detection(rgb_image, text_prompt)
            
            if results:
                self.visualize_detection_results(bgr_image, results, text_prompt)
        
        elif command.lower() == 'capture':
            print("📹 捕获当前画面...")
            bgr_image, rgb_image = self.capture_current_frame()
            
            if bgr_image is not None:
                # 保存图像
                timestamp = int(time.time())
                capture_path = os.path.join(os.path.dirname(__file__), f"captured_frame_{timestamp}.png")
                cv2.imwrite(capture_path, bgr_image)
                print(f"📸 画面已保存: {capture_path}")
                
                # 使用OpenCV显示（更稳定）
                try:
                    cv2.imshow('Captured Frame', bgr_image)
                    print("🖼️  捕获画面显示窗口已打开，按任意键关闭")
                    cv2.waitKey(0)
                    cv2.destroyWindow('Captured Frame')
                except Exception as e:
                    print(f"⚠️  图像显示失败: {e}")
                    print("📁 请查看保存的图像文件")
        
        elif command.lower() == 'reset':
            if self.is_enabled:
                self.current_joint_angles = self.ready_state.copy()
                print("🔄 重置到使能状态")
            else:
                print("❌ 请先完成使能过程")
        
        elif command.lower() == 'help':
            print("📖 可用命令:")
            print("   joint <1-6>           - 选择关节")
            print("   up/down               - 调整关节角度")
            print("   detect <文本描述>     - 目标检测")
            print("   capture               - 捕获画面")
            print("   reset                 - 重置姿态")
            print("   help                  - 显示帮助")
            print("   quit                  - 退出程序")
            print("\n📝 检测示例:")
            print("   detect red cube")
            print("   detect yellow object")
            print("   detect box on shelf")
        
        else:
            print(f"❌ 未知命令，输入 'help' 查看帮助")
        
        return True
    
    def run_simulation(self):
        """运行主程序"""
        print("\n🚀 启动程序...")
        
        if not self.load_yolo_model():
            return
        
        # 初始设置
        for i, joint_idx in enumerate(self.joint_indices):
            if joint_idx != -1:
                p.resetJointState(self.robot_id, joint_idx, math.radians(self.disabled_state[i]))
        
        input("按 Enter 开始使能过程...")
        self.execute_enable_process()
        
        if not self.running:
            return
        
        print("\n📹 启动虚拟相机...")
        self.camera_running = True
        camera_thread = threading.Thread(target=self.camera_thread)
        camera_thread.daemon = True
        camera_thread.start()
        
        print("\n💬 终端交互模式启动")
        print("输入 'help' 查看命令，输入 'detect <描述>' 进行检测")
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
        if self.physics_client is not None:
            p.disconnect()

def main():
    """主函数"""
    detector = VirtualCameraYOLODetector()
    
    if not detector.start_pybullet():
        return
    
    detector.run_simulation()
    detector.cleanup()

if __name__ == "__main__":
    main() 