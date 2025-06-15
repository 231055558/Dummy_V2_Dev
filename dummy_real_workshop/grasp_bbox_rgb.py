import os
import sys
import warnings
import json
import socket
import time
import numpy as np
import cv2
import open3d as o3d

# 完全禁用所有警告
warnings.filterwarnings('ignore')
os.environ['PYTHONWARNINGS'] = 'ignore'

# 禁用特定的FutureWarning
warnings.filterwarnings('ignore', category=FutureWarning, module='torch.cuda.amp.autocast')
warnings.filterwarnings('ignore', category=FutureWarning, module='torch.amp.autocast')

code_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(f'{code_dir}/../')
import argparse
import threading
import queue
import time
from queue import Queue
import pyrealsense2 as rs
from dataclasses import dataclass

@dataclass
class BBoxInfo:
    x1: int
    y1: int
    x2: int
    y2: int
    active: bool = True

class RMJointRobotController:
    def __init__(self, ip, port=8080):
        self.ip = ip
        self.port = port
        self.sock = None
        
    def connect(self):
        """建立TCP连接"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(10)  # 设置10秒超时
            self.sock.connect((self.ip, self.port))
            print(f"成功连接到机械臂 {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            return False
    
    def send_command(self, command_dict, wait_response=True):
        """发送JSON指令到机械臂"""
        if not self.sock:
            print("错误：未建立连接")
            return None
            
        try:
            json_str = json.dumps(command_dict, separators=(',', ':'))
            full_command = json_str + "\r\n"
            
            self.sock.sendall(full_command.encode('utf-8'))
            
            if wait_response:
                response = self.sock.recv(1024).decode('utf-8').strip()
                return json.loads(response)
            return None
            
        except socket.timeout:
            print("警告：接收响应超时")
            return None
        except Exception as e:
            print(f"发送指令时出错: {e}")
            return None

class StereoCamera:
    def __init__(self):
        self.pipeline = None
        self.config = None
        self.align = None  # 用于对齐深度和RGB图像
        self.fps = 10
        self.frame_time = 1.0 / self.fps
        
        # RealSense点云对象
        self.pc = rs.pointcloud()
        self.depth_scale = None
        
        # 线程控制
        self.is_running = False
        self.capture_thread = None
        self.process_thread = None
        
        # 队列
        self.frame_queue = Queue(maxsize=1)
        self.display_queue = Queue(maxsize=1)
        
        # 框选状态
        self.bbox = None
        self.drawing = False
        self.start_point = None
        self.end_point = None
        
        # 线程锁
        self.lock = threading.Lock()

    @staticmethod
    def check_camera_availability():
        """检查RealSense相机是否可用"""
        try:
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                print("❌ 未检测到RealSense设备")
                return False
            
            # 尝试简单的连接测试
            test_pipeline = rs.pipeline()
            test_config = rs.config()
            test_config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            try:
                test_pipeline.start(test_config)
                # 尝试获取一帧数据
                frames = test_pipeline.wait_for_frames(timeout_ms=5000)
                test_pipeline.stop()
                print("✅ 相机可用性测试通过")
                return True
            except Exception as e:
                print(f"❌ 相机被占用或不可用: {e}")
                try:
                    test_pipeline.stop()
                except:
                    pass
                return False
                
        except Exception as e:
            print(f"❌ 检查相机可用性时出错: {e}")
            return False

    def set_fps(self, fps: int):
        """设置帧率"""
        self.fps = fps
        self.frame_time = 1.0 / self.fps

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
            self.start_point = (x, y)
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            self.end_point = (x, y)
        elif event == cv2.EVENT_LBUTTONUP:
            self.drawing = False
            if self.start_point and (x, y):
                x1, y1 = self.start_point
                x2, y2 = x, y
                # 确保坐标为正
                x1, x2 = min(x1, x2), max(x1, x2)
                y1, y2 = min(y1, y2), max(y1, y2)
                self.bbox = BBoxInfo(x1, y1, x2, y2)
                print(f"框选区域坐标: ({x1}, {y1}), ({x2}, {y2})")

    def init_realsense(self):
        """初始化RealSense相机"""
        try:
            # 先尝试获取可用设备
            ctx = rs.context()
            devices = ctx.query_devices()
            if len(devices) == 0:
                print("❌ 未检测到RealSense设备")
                return False
            
            device = devices[0]
            print(f"✅ 检测到设备: {device.get_info(rs.camera_info.name)}")
            
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # 配置深度和RGB流（确保完美对齐）
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            # 启动管道
            profile = self.pipeline.start(self.config)
            
            # 获取深度传感器并配置
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            print(f"📏 深度单位: {self.depth_scale:.6f} 米/单位")
            
            # 优化深度传感器设置
            if depth_sensor.supports(rs.option.visual_preset):
                try:
                    # 设置为高精度模式
                    depth_sensor.set_option(rs.option.visual_preset, 4)  # High Accuracy preset
                    print("🎯 已设置高精度模式")
                except:
                    try:
                        depth_sensor.set_option(rs.option.visual_preset, 3)  # High Density preset
                        print("🎯 已设置高密度模式")
                    except:
                        print("⚠️ 无法设置视觉预设，使用默认设置")
            
            if depth_sensor.supports(rs.option.laser_power):
                # 设置激光功率为最大
                depth_sensor.set_option(rs.option.laser_power, 240)
                print("🔆 已设置最大激光功率")
            
            # 创建对齐对象：将深度图对齐到RGB图像坐标系
            self.align = rs.align(rs.stream.color)
            
            # 设置后处理滤波器（提高点云质量）
            self.setup_post_processing_filters()
            
            # 等待几帧以确保相机稳定
            print("📷 等待相机稳定...")
            for i in range(10):
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                    if frames:
                        print(f"第{i+1}/10帧稳定测试成功")
                        break
                except RuntimeError as e:
                    print(f"第{i+1}/10帧稳定测试失败: {e}")
                    if i == 9:  # 最后一次尝试
                        raise e
                    time.sleep(0.1)
            
            print("✅ RealSense D435i (深度+RGB模式) 初始化成功")
            return True
        except Exception as e:
            print(f"❌ RealSense初始化失败: {str(e)}")
            if self.pipeline:
                try:
                    self.pipeline.stop()
                except:
                    pass
            return False

    def setup_post_processing_filters(self):
        """设置后处理滤波器链"""
        try:
            # 1. 视差变换滤波器
            self.disparity_to_depth = rs.disparity_transform(True)
            self.depth_to_disparity = rs.disparity_transform(False)
            
            # 2. 空间滤波器 - 减少噪点，保持边缘
            self.spatial_filter = rs.spatial_filter()
            self.spatial_filter.set_option(rs.option.filter_magnitude, 2)
            self.spatial_filter.set_option(rs.option.filter_smooth_alpha, 0.5)
            self.spatial_filter.set_option(rs.option.filter_smooth_delta, 20)
            
            # 3. 时序滤波器 - 利用帧间信息减少时间噪声
            self.temporal_filter = rs.temporal_filter()
            self.temporal_filter.set_option(rs.option.filter_smooth_alpha, 0.4)
            self.temporal_filter.set_option(rs.option.filter_smooth_delta, 20)
            
            # 4. 孔填充滤波器
            self.hole_filling = rs.hole_filling_filter()
            
            # 滤波器链
            self.filters = [
                self.disparity_to_depth,
                self.spatial_filter,
                self.temporal_filter,
                self.depth_to_disparity,
                self.hole_filling
            ]
            
            print(f"🔧 后处理滤波器链: {len(self.filters)} 个滤波器")
            
        except Exception as e:
            print(f"⚠️ 滤波器设置失败: {e}")
            self.filters = []

    def apply_filters(self, depth_frame):
        """应用后处理滤波器链"""
        if not hasattr(self, 'filters') or not self.filters:
            return depth_frame
        
        filtered_frame = depth_frame
        for filter_obj in self.filters:
            try:
                filtered_frame = filter_obj.process(filtered_frame)
            except:
                continue
        
        return filtered_frame

    def capture_frames(self):
        """图像采集线程"""
        if not self.init_realsense():
            print("无法启动RealSense相机")
            self.is_running = False
            return

        consecutive_failures = 0
        max_consecutive_failures = 5
        
        while self.is_running:
            try:
                # 使用较长的超时时间并处理超时异常
                frames = self.pipeline.wait_for_frames(timeout_ms=10000)
                
                # 重置失败计数器
                consecutive_failures = 0
                
                # 对齐深度图到RGB图像坐标系
                aligned_frames = self.align.process(frames)
                
                # 获取对齐后的深度图和RGB图像
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    print("⚠️ 未获取到深度帧或RGB帧")
                    continue
                
                # 应用后处理滤波器提高深度图质量
                filtered_depth_frame = self.apply_filters(depth_frame)
                
                # 转换为numpy数组
                rgb_image = np.asanyarray(color_frame.get_data())

                try:
                    # 传递深度帧和RGB图像用于点云计算
                    self.frame_queue.put_nowait((filtered_depth_frame, color_frame, rgb_image.copy(), self.bbox))
                except queue.Full:
                    try:
                        self.frame_queue.get_nowait()
                        self.frame_queue.put_nowait((filtered_depth_frame, color_frame, rgb_image.copy(), self.bbox))
                    except queue.Empty:
                        pass

            except RuntimeError as e:
                error_msg = str(e)
                consecutive_failures += 1
                
                if "Frame didn't arrive within" in error_msg:
                    print(f"⚠️ 帧接收超时 ({consecutive_failures}/{max_consecutive_failures}): {error_msg}")
                elif "Frame was corrupted" in error_msg:
                    print(f"⚠️ 帧损坏 ({consecutive_failures}/{max_consecutive_failures}): {error_msg}")
                else:
                    print(f"⚠️ 采集帧时出错 ({consecutive_failures}/{max_consecutive_failures}): {error_msg}")
                
                # 如果连续失败次数过多，尝试重新初始化相机
                if consecutive_failures >= max_consecutive_failures:
                    print("🔄 连续失败次数过多，尝试重新初始化相机...")
                    self.release()
                    time.sleep(1)
                    if self.init_realsense():
                        consecutive_failures = 0
                        print("✅ 相机重新初始化成功")
                    else:
                        print("❌ 相机重新初始化失败，停止采集")
                        self.is_running = False
                        break
                else:
                    # 短暂等待后重试
                    time.sleep(0.1)
                    
            except Exception as e:
                consecutive_failures += 1
                print(f"⚠️ 采集帧时出现未知错误 ({consecutive_failures}/{max_consecutive_failures}): {str(e)}")
                
                if consecutive_failures >= max_consecutive_failures:
                    print("❌ 错误次数过多，停止采集")
                    self.is_running = False
                    break
                else:
                    time.sleep(0.1)

    def process_frames(self, model, args):
        """图像处理线程 - 使用RealSense内置点云生成"""
        while self.is_running:
            try:
                depth_frame, color_frame, rgb_img, bbox = self.frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            try:
                # 使用RealSense内置的高效点云生成算法
                # 这样可以确保RGB颜色与3D坐标完美对应
                
                # 设置点云的纹理映射
                self.pc.map_to(color_frame)
                
                # 计算点云
                points = self.pc.calculate(depth_frame)
                
                # 获取顶点坐标（3D位置）
                vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
                
                # 获取纹理坐标用于颜色映射
                tex_coords = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
                
                # 从RGB图像中提取颜色
                h, w = rgb_img.shape[:2]
                
                # 批量处理纹理坐标以获取颜色
                u_coords = np.clip((tex_coords[:, 0] * w).astype(np.int32), 0, w-1)
                v_coords = np.clip((tex_coords[:, 1] * h).astype(np.int32), 0, h-1)
                
                # 向量化颜色提取（BGR转RGB并归一化）
                colors = rgb_img[v_coords, u_coords]  # BGR格式
                colors = colors[:, [2, 1, 0]] / 255.0  # 转换为RGB并归一化
                
                # 创建Open3D点云
                pcd = o3d.geometry.PointCloud()
                pcd.points = o3d.utility.Vector3dVector(vertices)
                pcd.colors = o3d.utility.Vector3dVector(colors)
                
                # 过滤点云：移除无效点和远距离点
                points_array = np.asarray(pcd.points)
                colors_array = np.asarray(pcd.colors)
                
                # 计算距离并过滤
                distances = np.linalg.norm(points_array, axis=1)
                valid_mask = (distances > 0.2) & (distances < args.z_far) & (distances > 0)
                
                if np.sum(valid_mask) > 0:
                    pcd.points = o3d.utility.Vector3dVector(points_array[valid_mask])
                    pcd.colors = o3d.utility.Vector3dVector(colors_array[valid_mask])
                
                # 可选：统计滤波去除离群点
                if args.denoise_cloud and len(pcd.points) > 1000:
                    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=args.denoise_nb_points, std_ratio=2.0)
                
                # 处理bbox区域标记
                if bbox and bbox.active:
                    # 由于深度图和RGB图已经对齐，bbox坐标可以直接使用
                    x1, y1, x2, y2 = bbox.x1, bbox.y1, bbox.x2, bbox.y2
                    
                    # 保存标记后的RGB图像
                    marked_img = rgb_img.copy()
                    cv2.rectangle(marked_img, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    os.makedirs('static', exist_ok=True)
                    cv2.imwrite('static/marked_rgb_image.jpg', marked_img)
                    
                    # 标记点云中bbox区域的点为红色
                    # 由于RGB和深度已经对齐，我们可以直接使用像素坐标来标记点云
                    
                    # 获取深度图数组
                    depth_image = np.asanyarray(depth_frame.get_data())
                    
                    # 创建bbox区域的掩码
                    bbox_mask = np.zeros_like(depth_image, dtype=bool)
                    bbox_mask[y1:y2, x1:x2] = True
                    
                    # 将掩码转换为点云索引
                    # 由于点云是按照图像的行列顺序排列的，我们可以计算对应的索引
                    bbox_indices = []
                    for y in range(y1, min(y2, h)):
                        for x in range(x1, min(x2, w)):
                            if depth_image[y, x] > 0:  # 只处理有效深度的点
                                idx = y * w + x
                                if idx < len(vertices) and valid_mask[idx]:
                                    # 找到过滤后点云中的对应索引
                                    original_idx = np.where(valid_mask)[0]
                                    filtered_idx = np.where(original_idx == idx)[0]
                                    if len(filtered_idx) > 0:
                                        bbox_indices.append(filtered_idx[0])
                    
                    # 将bbox区域内的点标记为红色
                    if bbox_indices:
                        colors_array = np.asarray(pcd.colors)
                        colors_array[bbox_indices] = [1, 0, 0]  # 红色
                        pcd.colors = o3d.utility.Vector3dVector(colors_array)
                        
                        # 计算bbox区域内点的中心坐标
                        bbox_points = np.asarray(pcd.points)[bbox_indices]
                        if len(bbox_points) > 0:
                            center_point = np.mean(bbox_points, axis=0)
                            print(f"\n🎯 目标物体中心坐标 (米):")
                            print(f"   X={center_point[0]:.3f}, Y={center_point[1]:.3f}, Z={center_point[2]:.3f}")
                            print(f"   标记点数: {len(bbox_points)}")

                try:
                    self.display_queue.put_nowait(pcd)
                except queue.Full:
                    try:
                        self.display_queue.get_nowait()
                        self.display_queue.put_nowait(pcd)
                    except queue.Empty:
                        pass
                        
            except Exception as e:
                print(f"⚠️ 处理帧时出错: {str(e)}")
                import traceback
                traceback.print_exc()

    def get_display_frame(self):
        """获取最新的显示帧但不删除队列中的数据"""
        try:
            with self.lock:
                if self.frame_queue.empty() or self.display_queue.empty():
                    return None
                    
                if not self.frame_queue.empty() and not self.display_queue.empty():
                    frame_data = self.frame_queue.queue[0]
                    depth_frame, color_frame, rgb_img, _ = frame_data
                    pcd = self.display_queue.queue[0]
                    return depth_frame, color_frame, rgb_img, pcd
                return None
                
        except Exception as e:
            print(f"获取显示帧时出错: {str(e)}")
            return None

    def start_processing(self, model, args):
        """启动所有处理线程"""
        self.is_running = True
        
        # 启动采集线程
        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        # 启动处理线程
        self.process_thread = threading.Thread(target=self.process_frames, args=(model, args))
        self.process_thread.daemon = True
        self.process_thread.start()

    def stop_processing(self):
        """停止所有处理线程"""
        self.is_running = False
        
        if self.capture_thread:
            self.capture_thread.join(timeout=1.0)
        if self.process_thread:
            self.process_thread.join(timeout=1.0)
            
        self.release()

    def release(self):
        """释放资源"""
        try:
            if self.pipeline is not None:
                print("🔄 正在释放RealSense资源...")
                self.pipeline.stop()
                self.pipeline = None
                print("✅ RealSense资源释放完成")
        except Exception as e:
            print(f"⚠️ 释放资源时出错: {e}")
        finally:
            self.pipeline = None

    def calculate_object_position(self, pcd):
        """计算框选区域内点云的空间位置并转换为机械臂坐标"""
        # 获取点云数据
        points = np.asarray(pcd.points)
        colors = np.asarray(pcd.colors)
        
        # 找出红色点的索引 (RGB值接近[1,0,0])
        red_mask = (colors[:, 0] > 0.8) & (colors[:, 1] < 0.2) & (colors[:, 2] < 0.2)
        red_points = points[red_mask]
        
        if len(red_points) < 3:  # 如果点太少，返回None
            print(f"框选区域内有效点数太少: {len(red_points)}")
            return None, None
        
        # 计算有效点的统计信息
        # 使用百分位数去除异常值
        percentile_low, percentile_high = 10, 90
        x_filtered = np.percentile(red_points[:, 0], [percentile_low, percentile_high])
        y_filtered = np.percentile(red_points[:, 1], [percentile_low, percentile_high])
        z_filtered = np.percentile(red_points[:, 2], [percentile_low, percentile_high])
        
        # 在过滤范围内的点
        mask = ((red_points[:, 0] >= x_filtered[0]) & (red_points[:, 0] <= x_filtered[1]) &
                (red_points[:, 1] >= y_filtered[0]) & (red_points[:, 1] <= y_filtered[1]) &
                (red_points[:, 2] >= z_filtered[0]) & (red_points[:, 2] <= z_filtered[1]))
        
        filtered_points = red_points[mask]
        
        if len(filtered_points) < 3:
            print("过滤后的有效点太少")
            return None, None
        
        # 计算中心点（使用均值）
        center_point = np.mean(filtered_points, axis=0)
        
        # 获取机械臂当前位置
        ROBOT_IP = "192.168.1.18"
        controller = RMJointRobotController(ROBOT_IP)
        if controller.connect():
            get_cmd = {"command": "get_current_arm_state"}
            response = controller.send_command(get_cmd)
            
            if response and "arm_state" in response:
                current_pose = response["arm_state"]["pose"]
                # 提取当前位置（单位转换：0.000001m -> m）
                X = current_pose[0] * 0.000001
                Y = current_pose[1] * 0.000001
                Z = current_pose[2] * 0.000001

                print(X)
                print(Y)
                print(Z)
                
                # 计算目标位置
                x1, y1, z1 = center_point
                print(x1)
                print(y1)
                print(z1)
                X_new = X - x1 + 0.003884
                Y_new = Y + y1 - 0.09146
                Z_new = Z - z1 + 0.128848
                
                # 转换回机械臂单位（m -> 0.000001m）并保持整数
                X_new = int(X_new * 1000000)
                Y_new = int(Y_new * 1000000)
                Z_new = int(Z_new * 1000000)
                
                # 输出新的位姿
                print(f'"pose": [{X_new}, {Y_new}, {Z_new}, 3142, 0, -523]')
                
                return center_point.tolist(), None  # 不再需要返回relative_position
        
        return center_point.tolist(), None

class Point_geometry:
    def __init__(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.vis.get_render_option().point_size = 1.0
        self.vis.get_render_option().background_color = np.array([0.5, 0.5, 0.5])
        self.current_scene = None
        self.coordinate_frame = None

    def update_geometry(self, pcd):
        # 移除旧的点云
        if self.current_scene is not None:
            view_control = self.vis.get_view_control()
            camera_params = view_control.convert_to_pinhole_camera_parameters()
            self.vis.remove_geometry(self.current_scene, False)
            
            # 添加新的坐标系和点云
            self.current_scene = pcd
            self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3, origin=[0, 0, 0])
            self.vis.add_geometry(self.current_scene, False)
            self.vis.poll_events()
            self.vis.update_renderer()
        else:
            # 添加新的坐标系和点云
            self.current_scene = pcd
            self.coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3, origin=[0, 0, 0])
            self.vis.add_geometry(self.current_scene)

            self.vis.poll_events()
            self.vis.update_renderer()
            # 重置视角
            ctr = self.vis.get_view_control()
            ctr.set_front([0, 0, -1])  # 设置相机朝向（沿Z轴负方向）
            ctr.set_lookat([0, 0, 0])  # 设置观察点（原点）
            ctr.set_up([0, -1, 0])  # 设置相机上方向

def main(args):
    # 禁用所有警告
    import warnings
    warnings.simplefilter("ignore")
    
    print("🚀 启动RealSense RGB框选+高效点云生成器")
    print("=" * 50)
    
    # 检查相机可用性
    print("🔍 检查相机可用性...")
    if not StereoCamera.check_camera_availability():
        print("❌ 相机不可用，请检查：")
        print("   1. RealSense相机是否正确连接")
        print("   2. 是否有其他程序正在使用相机")
        print("   3. USB连接是否稳定")
        print("   4. RealSense驱动是否正确安装")
        return
    
    # 创建相机对象
    camera = StereoCamera()
    camera.set_fps(args.fps)
    
    # 现在使用RealSense内置深度算法，无需加载外部模型
    print("📊 使用RealSense内置深度算法，无需加载外部模型")

    # 创建窗口
    cv2.namedWindow('RGB View for Selection', cv2.WINDOW_NORMAL)  # RGB图像用于框选
    cv2.namedWindow('Depth and RGB', cv2.WINDOW_NORMAL)        # 深度和RGB图像
    cv2.setMouseCallback('RGB View for Selection', camera.mouse_callback)
    vis_scene = Point_geometry()

    try:
        camera.start_processing(None, args)  # 不需要传递模型
        
        while True:
            frame_data = camera.get_display_frame()
            if frame_data is not None:
                depth_frame, color_frame, rgb_img, pcd = frame_data
                
                # 准备RGB图像显示（用于框选）
                rgb_display = rgb_img.copy()
                if camera.drawing and camera.start_point and camera.end_point:
                    cv2.rectangle(rgb_display, camera.start_point, camera.end_point, (0, 255, 0), 2)
                elif camera.bbox and camera.bbox.active:
                    cv2.rectangle(rgb_display, 
                                (camera.bbox.x1, camera.bbox.y1),
                                (camera.bbox.x2, camera.bbox.y2),
                                (0, 255, 0), 2)

                # 显示RGB图像（用于框选）
                cv2.imshow('RGB View for Selection', rgb_display)

                # 显示深度图和RGB图像
                # 将深度图转换为可视化格式
                if depth_frame:
                    depth_image = np.asanyarray(depth_frame.get_data())
                    # 深度图归一化到0-255范围用于显示
                    depth_colormap = cv2.applyColorMap(
                        cv2.convertScaleAbs(depth_image, alpha=0.03), 
                        cv2.COLORMAP_JET
                    )
                else:
                    depth_colormap = np.zeros((480, 640, 3), dtype=np.uint8)
                
                # 组合深度图和RGB图像
                combined = np.hstack([depth_colormap, rgb_img])
                cv2.putText(combined, f"Target FPS: {camera.fps}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(combined, "Depth                                RGB", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1)
                cv2.imshow('Depth and RGB', combined)
                
                # 更新点云显示
                vis_scene.update_geometry(pcd)

            key = cv2.waitKey(1)
            if key == ord('q'):
                camera.is_running = False
                break
            elif key == ord('c'):  # 清除框选
                camera.bbox = None
            elif key == ord('s'):  # 保存当前RGB图像
                if 'rgb_img' in locals():
                    os.makedirs('static', exist_ok=True)
                    cv2.imwrite('static/current_rgb.jpg', rgb_img)
                    print("已保存当前RGB图像到 static/current_rgb.jpg")

            time.sleep(1.0 / camera.fps)

    finally:
        camera.stop_processing()
        cv2.destroyAllWindows()

def parse_args():
    parser = argparse.ArgumentParser(description='RealSense RGB框选+高效点云生成器')
    parser.add_argument('--fps', default=10, type=int, help='相机帧率')
    parser.add_argument('--z_far', default=3, type=float, help='点云中最大深度的裁剪值')
    parser.add_argument('--denoise_cloud', type=int, default=0, help='是否对点云进行去噪')
    parser.add_argument('--denoise_nb_points', type=int, default=20, help='去噪时的邻域点数')

    args = parser.parse_args()
    return args

if __name__ == '__main__':
    args = parse_args()
    main(args) 