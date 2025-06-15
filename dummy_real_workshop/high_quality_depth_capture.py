import os
import sys
import numpy as np
import cv2
import pyrealsense2 as rs
import open3d as o3d
import time
from typing import Tuple, Optional
import json

class HighQualityDepthCapture:
    def __init__(self, 
                 frames_to_average: int = 30,
                 stabilization_time: float = 3.0,
                 depth_filter_magnitude: int = 2,
                 temporal_alpha: float = 0.4,
                 spatial_alpha: float = 0.4):
        """
        初始化高质量深度图采集器
        
        Args:
            frames_to_average: 用于平均的帧数
            stabilization_time: 相机稳定等待时间（秒）
            depth_filter_magnitude: 深度滤波器强度
            temporal_alpha: 时间滤波器平滑系数
            spatial_alpha: 空间滤波器平滑系数
        """
        self.frames_to_average = frames_to_average
        self.stabilization_time = stabilization_time
        self.depth_filter_magnitude = depth_filter_magnitude
        self.temporal_alpha = temporal_alpha
        self.spatial_alpha = spatial_alpha
        
        # RealSense相关对象
        self.pipeline = None
        self.config = None
        self.align = None
        self.depth_scale = None
        
        # 点云处理对象
        self.pc = rs.pointcloud()
        
        # 滤波器链
        self.filters = None
        
        # 输出目录
        self.output_dir = "captured_data"
        os.makedirs(self.output_dir, exist_ok=True)
        
    def setup_filters(self) -> None:
        """设置深度图后处理滤波器链"""
        try:
            # 1. 视差变换（可选，如果导致问题可以注释掉）
            # self.disparity_transform = rs.disparity_transform(True)
            # self.depth_to_disparity = rs.disparity_transform(False)
            
            # 2. 空间滤波器（降低强度以提高稳定性）
            self.spatial = rs.spatial_filter()
            self.spatial.set_option(rs.option.filter_magnitude, 1)  # 降低强度
            self.spatial.set_option(rs.option.filter_smooth_alpha, 0.3)  # 降低平滑程度
            self.spatial.set_option(rs.option.filter_smooth_delta, 10)  # 降低增量
            
            # 3. 时序滤波器（降低强度）
            self.temporal = rs.temporal_filter()
            self.temporal.set_option(rs.option.filter_smooth_alpha, 0.2)  # 降低平滑程度
            self.temporal.set_option(rs.option.filter_smooth_delta, 10)
            
            # 4. 孔洞填充滤波器
            self.hole_filling = rs.hole_filling_filter()
            
            # 简化滤波器链，移除可能导致问题的滤波器
            self.filters = [
                # self.disparity_transform,  # 暂时注释掉可能导致问题的滤波器
                self.spatial,
                self.temporal,
                # self.depth_to_disparity,  # 暂时注释掉可能导致问题的滤波器
                self.hole_filling
            ]
            
            print("滤波器链设置成功")
            
        except Exception as e:
            print(f"设置滤波器链时出错: {str(e)}")
            self.filters = []  # 如果出错，使用空滤波器链
            
    def init_camera(self) -> bool:
        """初始化RealSense相机"""
        try:
            print("正在初始化RealSense相机...")
            self.pipeline = rs.pipeline()
            self.config = rs.config()
            
            # 配置深度和RGB流
            self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            
            # 启动流
            profile = self.pipeline.start(self.config)
            
            # 获取深度传感器
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            
            # 设置深度传感器选项
            if depth_sensor.supports(rs.option.visual_preset):
                depth_sensor.set_option(rs.option.visual_preset, 4)  # 高精度预设
            
            if depth_sensor.supports(rs.option.laser_power):
                depth_sensor.set_option(rs.option.laser_power, 360)  # 最大激光功率
                
            # 创建对齐对象
            self.align = rs.align(rs.stream.color)
            
            # 设置滤波器链
            self.setup_filters()
            
            print("相机初始化成功！")
            print(f"深度比例: {self.depth_scale}")
            return True
            
        except Exception as e:
            print(f"相机初始化失败: {str(e)}")
            return False
            
    def apply_filters(self, depth_frame: rs.frame) -> rs.frame:
        """应用滤波器链处理深度帧"""
        if not self.filters:
            return depth_frame
            
        filtered = depth_frame
        try:
            for filter_obj in self.filters:
                if filtered is None:
                    return depth_frame
                filtered = filter_obj.process(filtered)
            return filtered
        except Exception as e:
            print(f"应用滤波器时出错: {str(e)}")
            return depth_frame  # 如果滤波失败，返回原始帧
        
    def wait_for_camera_stable(self) -> None:
        """等待相机稳定"""
        print(f"\n等待相机稳定 ({self.stabilization_time}秒)...")
        start_time = time.time()
        
        while time.time() - start_time < self.stabilization_time:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                remaining = int(self.stabilization_time - (time.time() - start_time))
                print(f"\r剩余稳定时间: {remaining}秒", end="", flush=True)
                time.sleep(0.1)
            except Exception:
                pass
                
        print("\n相机已稳定！")
        
    def capture_high_quality_depth(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        采集高质量深度图和RGB图像
        
        Returns:
            tuple: (depth_image, color_image) 如果成功
                  (None, None) 如果失败
        """
        try:
            print(f"\n开始采集 {self.frames_to_average} 帧进行平均...")
            
            # 用于存储多帧数据
            depth_frames = []
            last_color_frame = None
            
            # 采集多帧
            for i in range(self.frames_to_average):
                try:
                    frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                    if not frames:
                        print(f"第 {i+1} 帧采集超时")
                        continue
                        
                    aligned_frames = self.align.process(frames)
                    
                    depth_frame = aligned_frames.get_depth_frame()
                    color_frame = aligned_frames.get_color_frame()
                    
                    if not depth_frame or not color_frame:
                        print(f"第 {i+1} 帧数据无效")
                        continue
                        
                    # 应用滤波器链
                    try:
                        filtered_depth = self.apply_filters(depth_frame)
                        if filtered_depth is None:
                            print(f"第 {i+1} 帧滤波失败")
                            continue
                    except Exception as e:
                        print(f"处理第 {i+1} 帧时出错: {str(e)}")
                        continue
                    
                    # 转换为numpy数组
                    depth_image = np.asanyarray(filtered_depth.get_data())
                    
                    # 检查深度图是否有效
                    if np.all(depth_image == 0) or not np.any(np.isfinite(depth_image)):
                        print(f"第 {i+1} 帧深度图无效")
                        continue
                    
                    depth_frames.append(depth_image)
                    last_color_frame = np.asanyarray(color_frame.get_data())
                    
                    print(f"\r已采集: {i+1}/{self.frames_to_average}", end="", flush=True)
                    
                except Exception as e:
                    print(f"\n处理第 {i+1} 帧时出错: {str(e)}")
                    continue
                
            print("\n帧采集完成，正在处理...")
            
            if not depth_frames:
                print("没有采集到有效帧")
                return None, None
                
            if len(depth_frames) < self.frames_to_average * 0.5:
                print(f"有效帧数太少（{len(depth_frames)}/{self.frames_to_average}）")
                return None, None
                
            # 计算深度图的中值
            depth_stack = np.stack(depth_frames, axis=0)
            median_depth = np.median(depth_stack, axis=0).astype(np.uint16)
            
            # 移除异常值（使用更保守的阈值）
            depth_mask = np.abs(depth_stack - median_depth) > 200  # 降低阈值
            depth_stack[depth_mask] = median_depth[depth_mask[0]]
            
            # 计算平均深度图
            final_depth = np.mean(depth_stack, axis=0).astype(np.uint16)
            
            # 验证最终深度图
            if np.all(final_depth == 0) or not np.any(np.isfinite(final_depth)):
                print("生成的深度图无效")
                return None, None
                
            print(f"成功处理 {len(depth_frames)} 帧数据")
            return final_depth, last_color_frame
            
        except Exception as e:
            print(f"采集过程出错: {str(e)}")
            import traceback
            traceback.print_exc()
            return None, None
            
    def generate_point_cloud(self, 
                           depth_image: np.ndarray, 
                           color_image: np.ndarray,
                           save_results: bool = True) -> o3d.geometry.PointCloud:
        """
        从深度图和RGB图像生成点云
        
        Args:
            depth_image: 深度图
            color_image: RGB图像
            save_results: 是否保存结果
            
        Returns:
            o3d.geometry.PointCloud: 生成的点云对象
        """
        # 创建Open3D深度图和RGB图像
        depth = o3d.geometry.Image(depth_image)
        color = o3d.geometry.Image(color_image)
        
        # 创建RGBD图像
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
            color, depth,
            depth_scale=1.0/self.depth_scale,
            depth_trunc=5.0,
            convert_rgb_to_intensity=False
        )
        
        # 设置相机内参
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            width=640, height=480,
            fx=595.0, fy=595.0,
            cx=320.0, cy=240.0
        )
        
        # 生成点云
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
            rgbd, intrinsic
        )
        
        # 移除无效点
        pcd = pcd.remove_non_finite_points()
        
        # 统计滤波去除离群点
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=20,
            std_ratio=2.0
        )
        
        if save_results:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            
            # 保存点云
            o3d.io.write_point_cloud(
                f"{self.output_dir}/cloud_{timestamp}.ply",
                pcd
            )
            
            # 保存深度图和RGB图像
            cv2.imwrite(
                f"{self.output_dir}/depth_{timestamp}.png",
                cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.03),
                    cv2.COLORMAP_JET
                )
            )
            
            cv2.imwrite(
                f"{self.output_dir}/rgb_{timestamp}.jpg",
                color_image
            )
            
            # 保存相机参数
            params = {
                "intrinsic": {
                    "width": 640,
                    "height": 480,
                    "fx": 595.0,
                    "fy": 595.0,
                    "cx": 320.0,
                    "cy": 240.0
                },
                "depth_scale": self.depth_scale,
                "timestamp": timestamp
            }
            
            with open(f"{self.output_dir}/params_{timestamp}.json", "w") as f:
                json.dump(params, f, indent=4)
                
            print(f"\n结果已保存到 {self.output_dir} 目录")
            
        return pcd
        
    def visualize_results(self, 
                         depth_image: np.ndarray, 
                         color_image: np.ndarray,
                         point_cloud: o3d.geometry.PointCloud) -> None:
        """可视化结果"""
        # 创建窗口
        cv2.namedWindow("深度图", cv2.WINDOW_NORMAL)
        cv2.namedWindow("RGB图像", cv2.WINDOW_NORMAL)
        
        # 显示深度图
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )
        cv2.imshow("深度图", depth_colormap)
        
        # 显示RGB图像
        cv2.imshow("RGB图像", color_image)
        
        # 显示点云
        o3d.visualization.draw_geometries([
            point_cloud,
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.3)
        ])
        
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
    def capture_and_process(self, visualize: bool = True) -> None:
        """执行完整的采集和处理流程"""
        try:
            # 初始化相机
            if not self.init_camera():
                return
                
            # 等待相机稳定
            self.wait_for_camera_stable()
            
            # 采集高质量深度图
            depth_image, color_image = self.capture_high_quality_depth()
            
            if depth_image is None or color_image is None:
                print("采集失败！")
                return
                
            # 生成点云
            point_cloud = self.generate_point_cloud(depth_image, color_image)
            
            # 可视化结果
            if visualize:
                self.visualize_results(depth_image, color_image, point_cloud)
                
        except Exception as e:
            print(f"处理过程出错: {str(e)}")
            
        finally:
            if self.pipeline:
                self.pipeline.stop()
                
    def release(self):
        """释放资源"""
        if self.pipeline:
            self.pipeline.stop()
            
class ResultVisualizer:
    def __init__(self):
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window("点云可视化", width=1280, height=720)
        
        # 设置渲染选项
        render_option = self.vis.get_render_option()
        render_option.point_size = 2.0
        render_option.background_color = np.array([0.1, 0.1, 0.1])
        render_option.show_coordinate_frame = True
        
        # 获取视图控制
        self.view_control = self.vis.get_view_control()
        
    def setup_camera(self):
        """设置默认相机视角"""
        self.view_control.set_zoom(0.4)
        self.view_control.set_front([0, 0, -1])
        self.view_control.set_lookat([0, 0, 0])
        self.view_control.set_up([0, -1, 0])
        
    def visualize_results(self, 
                         depth_image: np.ndarray, 
                         color_image: np.ndarray,
                         point_cloud: o3d.geometry.PointCloud) -> None:
        """交互式可视化结果"""
        # 创建深度图和RGB图像窗口
        cv2.namedWindow("深度图", cv2.WINDOW_NORMAL)
        cv2.namedWindow("RGB图像", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("深度图", 640, 480)
        cv2.resizeWindow("RGB图像", 640, 480)
        
        # 显示深度图
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03),
            cv2.COLORMAP_JET
        )
        cv2.imshow("深度图", depth_colormap)
        
        # 显示RGB图像
        cv2.imshow("RGB图像", color_image)
        
        # 添加坐标系
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=0.3, origin=[0, 0, 0]
        )
        
        # 清除之前的几何体
        self.vis.clear_geometries()
        
        # 添加点云和坐标系
        self.vis.add_geometry(point_cloud)
        self.vis.add_geometry(coordinate_frame)
        
        # 设置默认视角
        self.setup_camera()
        
        print("\n=== 可视化控制说明 ===")
        print("- [鼠标左键] 旋转视角")
        print("- [鼠标右键] 平移视角")
        print("- [鼠标滚轮] 缩放视角")
        print("- [Shift + 左键] 改变环境光")
        print("- 按 [Q] 退出可视化")
        print("==================")
        
        # 交互式显示
        while True:
            # 更新可视化
            self.vis.poll_events()
            self.vis.update_renderer()
            
            # 检查OpenCV窗口按键
            key = cv2.waitKey(1)
            if key == ord('q') or key == ord('Q'):
                break
                
        # 清理资源
        cv2.destroyAllWindows()
        self.vis.destroy_window()
        
    def __del__(self):
        """析构函数，确保资源被释放"""
        try:
            self.vis.destroy_window()
        except:
            pass

def main():
    # 创建采集器实例
    capturer = HighQualityDepthCapture(
        frames_to_average=30,          # 平均30帧
        stabilization_time=3.0,        # 等待3秒让相机稳定
        depth_filter_magnitude=2,      # 深度滤波器强度
        temporal_alpha=0.4,            # 时间滤波器平滑系数
        spatial_alpha=0.4              # 空间滤波器平滑系数
    )
    
    try:
        # 初始化相机
        if not capturer.init_camera():
            return
            
        # 等待相机稳定
        capturer.wait_for_camera_stable()
        
        # 采集高质量深度图
        depth_image, color_image = capturer.capture_high_quality_depth()
        
        if depth_image is None or color_image is None:
            print("采集失败！")
            return
            
        # 生成点云
        point_cloud = capturer.generate_point_cloud(depth_image, color_image)
        
        # 创建可视化器并显示结果
        visualizer = ResultVisualizer()
        visualizer.visualize_results(depth_image, color_image, point_cloud)
        
    except Exception as e:
        print(f"处理过程出错: {str(e)}")
        import traceback
        traceback.print_exc()
        
    finally:
        capturer.release()

if __name__ == "__main__":
    main() 