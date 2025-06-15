from ultralytics import YOLO
from PIL import Image
import numpy as np
import cv2  # 添加OpenCV用于高效图像处理

# 加载模型并配置任务类型
model = YOLO("yoloe-11s-seg.pt", task='segment')

# 设置自定义检测类别
model.set_classes(["A red cube"], model.get_text_pe(["A red cube"]))


# 直接读取灰度图并预处理
def load_and_preprocess_grayscale(image_path):
    """直接读取单通道灰度图并转换为三通道格式"""
    # 使用OpenCV高效读取单通道灰度图
    gray_img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)

    # 转换为伪三通道（YOLO必需的三通道格式）
    pseudo_rgb = cv2.merge([gray_img, gray_img, gray_img])

    # 转换为PIL格式（与原始代码兼容）
    return Image.fromarray(pseudo_rgb)

def load_and_preprocess_rgb(image_path):
    """直接读取三通道RGB图并转换为单通道灰度图"""
    rgb_img = cv2.imread(image_path)
    return Image.fromarray(rgb_img)


# 执行灰度图识别流程
gray_image = load_and_preprocess_rgb("/mnt/mydisk/My_project/robot_arm/Dummy_V2_ren/dummy_workshop/vitual_img.png")

# 执行推理（保持原始参数）
results = model.predict(gray_image, conf=0.5)
