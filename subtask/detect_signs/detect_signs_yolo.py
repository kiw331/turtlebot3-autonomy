#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import torch
import cv2
import numpy as np
import os

from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

from ament_index_python.packages import get_package_share_directory

class DetectSignsYolo(Node):
    def __init__(self):
        super().__init__('detect_signs_yolo')

        # 패키지 경로 찾기
        package_name = 'turtlebot3_autorace_detect'
        package_path = get_package_share_directory(package_name)
        model_relative_path = 'resource/best.pt'
        model_path = os.path.join(package_path, model_relative_path)

        if not os.path.exists(model_path):
            self.get_logger().error(f"모델 파일을 찾을 수 없습니다: {model_path}")
            return

        self.get_logger().info(f"YOLO 모델 로드 중: {model_path}")
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=False)
        self.model.conf = 0.4  # confidence threshold (원하는대로 조정 가능)
        self.get_logger().info("YOLO 모델 로드 완료")

        self.bridge = CvBridge()

        # 최소 박스 크기 (픽셀 제곱 기준, 실험 통해 튜닝 필요)
        self.min_box_area = 3000

        # 클래스 이름 미리 정의 (참고용)
        self.classes = ['left_turn', 'right_turn', 'stop', 'speed_30', 'speed_40', 'speed_50']

        # 카메라 이미지 구독
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_compensated',
            self.image_callback,
            1
        )

        # 결과 퍼블리시
        self.pub_sign = self.create_publisher(
            String,
            '/detect/sign',
            1
        )

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        results = self.model(cv_image)
        detections = results.pandas().xyxy[0]  # pandas dataframe

        if detections.empty:
            self.publish_none()
            return

        # 박스 면적 계산
        detections['area'] = (detections['xmax'] - detections['xmin']) * (detections['ymax'] - detections['ymin'])
        detections = detections[detections['area'] >= self.min_box_area]

        if detections.empty:
            self.publish_none()
            return

        # 가장 큰 박스 선택
        largest_detection = detections.loc[detections['area'].idxmax()]
        label = largest_detection['name']

        if label not in self.classes:
            self.get_logger().warn(f"알 수 없는 클래스: {label}")
            self.publish_none()
            return

        # 결과 퍼블리시
        msg_out = String()
        msg_out.data = label
        self.pub_sign.publish(msg_out)

        self.get_logger().info(f"Detected sign: {label} (area: {largest_detection['area']})")

    def publish_none(self):
        msg_out = String()
        msg_out.data = 'None'
        self.pub_sign.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = DetectSignsYolo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
