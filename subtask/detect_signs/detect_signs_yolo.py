#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
import os
import json
import time

from ultralytics import YOLO
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

class DetectSignsYolo(Node):
    def __init__(self):
        super().__init__('detect_signs_yolo')

        package_name = 'turtlebot3_autorace_detect'
        package_path = get_package_share_directory(package_name)

        model_relative_path = 'resource/yolo11_v1.pt'
        model_path = os.path.join(package_path, model_relative_path)

        if not os.path.exists(model_path):
            self.get_logger().error(f"모델 파일을 찾을 수 없습니다: {model_path}")
            return

        self.get_logger().info(f"Ultralytics YOLO 모델 로드 중: {model_path}")
        self.model = YOLO(model_path)
        self.get_logger().info("YOLO 모델 로드 완료")

        self.bridge = CvBridge()
        self.min_box_area = 1100

        json_relative_path = 'resource/class_name.json'
        json_path = os.path.join(package_path, json_relative_path)
        if not os.path.exists(json_path):
            self.get_logger().error(f"클래스 매핑 JSON 파일을 찾을 수 없습니다: {json_path}")
            self.class_mapping = {}
        else:
            with open(json_path, 'r') as f:
                self.class_mapping = json.load(f)
            self.get_logger().info(f"클래스 매핑 로드 완료: {self.class_mapping}")

        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_compensated',
            self.image_callback,
            1
        )

        self.pub_sign = self.create_publisher(
            String,
            '/detect/sign',
            1
        )

        self.pub_sign_image = self.create_publisher(
            Image,
            '/detect/sign_image',
            1
        )

        # 타이머 기반으로 이미지 처리 빈도 제한
        self.last_image = None
        self.image_received = False
        self.timer = self.create_timer(0.1, self.process_image)

    def image_callback(self, msg):
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            self.image_received = False

    def process_image(self):
        if not self.image_received or self.last_image is None:
            return

        cv_image = self.last_image.copy()
        self.image_received = False  # 중복 처리 방지

        results = self.model.predict(source=cv_image, conf=0.8, verbose=False)
        result = results[0]

        boxes = result.boxes.xyxy.cpu().numpy()
        confidences = result.boxes.conf.cpu().numpy()
        class_ids = result.boxes.cls.cpu().numpy()

        if len(boxes) == 0:
            self.get_logger().info("YOLO 검출 결과 없음")
            self.publish_none()
            self.publish_image(cv_image)
            return

        areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
        valid_indices = np.where(areas >= self.min_box_area)[0]

        if len(valid_indices) == 0:
            self.get_logger().info("면적 필터 이후 유효한 박스 없음")
            self.publish_none()
            self.publish_image(cv_image)
            return

        largest_idx = valid_indices[np.argmax(areas[valid_indices])]
        class_idx = int(class_ids[largest_idx])
        label = self.class_mapping.get(str(class_idx), None)

        if label is None:
            self.get_logger().warn(f"알 수 없는 클래스 index: {class_idx}")
            self.publish_none()
        else:
            msg_out = String()
            msg_out.data = label
            self.pub_sign.publish(msg_out)
            self.get_logger().info(f"Detected sign: {label} (area: {areas[largest_idx]:.1f})")

        annotated_image = cv_image.copy()
        for idx in valid_indices:
            x1, y1, x2, y2 = boxes[idx].astype(int)
            conf = confidences[idx]
            class_idx = int(class_ids[idx])
            class_name = self.class_mapping.get(str(class_idx), 'Unknown')
            cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            label_text = f"{class_name} {conf:.2f}"
            cv2.putText(annotated_image, label_text, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        self.publish_image(annotated_image)

    def publish_none(self):
        msg_out = String()
        msg_out.data = 'None'
        self.pub_sign.publish(msg_out)

    def publish_image(self, image):
        img_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        self.pub_sign_image.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DetectSignsYolo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
