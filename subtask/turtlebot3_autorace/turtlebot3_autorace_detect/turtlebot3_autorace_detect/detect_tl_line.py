#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool

import time

class DetectTrafficLightLine(Node):
    def __init__(self):
        super().__init__('detect_tl_line')

        # 이미지 구독 (compensated 이미지 사용)
        self.sub_image = self.create_subscription(
            Image,
            '/camera/image_compensated',
            self.image_callback,
            1
        )

        # 정지선 감지 결과 퍼블리시
        self.pub_stop_line = self.create_publisher(
            Bool,
            '/detect/stop_line',
            1
        )

        self.bridge = CvBridge()

        # 임계값 기본 설정 (나중에 파라미터로 뺄 수도 있음)
        self.threshold = 1000

    def image_callback(self, msg):
        # ROS Image → OpenCV 이미지 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # 이미지 전처리 (하단 1/2 ROI)
        height, width, _ = frame.shape
        roi = frame[int(height/2):, :]

        # HSV 변환
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # 빨간색 마스크 범위 정의 (예시값, 조정 필요)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 빨간색 픽셀 개수 계산
        red_pixels = cv2.countNonZero(mask)

        # 감지 판정
        detected = red_pixels > self.threshold

        # 결과 퍼블리시
        msg_stop_line = Bool()
        msg_stop_line.data = detected
        self.pub_stop_line.publish(msg_stop_line)

        # 픽셀수 로그 출력
        self.get_logger().info(f"Red pixels: {red_pixels}, Detected: {detected}")
        time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = DetectTrafficLightLine()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
