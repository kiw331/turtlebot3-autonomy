import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import sys

class ImageSaver(Node):
    def __init__(self, prefix='sign'):
        super().__init__('image_saver')
        self.prefix = prefix
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.image_count = 1  # 1부터 시작

        self.save_dir = '/home/kiwi/Desktop/signs/sign_stop'
        os.makedirs(self.save_dir, exist_ok=True)

        self.latest_frame = None
        self.is_frame_available = False

        # 0.1초 간격 타이머
        self.timer = self.create_timer(0.4, self.timer_callback)

    def listener_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        self.latest_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.is_frame_available = True

    def timer_callback(self):
        if self.is_frame_available and self.latest_frame is not None:
            filename = os.path.join(self.save_dir, f'{self.prefix}{self.image_count}.png')
            cv2.imwrite(filename, self.latest_frame)
            self.get_logger().info(f'Saved {filename}')
            self.image_count += 1
            self.is_frame_available = False

def main(args=None):
    rclpy.init(args=args)

    # 실행시 prefix 인자 받기 (기본값: sign)
    prefix = 'sign_stop'
    if len(sys.argv) > 1:
        prefix = sys.argv[1]

    node = ImageSaver(prefix=prefix)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
