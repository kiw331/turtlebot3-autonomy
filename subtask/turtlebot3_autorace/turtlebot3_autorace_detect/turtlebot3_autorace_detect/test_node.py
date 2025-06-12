#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class DummySignPublisher(Node):
    def __init__(self):
        super().__init__('dummy_sign_publisher')
        self.publisher_ = self.create_publisher(String, '/detect/sign', 10)

        # 최초 퍼블리시 대기 시간
        self.declare_parameter('delay', 2.0)
        self.delay = self.get_parameter('delay').get_parameter_value().double_value

        # 표지판 종류도 파라미터화 가능
        self.declare_parameter('sign_label', 'left_turn')
        self.sign_label = self.get_parameter('sign_label').get_parameter_value().string_value

        self.get_logger().info(f"Waiting {self.delay} seconds before publishing '{self.sign_label}' sign...")
        self.timer = self.create_timer(self.delay, self.publish_sign)

    def publish_sign(self):
        msg = String()
        msg.data = self.sign_label
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published: {self.sign_label}")
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = DummySignPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
