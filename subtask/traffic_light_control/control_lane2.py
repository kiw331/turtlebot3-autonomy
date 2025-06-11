#!/usr/bin/env python3

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String

class ControlLane(Node):

    def __init__(self):
        super().__init__('control_lane')

        self.sub_lane = self.create_subscription(
            Float64,
            '/control/lane',
            self.callback_follow_lane,
            1
        )
        self.sub_max_vel = self.create_subscription(
            Float64,
            '/control/max_vel',
            self.callback_get_max_vel,
            1
        )
        self.sub_avoid_cmd = self.create_subscription(
            Twist,
            '/avoid_control',
            self.callback_avoid_cmd,
            1
        )
        self.sub_avoid_active = self.create_subscription(
            Bool,
            '/avoid_active',
            self.callback_avoid_active,
            1
        )

        # 신호등 색상 구독
        self.sub_traffic_light = self.create_subscription(
            String,
            '/detect/traffic_light_color',
            self.callback_traffic_light,
            1
        )

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/control/cmd_vel',
            1
        )

        self.last_error = 0
        self.MAX_VEL = 0.1
        self.avoid_active = False
        self.avoid_twist = Twist()

        # 신호등 상태
        self.is_stop_due_to_traffic = False  # 신호등으로 정지 중인지
        self.traffic_light_colors = []       # 감지된 색상 리스트

    def callback_get_max_vel(self, max_vel_msg):
        self.MAX_VEL = max_vel_msg.data

    def callback_traffic_light(self, msg):
        # String 형태로 들어옴 → ex: 'RED,YELLOW', 'GREEN', 'NONE'
        colors = msg.data.split(',') if msg.data != 'NONE' else []
        self.traffic_light_colors = colors
        self.get_logger().info(f'Traffic light colors: {self.traffic_light_colors}')

        # 로직:
        if 'RED' in colors or 'YELLOW' in colors:
            self.is_stop_due_to_traffic = True  # 정지 유지
        elif 'GREEN' in colors:
            self.is_stop_due_to_traffic = False  # 녹색이면 주행 재개

    def callback_follow_lane(self, desired_center):
        if self.avoid_active:
            return

        # 신호등 제어
        if self.is_stop_due_to_traffic:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.pub_cmd_vel.publish(twist)
            self.get_logger().info('Stopping due to traffic light (RED/YELLOW)')
            return

        # 정상 차선 주행
        center = desired_center.data
        error = center - 500

        Kp = 0.0025
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        twist = Twist()
        twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), 0.05)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish(twist)

    def callback_avoid_cmd(self, twist_msg):
        self.avoid_twist = twist_msg

        if self.avoid_active:
            self.pub_cmd_vel.publish(self.avoid_twist)

    def callback_avoid_active(self, bool_msg):
        self.avoid_active = bool_msg.data
        if self.avoid_active:
            self.get_logger().info('Avoidance mode activated.')
        else:
            self.get_logger().info('Avoidance mode deactivated. Returning to lane following.')

    def shut_down(self):
        self.get_logger().info('Shutting down. cmd_vel will be 0')
        twist = Twist()
        self.pub_cmd_vel.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shut_down()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
