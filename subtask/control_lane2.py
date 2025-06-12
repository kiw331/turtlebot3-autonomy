#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, String
import time

class ControlLane(Node):
    def __init__(self):
        super().__init__('control_lane')

        # 구독자
        self.sub_lane = self.create_subscription(Float64, '/control/lane', self.callback_follow_lane, 1)
        self.sub_max_vel = self.create_subscription(Float64, '/control/max_vel', self.callback_get_max_vel, 1)
        self.sub_avoid_cmd = self.create_subscription(Twist, '/avoid_control', self.callback_avoid_cmd, 1)
        self.sub_avoid_active = self.create_subscription(Bool, '/avoid_active', self.callback_avoid_active, 1)
        self.sub_traffic_light = self.create_subscription(String, '/detect/traffic_light_color', self.callback_traffic_light, 1)
        self.sub_stop_line = self.create_subscription(Bool, '/detect/stop_line', self.callback_stop_line, 1)
        self.sub_sign = self.create_subscription(String, '/detect/sign', self.callback_sign, 1)

        self.sub_level_crossing = self.create_subscription(
            String, '/detect/level_crossing_state', self.callback_level_crossing, 1) # 차단바


        # 퍼블리셔
        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 1)

        # 내부 변수
        self.last_error = 0
        self.base_max_vel = 0.1
        self.MAX_VEL = self.base_max_vel
        self.avoid_active = False
        self.avoid_twist = Twist()

        self.state = 'NORMAL'
        self.turn_direction = None  # 'LEFT' or 'RIGHT'

        self.stop_line_suppression_until = 0
        self.suppression_duration = 3.0

        self.sign_suppression_until = 0
        self.sign_suppression_duration = 5.0

        self.turn_start_time = 0
        self.turn_duration = 2.0  # soft turn 유지 시간

    def callback_get_max_vel(self, max_vel_msg):
        self.base_max_vel = max_vel_msg.data
        self.MAX_VEL = self.base_max_vel

    def callback_stop_line(self, msg):
        current_time = time.time()
        if current_time < self.stop_line_suppression_until:
            return

        if msg.data and self.state == 'NORMAL':
            self.get_logger().info('Stop line detected → Switching to STOP state')
            self.state = 'STOP'
            self.stop_line_suppression_until = current_time + self.suppression_duration

    def callback_traffic_light(self, msg):
        colors = msg.data.split(',') if msg.data != 'NONE' else []
        if self.state == 'WAIT_LIGHT' and 'GREEN' in colors:
            self.get_logger().info('Green light detected → Resuming driving')
            self.state = 'NORMAL'

    def callback_sign(self, msg):
        current_time = time.time()
        if current_time < self.sign_suppression_until:
            return

        sign = msg.data

        if sign == 'None':
            return

        if sign == 'speed_30':
            self.MAX_VEL = 0.05
            self.get_logger().info('Speed limit 30 detected → MAX_VEL = 0.05')
        elif sign == 'speed_40':
            self.MAX_VEL = 0.07
            self.get_logger().info('Speed limit 40 detected → MAX_VEL = 0.07')
        elif sign == 'speed_50':
            self.MAX_VEL = 0.1
            self.get_logger().info('Speed limit 50 detected → MAX_VEL = 0.1')

        elif sign == 'stop' and self.state == 'NORMAL':
            self.get_logger().info('Stop sign detected → STOP_SIGN state')
            self.state = 'STOP_SIGN'

        elif sign == 'left_turn' and self.state == 'NORMAL':
            self.get_logger().info('Left turn detected → entering LEFT_TURN state')
            self.state = 'LEFT_TURN'
            self.turn_direction = 'LEFT'
            self.turn_start_time = current_time

        elif sign == 'right_turn' and self.state == 'NORMAL':
            self.get_logger().info('Right turn detected → entering RIGHT_TURN state')
            self.state = 'RIGHT_TURN'
            self.turn_direction = 'RIGHT'
            self.turn_start_time = current_time

        self.sign_suppression_until = current_time + self.sign_suppression_duration

    def callback_follow_lane(self, desired_center):
        if self.avoid_active:
            return

        if self.state == 'STOP':
            self.publish_stop()
            self.state = 'WAIT_LIGHT'
            self.get_logger().info('Vehicle stopped at stop line. Waiting for traffic light.')
            return

        if self.state == 'WAIT_LIGHT':
            self.publish_stop()
            return

        if self.state == 'STOP_SIGN':
            self.publish_stop()
            time.sleep(1)
            self.get_logger().info('Stop sign stop complete → entering WAIT_LEVEL_CROSSING state')
            self.state = 'WAIT_LEVEL_CROSSING'
            return

        if self.state == 'WAIT_LEVEL_CROSSING':
            self.publish_stop()
            return

        # 기존 soft turn 및 PID 주행 로직 이하 동일
        current_time = time.time()

        if self.state in ['LEFT_TURN', 'RIGHT_TURN']:
            progress = (current_time - self.turn_start_time) / self.turn_duration
            progress = min(progress, 1.0)
        else:
            progress = 0.0

        if self.state == 'LEFT_TURN':
            desired_bias = -60 * progress
        elif self.state == 'RIGHT_TURN':
            desired_bias = 60 * progress
        else:
            desired_bias = 0

        center = desired_center.data + desired_bias
        error = center - 500

        Kp = 0.0025
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        velocity_limit = 0.03 if self.state in ['LEFT_TURN', 'RIGHT_TURN'] else 0.05

        twist = Twist()
        twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), velocity_limit)
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish(twist)


    def callback_level_crossing(self, msg):
        if self.state == 'WAIT_LEVEL_CROSSING':
            # 첫 wait 상태 → 바로 ready 상태 진입
            self.get_logger().info("Level crossing: waiting for first stop signal")
            self.state = 'WAIT_LC_READY'

        elif self.state == 'WAIT_LC_READY':
            if msg.data == 'stop':
                self.get_logger().info("Level crossing: stop detected → waiting for go")
                self.state = 'WAIT_LC_GO'

        elif self.state == 'WAIT_LC_GO':
            if msg.data == 'go':
                self.get_logger().info("Level crossing opened → Resuming driving")
                self.state = 'NORMAL'


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

    def publish_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

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
