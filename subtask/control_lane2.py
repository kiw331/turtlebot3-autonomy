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
            String, '/detect/level_crossing_state', self.callback_level_crossing, 1)


        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 1)

        self.last_error = 0
        self.base_max_vel = 0.15
        self.MAX_VEL = self.base_max_vel
        self.avoid_active = False
        self.avoid_twist = Twist()

        self.state = 'NORMAL'
        self.turn_direction = None

        self.stop_line_suppression_until = 0
        self.suppression_duration = 3.0

        self.sign_suppression_until = 0
        self.sign_suppression_duration = 5.0

        self.turn_start_time = 0
        self.turn_duration = 2.0

        self.speed_limit_vel = 0.3

        self.crossing_cleared = False
        self.level_crossing_stop_detected = False

    def callback_get_max_vel(self, msg):
        self.base_max_vel = msg.data
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

        if sign == 'speed_limit':
            self.get_logger().info('Speed limit sign detected → keeping current MAX_VEL')
            self.MAX_VEL = self.speed_limit_vel

        elif sign == 'left' and self.state == 'NORMAL':
            self.get_logger().info('Left sign detected → entering LEFT_TURN state')
            self.state = 'LEFT_TURN'
            self.turn_direction = 'LEFT'
            self.turn_start_time = current_time

        elif sign == 'right' and self.state == 'NORMAL':
            self.get_logger().info('Right sign detected → entering RIGHT_TURN state')
            self.state = 'RIGHT_TURN'
            self.turn_direction = 'RIGHT'
            self.turn_start_time = current_time

        elif sign == 'stop' and self.state == 'NORMAL':
            self.get_logger().info('Stop sign detected → entering STOPPING state')
            self.state = 'STOPPING'
            self.stop_sign_start_time = current_time

    def callback_level_crossing(self, msg):

        if self.state != 'WAIT_LEVEL_CROSSING':
            return

        signal = msg.data
        # self.get_logger().info(f'Received level crossing signal: {signal}')

        if signal == 'stop':
            self.level_crossing_stop_detected = True

        if self.crossing_cleared:
            return

        if signal == 'go' and self.level_crossing_stop_detected:
            self.crossing_cleared = True
            self.get_logger().info('Level crossing open → clearance granted')



    def callback_follow_lane(self, desired_center):
        if self.avoid_active:
            return

        current_time = time.time()

        if self.state == 'STOP':
            self.publish_stop()
            self.state = 'WAIT_LIGHT'
            self.get_logger().info('Vehicle stopped at stop line. Waiting for traffic light.')
            return

        if self.state == 'WAIT_LIGHT':
            self.publish_stop()
            return

        if self.state == 'STOPPING':
            if current_time - self.stop_sign_start_time >= 8:
                self.get_logger().info('Stop sign wait complete → stopping and entering WAIT_LEVEL_CROSSING')
                self.publish_stop()
                self.state = 'WAIT_LEVEL_CROSSING'
                return
            # 3초 동안 주행 유지 → 아래 PID 계속 수행

        if self.state == 'WAIT_LEVEL_CROSSING':
            self.get_logger().info(f'check level_crossing_stop_detected: {self.level_crossing_stop_detected}')
            self.get_logger().info(f'check crossing_cleared: {self.crossing_cleared}')

            if self.crossing_cleared:
                self.get_logger().info('Level crossing cleared → returning to NORMAL state')
                self.state = 'NORMAL'
                self.level_crossing_stop_detected = False
                self.crossing_cleared = False
            else:
                self.publish_stop()  # 아직 clearance 안 됐을 때만 정지 유지
            return

        if self.state in ['LEFT_TURN', 'RIGHT_TURN']:
            if not hasattr(self, 'turn_start_time') or self.turn_start_time is None:
                self.turn_start_time = current_time
                self.turn_duration = 3.0

            progress = (current_time - self.turn_start_time) / self.turn_duration
            progress = min(progress, 1.0)
            if progress >= 1.0:
                self.get_logger().info('Turn completed → returning to NORMAL state')
                self.state = 'NORMAL'
                progress = 0.0
                self.turn_start_time = None
                self.sign_suppression_until = current_time + self.sign_suppression_duration
        else:
            progress = 0.0

        if self.state == 'LEFT_TURN':
            desired_bias = -100 * progress
        elif self.state == 'RIGHT_TURN':
            desired_bias = 100 * progress
        else:
            desired_bias = 0

        center = desired_center.data + desired_bias
        error = center - 500

        Kp = 0.0025
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.last_error)
        self.last_error = error

        velocity_limit = 0.04 if self.state in ['LEFT_TURN', 'RIGHT_TURN'] else 0.05

        twist = Twist()
        twist.linear.x = min(self.MAX_VEL * (max(1 - abs(error) / 500, 0) ** 2.2), velocity_limit)
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
