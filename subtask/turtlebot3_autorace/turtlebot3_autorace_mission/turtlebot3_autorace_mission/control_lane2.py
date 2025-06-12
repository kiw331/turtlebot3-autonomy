#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float64, String
import threading
import time
import sys
import tty
import termios
import select

class ControlLane(Node):
    def __init__(self):
        super().__init__('control_lane')

        # --- UI 및 FSM 상태 변수 초기화 ---
        self.is_running = True
        self.ready_to_drive = False
        self.state = 'IDLE'

        # --- drive_mode YAML 파라미터 선언 ---
        self.declare_parameter('drive_mode', 'comfort')

        for mode in ['eco', 'comfort', 'sport']:
            self.declare_parameter(f'drive_modes.{mode}.max_vel', 0.05 if mode=='eco' else 0.1 if mode=='comfort' else 0.2)
            self.declare_parameter(f'drive_modes.{mode}.kp', 0.0025 if mode in ['eco', 'comfort'] else 0.0035)
            self.declare_parameter(f'drive_modes.{mode}.kd', 0.005 if mode=='eco' else 0.007)

        # UI 텍스트 파라미터 선언
        self.declare_parameter('ui_texts.welcome_prompt', ">> 프로젝트 시작. 사용자 이름을 입력하세요: ")

        # 초기 drive_mode 설정 및 관련 파라미터 로드
        self.current_drive_mode = self.get_parameter('drive_mode').get_parameter_value().string_value
        self.load_drive_mode_params()

        # UI 텍스트 읽기
        self.welcome_prompt = self.get_parameter('ui_texts.welcome_prompt').get_parameter_value().string_value

        # --- ROS2 구독자 및 발행자 ---
        self.sub_lane = self.create_subscription(Float64, '/control/lane', self.callback_follow_lane, 10)
        self.sub_traffic_light = self.create_subscription(String, '/detect/traffic_light_color', self.callback_traffic_light, 10)
        self.sub_stop_line = self.create_subscription(Bool, '/detect/stop_line', self.callback_stop_line, 10)
        self.sub_sign = self.create_subscription(String, '/detect/sign', self.callback_sign, 10)
        self.sub_level_crossing = self.create_subscription(String, '/detect/level_crossing_state', self.callback_level_crossing, 10)
        self.sub_avoid_active = self.create_subscription(Bool, '/avoid_active', self.callback_avoid_active, 1)

        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_cmd_vel = self.create_publisher(Twist, '/control/cmd_vel', 10)
        self.pub_user_name = self.create_publisher(String, '/control/user_name', qos_profile)
        self.pub_mode_status = self.create_publisher(String, '/control/current_drive_mode', 10)

        # --- 내부 상태 및 변수 ---
        self.last_error = 0
        self.MAX_VEL = self.drive_mode_max_vel  # FSM에서 상황별 최고 속도 조정용 초기값
        self.kp = self.drive_mode_kp
        self.kd = self.drive_mode_kd

        self.avoid_active = False
        self.turn_direction = None
        self.stop_line_suppression_until = 0
        self.suppression_duration = 3.0
        self.sign_suppression_until = 0
        self.sign_suppression_duration = 5.0
        self.turn_start_time = 0
        self.turn_duration = 2.0

        # --- 터미널 입력 스레드 시작 ---
        self.input_thread = threading.Thread(target=self.terminal_input_thread)
        self.input_thread.daemon = True
        self.input_thread.start()
        self.get_logger().info("ControlLane Node has been initialized.")

    def load_drive_mode_params(self):
        prefix = f'drive_modes.{self.current_drive_mode}'
        self.drive_mode_max_vel = self.get_parameter(f'{prefix}.max_vel').value
        self.drive_mode_kp = self.get_parameter(f'{prefix}.kp').value
        self.drive_mode_kd = self.get_parameter(f'{prefix}.kd').value

        self.get_logger().info(
            f"Drive mode '{self.current_drive_mode}' params loaded: max_vel={self.drive_mode_max_vel}, kp={self.drive_mode_kp}, kd={self.drive_mode_kd}"
        )
        # FSM 기본 속도 및 PID 파라미터도 갱신
        self.MAX_VEL = self.drive_mode_max_vel
        self.kp = self.drive_mode_kp
        self.kd = self.drive_mode_kd

    def get_key(self, settings):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist: key = sys.stdin.read(1)
        else: key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def publish_drive_mode(self):
        mode_msg = String()
        mode_msg.data = self.current_drive_mode
        self.pub_mode_status.publish(mode_msg)

    def terminal_input_thread(self):
        time.sleep(1.0)
        settings = termios.tcgetattr(sys.stdin)

        # UI 텍스트 출력
        print(self.welcome_prompt, end="", flush=True)

        user_name = ""
        while rclpy.ok():
            key = self.get_key(settings)
            if key == '\r' or key == '\n':
                break
            elif key == '\x7f' and len(user_name) > 0:
                user_name = user_name[:-1]
                print("\b \b", end="", flush=True)
            elif key.isprintable():
                user_name += key
                print(key, end="", flush=True)

        name_msg = String()
        name_msg.data = user_name
        self.pub_user_name.publish(name_msg)
        print(f"\n>> {user_name}님, 환영합니다. 주행 준비를 시작합니다.")
        self.ready_to_drive = True
        self.state = 'NORMAL'
        self.get_logger().info("Initial setup complete. Autonomous driving is now authorized.")

        self.publish_drive_mode()

        print("\n=============================================")
        print(">> Drive Mode (1:Eco, 2:Comfort, 3:Sport) | 'q' to quit")
        print("=============================================")

        while self.is_running:
            key = self.get_key(settings)
            mode_changed = False
            if key == '1':
                self.current_drive_mode = 'eco'
                mode_changed = True
            elif key == '2':
                self.current_drive_mode = 'comfort'
                mode_changed = True
            elif key == '3':
                self.current_drive_mode = 'sport'
                mode_changed = True

            if mode_changed:
                self.load_drive_mode_params()
                self.get_logger().info(f"Drive Mode -> '{self.current_drive_mode.capitalize()}'")
                print(f"\r>> Mode: {self.current_drive_mode.capitalize():<10}", end="", flush=True)
                self.publish_drive_mode()

            elif key == 'q':
                self.is_running = False
                print("\n'q' pressed. Shutting down...")
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
                if rclpy.ok():
                    rclpy.shutdown()
                break

    def callback_follow_lane(self, desired_center):
        if not self.ready_to_drive or self.avoid_active:
            self.publish_stop()
            return

        if self.state == 'STOP':
            self.publish_stop()
            self.state = 'WAIT_LIGHT'
            self.get_logger().info('Vehicle stopped. Waiting for traffic light.')
            return
        if self.state == 'WAIT_LIGHT':
            self.publish_stop()
            return
        if self.state == 'STOP_SIGN':
            self.publish_stop()
            time.sleep(1)
            self.state = 'WAIT_LEVEL_CROSSING'
            self.get_logger().info('Stop complete. Waiting for level crossing.')
            return
        if self.state == 'WAIT_LEVEL_CROSSING':
            self.publish_stop()
            return

        current_time = time.time()
        desired_bias = 0
        if self.state in ['LEFT_TURN', 'RIGHT_TURN']:
            progress = min((current_time - self.turn_start_time) / self.turn_duration, 1.0)
            desired_bias = -60 * progress if self.state == 'LEFT_TURN' else 60 * progress
            if progress >= 1.0:
                self.state = 'NORMAL'

        center = desired_center.data + desired_bias
        error = center - 500
        angular_z = self.kp * error + self.kd * (error - self.last_error)
        self.last_error = error

        fsm_max_vel = self.MAX_VEL
        situational_limit = 0.03 if self.state in ['LEFT_TURN', 'RIGHT_TURN'] else self.drive_mode_max_vel
        dynamic_speed = fsm_max_vel * (max(1 - abs(error) / 500, 0) ** 2.2)
        final_linear_x = min(dynamic_speed, situational_limit)

        twist = Twist()
        twist.linear.x = final_linear_x
        twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
        self.pub_cmd_vel.publish(twist)

    def callback_stop_line(self, msg):
        current_time = time.time()
        if msg.data and self.state == 'NORMAL' and current_time > self.stop_line_suppression_until:
            self.state = 'STOP'
            self.get_logger().info('Stop line -> STOP state')
            self.stop_line_suppression_until = current_time + self.suppression_duration

    def callback_traffic_light(self, msg):
        if self.state == 'WAIT_LIGHT' and 'GREEN' in msg.data:
            self.state = 'NORMAL'
            self.get_logger().info('Green light -> NORMAL state')

    def callback_sign(self, msg):
        current_time = time.time()
        sign = msg.data
        if sign == 'None' or current_time < self.sign_suppression_until:
            return
        self.get_logger().info(f"Sign Detected: {sign}")
        self.sign_suppression_until = current_time + self.sign_suppression_duration
        if sign == 'speed_30':
            self.MAX_VEL = 0.05
        elif sign == 'speed_40':
            self.MAX_VEL = 0.07
        elif sign == 'speed_50':
            self.MAX_VEL = 0.1
        elif self.state == 'NORMAL':
            if sign == 'stop':
                self.state = 'STOP_SIGN'
            elif sign == 'left_turn':
                self.state = 'LEFT_TURN'
                self.turn_direction = 'LEFT'
                self.turn_start_time = current_time
            elif sign == 'right_turn':
                self.state = 'RIGHT_TURN'
                self.turn_direction = 'RIGHT'
                self.turn_start_time = current_time

    def callback_level_crossing(self, msg):
        if self.state == 'WAIT_LEVEL_CROSSING':
            self.state = 'WAIT_LC_READY'
        elif self.state == 'WAIT_LC_READY' and msg.data == 'stop':
            self.state = 'WAIT_LC_GO'
        elif self.state == 'WAIT_LC_GO' and msg.data == 'go':
            self.state = 'NORMAL'

    def callback_avoid_active(self, msg):
        self.avoid_active = msg.data

    def callback_avoid_cmd(self, msg):
        if self.avoid_active:
            self.pub_cmd_vel.publish(msg)

    def publish_stop(self):
        self.pub_cmd_vel.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = ControlLane()
    try:
        while rclpy.ok() and node.is_running:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish_stop()
            if node.is_running:
                node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
