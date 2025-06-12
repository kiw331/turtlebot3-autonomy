import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class StopRobot(Node):
    def __init__(self):
        super().__init__('stop_robot')
        self.cmd_vel_pub = self.create_publisher(Twist, '/control/cmd_vel', 10)

        # 0.1초마다 속도 0 발행 (혹시 모를 잔여속도 방지)
        self.timer = self.create_timer(0.1, self.publish_stop_command)
        self.get_logger().info('StopRobot Node started: publishing zero velocity.')

    def publish_stop_command(self):
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StopRobot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
