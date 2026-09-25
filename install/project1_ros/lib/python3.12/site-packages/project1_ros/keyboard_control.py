import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class KeyboardControl(Node):

    def __init__(self):
        super().__init__('keyboard_control')

        self.publisher = self.create_publisher(
            Twist,
            '/keyboard_cmd_vel',
            10
        )

        self.get_logger().info(
            'Keyboard control started. Use W/A/S/D to move, X to stop, Q to quit.'
        )

    def publish_command(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.publisher.publish(msg)

    def run(self):
        settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setraw(sys.stdin.fileno())

            while rclpy.ok():
                key = sys.stdin.read(1).lower()

                if key == 'w':
                    self.publish_command(linear_x=0.2)

                elif key == 's':
                    self.publish_command(linear_x=-0.2)

                elif key == 'a':
                    self.publish_command(angular_z=0.6)

                elif key == 'd':
                    self.publish_command(angular_z=-0.6)

                elif key == 'x':
                    self.publish_command()

                elif key == 'q':
                    self.publish_command()
                    break

        finally:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                settings
            )


def main(args=None):
    rclpy.init(args=args)

    node = KeyboardControl()

    node.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
