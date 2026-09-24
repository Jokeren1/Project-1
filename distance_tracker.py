import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class DistanceTracker(Node):

    def __init__(self):
        super().__init__('distance_tracker')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.previous_x = None
        self.previous_y = None
        self.distance_traveled = 0.0

        self.one_foot_meters = 0.3048

        self.get_logger().info('Distance tracker started')

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y

        # First odometry reading: save position and wait for next update.
        if self.previous_x is None or self.previous_y is None:
            self.previous_x = current_x
            self.previous_y = current_y
            return

        dx = current_x - self.previous_x
        dy = current_y - self.previous_y

        distance_step = math.sqrt(dx ** 2 + dy ** 2)

        self.distance_traveled += distance_step

        self.previous_x = current_x
        self.previous_y = current_y

        if self.distance_traveled >= self.one_foot_meters:
            self.get_logger().info('Robot traveled approximately 1 foot')
            self.distance_traveled = 0.0


def main(args=None):
    rclpy.init(args=args)

    node = DistanceTracker()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()