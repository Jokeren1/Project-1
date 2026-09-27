import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ReactiveController(Node):
    """
    Reactive behavior controller for Project 1.

    Responsibilities:
    - LiDAR processing
    - Forward behavior
    - Obstacle avoidance
    - Escape behavior
    - Random turning after approximately 1 ft of forward motion
    """

    def __init__(self):
        super().__init__('reactive_controller')

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/reactive_cmd_vel',
            10
        )

        # 1 foot in meters
        self.obstacle_distance = 0.3048

        # Movement speeds
        self.forward_speed = 0.15
        self.turn_speed = 0.5

        # Most recent obstacle measurements
        self.left_distance = float('inf')
        self.center_distance = float('inf')
        self.right_distance = float('inf')

        self.get_logger().info('Reactive controller initialized')

    def scan_callback(self, msg):
        """
        Process the front portion of the LiDAR scan.

        The front is divided into:
        - left sector
        - center sector
        - right sector
        """

        left_ranges = []
        center_ranges = []
        right_ranges = []

        for i, distance in enumerate(msg.ranges):

            # Ignore invalid LiDAR values.
            if math.isnan(distance) or math.isinf(distance):
                continue

            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = msg.angle_min + (i * msg.angle_increment)
            angle_degrees = math.degrees(angle)

            # Front-right: -45 to -15 degrees
            if -45.0 <= angle_degrees < -15.0:
                right_ranges.append(distance)

            # Front-center: -15 to +15 degrees
            elif -15.0 <= angle_degrees <= 15.0:
                center_ranges.append(distance)

            # Front-left: +15 to +45 degrees
            elif 15.0 < angle_degrees <= 45.0:
                left_ranges.append(distance)

        self.left_distance = (
            min(left_ranges) if left_ranges else float('inf')
        )

        self.center_distance = (
            min(center_ranges) if center_ranges else float('inf')
        )

        self.right_distance = (
            min(right_ranges) if right_ranges else float('inf')
        )

        self.print_lidar_status()

    def print_lidar_status(self):
        """Print the closest obstacle detected in each front sector."""

        self.get_logger().info(
            'LiDAR | '
            f'Left: {self.format_distance(self.left_distance)} m | '
            f'Center: {self.format_distance(self.center_distance)} m | '
            f'Right: {self.format_distance(self.right_distance)} m'
        )

        closest = min(
            self.left_distance,
            self.center_distance,
            self.right_distance
        )

        if closest <= self.obstacle_distance:
            self.get_logger().info(
                'Obstacle detected within 1 foot'
            )

    def format_distance(self, distance):
        """Make infinite LiDAR readings easier to read."""

        if math.isinf(distance):
            return 'none'

        return f'{distance:.2f}'

    def drive_forward(self):
        """Lowest-priority autonomous behavior."""

        command = Twist()
        command.linear.x = self.forward_speed

        self.cmd_publisher.publish(command)

    def avoid_obstacle(self):
        """Turn away from an asymmetric obstacle."""

        command = Twist()

        if self.left_distance < self.right_distance:
            # Obstacle is closer on the left, so turn right.
            command.angular.z = -self.turn_speed

        elif self.right_distance < self.left_distance:
            # Obstacle is closer on the right, so turn left.
            command.angular.z = self.turn_speed

        self.cmd_publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)

    controller = ReactiveController()

    try:
        rclpy.spin(controller)

    except KeyboardInterrupt:
        pass

    finally:
        controller.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
