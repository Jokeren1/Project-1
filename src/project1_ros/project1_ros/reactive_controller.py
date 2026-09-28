import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ReactiveController(Node):
    """
    Reactive behavior controller for Project 1.

    Current responsibilities:
    - LiDAR processing
    - Forward behavior
    - Asymmetric obstacle avoidance
    - Symmetric obstacle detection / escape trigger
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

        # Project requirement: obstacle behaviors trigger within 1 foot.
        self.obstacle_distance = 0.3048

        # Movement speeds.
        self.forward_speed = 0.15
        self.turn_speed = 0.5

        # Difference allowed between left and right obstacle distances
        # before treating the obstacle layout as roughly symmetric.
        self.symmetry_tolerance = 0.08

        self.left_distance = float('inf')
        self.center_distance = float('inf')
        self.right_distance = float('inf')

        self.get_logger().info('Reactive controller initialized')

    def scan_callback(self, msg):
        """
        Process the front 90 degrees of the LiDAR scan.

        Front-right: -45 to -15 degrees
        Front-center: -15 to +15 degrees
        Front-left: +15 to +45 degrees
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

            if -45.0 <= angle_degrees < -15.0:
                right_ranges.append(distance)

            elif -15.0 <= angle_degrees <= 15.0:
                center_ranges.append(distance)

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

        self.choose_behavior()

    def choose_behavior(self):
        """
        Select the highest-priority autonomous behavior.

        Current priority:
        1. Escape
        2. Avoid
        3. Forward

        Random turning will be inserted between avoid and forward later.
        """

        obstacle_left = self.left_distance <= self.obstacle_distance
        obstacle_center = self.center_distance <= self.obstacle_distance
        obstacle_right = self.right_distance <= self.obstacle_distance

        obstacle_present = (
            obstacle_left or
            obstacle_center or
            obstacle_right
        )

        if not obstacle_present:
            self.drive_forward()
            return

        if self.is_symmetric_obstacle():
            self.get_logger().info(
                'Behavior: ESCAPE - roughly symmetric obstacle detected'
            )
            self.escape_obstacle()
            return

        self.get_logger().info(
            'Behavior: AVOID - asymmetric obstacle detected'
        )
        self.avoid_obstacle()

    def is_symmetric_obstacle(self):
        """
        Determine whether obstacles in front are roughly symmetric.

        Escape is used when obstacles appear on both sides at similar
        distances, especially when the center is also blocked.
        """

        left_close = self.left_distance <= self.obstacle_distance
        right_close = self.right_distance <= self.obstacle_distance
        center_close = self.center_distance <= self.obstacle_distance

        if not left_close or not right_close:
            return False

        difference = abs(
            self.left_distance - self.right_distance
        )

        if difference <= self.symmetry_tolerance:
            return True

        if center_close and difference <= (
            self.symmetry_tolerance * 2.0
        ):
            return True

        return False

    def drive_forward(self):
        """Lowest-priority autonomous behavior."""

        command = Twist()
        command.linear.x = self.forward_speed
        command.angular.z = 0.0

        self.cmd_publisher.publish(command)

    def avoid_obstacle(self):
        """
        Reflexively turn away from the closer obstacle.

        This behavior should only continue while the asymmetric
        obstacle is still detected.
        """

        command = Twist()
        command.linear.x = 0.0

        if self.left_distance < self.right_distance:
            # Obstacle is closer on the left -> turn right.
            command.angular.z = -self.turn_speed
            self.get_logger().info('Avoid direction: RIGHT')

        elif self.right_distance < self.left_distance:
            # Obstacle is closer on the right -> turn left.
            command.angular.z = self.turn_speed
            self.get_logger().info('Avoid direction: LEFT')

        else:
            # Equal readings but not classified as symmetric.
            command.angular.z = self.turn_speed

        self.cmd_publisher.publish(command)

    def escape_obstacle(self):
        """
        Initial escape behavior.

        The project requires a fixed-action turn of about 180 degrees.
        For now, this starts the turn. The next step will make it
        continue for the full target angle even after the obstacle
        disappears.
        """

        command = Twist()

        command.linear.x = 0.0
        command.angular.z = self.turn_speed

        self.cmd_publisher.publish(command)

    def format_distance(self, distance):
        """Format a LiDAR distance for logging."""

        if math.isinf(distance):
            return 'none'

        return f'{distance:.2f}'

    def log_scan(self):
        """Optional helper for debugging LiDAR measurements."""

        self.get_logger().info(
            'LiDAR | '
            f'Left: {self.format_distance(self.left_distance)} m | '
            f'Center: {self.format_distance(self.center_distance)} m | '
            f'Right: {self.format_distance(self.right_distance)} m'
        )


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
