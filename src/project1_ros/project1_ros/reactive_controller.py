import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ReactiveController(Node):
    """
    Reactive behavior controller for Project 1.

    Autonomous behavior priority:
    1. Escape
    2. Avoid
    3. Random turn
    4. Forward
    """

    def __init__(self):
        super().__init__('reactive_controller')

        # ---------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------

        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.distance_subscription = self.create_subscription(
            Bool,
            '/one_foot_traveled',
            self.one_foot_callback,
            10
        )

        self.cmd_publisher = self.create_publisher(
            Twist,
            '/reactive_cmd_vel',
            10
        )

        # Main behavior loop runs at 10 Hz.
        self.control_timer = self.create_timer(
            0.1,
            self.control_loop
        )

        # ---------------------------------------------------------
        # Project constants
        # ---------------------------------------------------------

        # 1 foot in meters.
        self.obstacle_distance = 0.3048

        # Robot motion speeds.
        self.forward_speed = 0.15
        self.turn_speed = 0.5

        # Left and right distances within 8 cm are treated
        # as roughly symmetric.
        self.symmetry_tolerance = 0.08

        # ---------------------------------------------------------
        # LiDAR state
        # ---------------------------------------------------------

        self.scan_received = False
        self.scan_number = 0

        self.left_distance = float('inf')
        self.center_distance = float('inf')
        self.right_distance = float('inf')

        # Used to prevent the same stale LiDAR scan from
        # repeatedly triggering escape.
        self.waiting_for_new_scan_after_escape = False
        self.escape_completion_scan_number = -1

        # ---------------------------------------------------------
        # Escape behavior state
        # ---------------------------------------------------------

        self.escape_active = False
        self.escape_end_time = None

        # Escape target: approximately 180 degrees.
        self.escape_angle = math.pi

        # Approximate turn duration:
        # time = angle / angular velocity
        self.escape_duration = (
            self.escape_angle / self.turn_speed
        )

        # ---------------------------------------------------------
        # Random-turn behavior state
        # ---------------------------------------------------------

        self.random_turn_requested = False
        self.random_turn_active = False

        self.random_turn_end_time = None
        self.random_turn_direction = 1.0
        self.random_turn_angle = 0.0

        self.get_logger().info(
            'Reactive controller initialized'
        )

    def scan_callback(self, msg):
        """
        Process the front 90 degrees of the LiDAR.

        Front-right:
            -45 to -15 degrees

        Front-center:
            -15 to +15 degrees

        Front-left:
            +15 to +45 degrees
        """

        self.scan_received = True
        self.scan_number += 1

        left_ranges = []
        center_ranges = []
        right_ranges = []

        for i, distance in enumerate(msg.ranges):

            # Ignore NaN and infinite measurements.
            if math.isnan(distance) or math.isinf(distance):
                continue

            # Ignore measurements outside the sensor's valid range.
            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = (
                msg.angle_min +
                (i * msg.angle_increment)
            )

            angle_degrees = math.degrees(angle)

            # Front-right sector.
            if -45.0 <= angle_degrees < -15.0:
                right_ranges.append(distance)

            # Front-center sector.
            elif -15.0 <= angle_degrees <= 15.0:
                center_ranges.append(distance)

            # Front-left sector.
            elif 15.0 < angle_degrees <= 45.0:
                left_ranges.append(distance)

        self.left_distance = (
            min(left_ranges)
            if left_ranges
            else float('inf')
        )

        self.center_distance = (
            min(center_ranges)
            if center_ranges
            else float('inf')
        )

        self.right_distance = (
            min(right_ranges)
            if right_ranges
            else float('inf')
        )

        # If escape previously completed, require a new scan
        # before allowing another escape to trigger.
        if (
            self.waiting_for_new_scan_after_escape
            and self.scan_number > self.escape_completion_scan_number
        ):
            self.waiting_for_new_scan_after_escape = False

    def one_foot_callback(self, msg):
        """
        Receive notification that the robot has traveled
        approximately one foot.
        """

        if msg.data:
            self.random_turn_requested = True

            self.get_logger().info(
                'Random turn requested after 1 foot'
            )

    def control_loop(self):
        """
        Select and execute the highest-priority
        autonomous behavior.
        """

        # Wait until we have real LiDAR information.
        if not self.scan_received:
            return

        # ---------------------------------------------------------
        # Priority 1: Escape
        # ---------------------------------------------------------

        # Escape is a fixed-action pattern. Once it begins,
        # it continues even if the triggering obstacle changes.
        if self.escape_active:
            self.continue_escape()
            return

        obstacle_present = self.obstacle_in_front()

        if (
            obstacle_present
            and not self.waiting_for_new_scan_after_escape
            and self.is_symmetric_obstacle()
        ):
            self.start_escape()
            return

        # ---------------------------------------------------------
        # Priority 2: Avoid
        # ---------------------------------------------------------

        if obstacle_present:
            self.avoid_obstacle()
            return

        # ---------------------------------------------------------
        # Priority 3: Random turn
        # ---------------------------------------------------------

        if self.random_turn_active:
            self.continue_random_turn()
            return

        if self.random_turn_requested:
            self.start_random_turn()
            return

        # ---------------------------------------------------------
        # Priority 4: Forward
        # ---------------------------------------------------------

        self.drive_forward()

    def obstacle_in_front(self):
        """
        Return True if any front LiDAR sector contains
        an obstacle within one foot.
        """

        return (
            self.left_distance <= self.obstacle_distance
            or
            self.center_distance <= self.obstacle_distance
            or
            self.right_distance <= self.obstacle_distance
        )

    def is_symmetric_obstacle(self):
        """
        Determine whether the obstacle arrangement in front
        of the robot is roughly symmetric.
        """

        left_close = (
            self.left_distance <= self.obstacle_distance
        )

        center_close = (
            self.center_distance <= self.obstacle_distance
        )

        right_close = (
            self.right_distance <= self.obstacle_distance
        )

        # There must be nearby obstacles on both sides
        # for the situation to be considered symmetric.
        if not left_close or not right_close:
            return False

        difference = abs(
            self.left_distance -
            self.right_distance
        )

        # Similar left/right distances indicate symmetry.
        if difference <= self.symmetry_tolerance:
            return True

        # If the center is blocked too, allow slightly
        # more difference between the two sides.
        if (
            center_close
            and difference <= self.symmetry_tolerance * 2.0
        ):
            return True

        return False

    def drive_forward(self):
        """
        Lowest-priority autonomous behavior.
        """

        command = Twist()

        command.linear.x = self.forward_speed
        command.angular.z = 0.0

        self.cmd_publisher.publish(command)

    def avoid_obstacle(self):
        """
        Reflexively turn away from an asymmetric obstacle.
        """

        command = Twist()

        command.linear.x = 0.0

        if self.left_distance < self.right_distance:
            # Obstacle is closer on the left.
            # Turn right.
            command.angular.z = -self.turn_speed

        elif self.right_distance < self.left_distance:
            # Obstacle is closer on the right.
            # Turn left.
            command.angular.z = self.turn_speed

        else:
            # Fallback if distances happen to match but
            # the pattern was not classified as symmetric.
            command.angular.z = self.turn_speed

        self.cmd_publisher.publish(command)

    def start_escape(self):
        """
        Begin a fixed-action escape turn of approximately
        180 degrees.
        """

        self.escape_active = True

        current_time = self.current_time_seconds()

        self.escape_end_time = (
            current_time +
            self.escape_duration
        )

        self.get_logger().info(
            'Behavior: ESCAPE started'
        )

    def continue_escape(self):
        """
        Continue the fixed-action escape until its target
        duration has completed.
        """

        current_time = self.current_time_seconds()

        if current_time >= self.escape_end_time:

            self.escape_active = False
            self.escape_end_time = None

            # Record which scan was current when escape ended.
            # Another escape cannot begin until a newer scan arrives.
            self.escape_completion_scan_number = self.scan_number
            self.waiting_for_new_scan_after_escape = True

            self.stop_robot()

            self.get_logger().info(
                'Behavior: ESCAPE completed'
            )

            return

        command = Twist()

        command.linear.x = 0.0
        command.angular.z = self.turn_speed

        self.cmd_publisher.publish(command)

    def start_random_turn(self):
        """
        Start a uniformly random turn in the range
        -15 degrees through +15 degrees.
        """

        self.random_turn_requested = False
        self.random_turn_active = True

        angle_degrees = random.uniform(
            -15.0,
            15.0
        )

        self.random_turn_angle = math.radians(
            angle_degrees
        )

        if self.random_turn_angle >= 0.0:
            self.random_turn_direction = 1.0
        else:
            self.random_turn_direction = -1.0

        turn_duration = (
            abs(self.random_turn_angle) /
            self.turn_speed
        )

        current_time = self.current_time_seconds()

        self.random_turn_end_time = (
            current_time +
            turn_duration
        )

        self.get_logger().info(
            f'Behavior: RANDOM TURN started '
            f'({angle_degrees:.1f} degrees)'
        )

    def continue_random_turn(self):
        """
        Continue the random turn until the selected
        angle has approximately been completed.
        """

        current_time = self.current_time_seconds()

        if current_time >= self.random_turn_end_time:

            self.random_turn_active = False
            self.random_turn_end_time = None

            self.stop_robot()

            self.get_logger().info(
                'Behavior: RANDOM TURN completed'
            )

            return

        command = Twist()

        command.linear.x = 0.0

        command.angular.z = (
            self.random_turn_direction *
            self.turn_speed
        )

        self.cmd_publisher.publish(command)

    def current_time_seconds(self):
        """
        Return the ROS clock time in seconds.
        """

        return (
            self.get_clock().now().nanoseconds /
            1_000_000_000.0
        )

    def stop_robot(self):
        """
        Publish a zero-velocity command.
        """

        command = Twist()

        command.linear.x = 0.0
        command.angular.z = 0.0

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
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
