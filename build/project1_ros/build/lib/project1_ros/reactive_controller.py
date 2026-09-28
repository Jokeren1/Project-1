import math
import random

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class ReactiveController(Node):
    """
    Reactive behavior controller for Project 1.

    Behavior priority:
    1. Halt on bumper collision
    2. Keyboard control
    3. Escape from symmetric obstacles
    4. Avoid asymmetric obstacles
    5. Random turn after 1 foot
    6. Drive forward
    """

    def __init__(self):
        super().__init__('reactive_controller')

        # ---------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------

        # LiDAR data used for escape and avoidance behaviors.
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Event sent by distance_tracker after approximately
        # one foot of movement.
        self.distance_subscription = self.create_subscription(
            Bool,
            '/one_foot_traveled',
            self.one_foot_callback,
            10
        )

        # Manual movement commands from keyboard_control.
        self.keyboard_subscription = self.create_subscription(
            Twist,
            '/keyboard_cmd_vel',
            self.keyboard_callback,
            10
        )

        # Gazebo contact information for the TurtleBot bumper.
        self.bumper_subscription = self.create_subscription(
            Contacts,
            '/bumper_contact',
            self.bumper_callback,
            10
        )

        # The TurtleBot simulation accepts an unstamped Twist on
        # /cmd_vel_unstamped. motion_control consumes this topic
        # and applies the command to the simulated robot.
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/cmd_vel_unstamped',
            10
        )

        # Run behavior arbitration at 10 Hz.
        # 0.1 seconds between updates = 10 updates per second.
        self.control_timer = self.create_timer(
            0.1,
            self.control_loop
        )

        # ---------------------------------------------------------
        # Project constants
        # ---------------------------------------------------------

        # 0.3048 meters = exactly 1 foot.
        # The assignment specifies obstacle reactions for objects
        # within 1 foot of the robot.
        self.obstacle_distance = 0.3048

        # Forward driving speed in meters per second.
        # A relatively slow value gives the robot time to react
        # to nearby obstacles in simulation.
        self.forward_speed = 0.15

        # Turning speed in radians per second.
        # This is used for avoid, escape, and random-turn behaviors.
        self.turn_speed = 0.5

        # If left and right obstacle distances differ by no more
        # than 0.08 m (8 cm), they are treated as roughly symmetric.
        self.symmetry_tolerance = 0.08

        # ---------------------------------------------------------
        # Keyboard state
        # ---------------------------------------------------------

        self.keyboard_command = Twist()
        self.keyboard_active = False
        self.last_keyboard_time = None

        # A keyboard command temporarily overrides autonomous
        # behavior. If no new key command arrives for 0.5 seconds,
        # autonomous control resumes.
        self.keyboard_timeout = 0.5

        # ---------------------------------------------------------
        # Bumper state
        # ---------------------------------------------------------

        self.bumper_active = False
        self.last_bumper_time = None

        # /bumper_contact publishes repeatedly while physical
        # contact exists. If no new bumper contact arrives for
        # 0.30 seconds, the collision is considered cleared.
        self.bumper_clear_timeout = 0.30

        # ---------------------------------------------------------
        # LiDAR state
        # ---------------------------------------------------------

        self.scan_received = False
        self.scan_number = 0

        self.left_distance = float('inf')
        self.center_distance = float('inf')
        self.right_distance = float('inf')

        # The simulated LiDAR reports a full 360-degree scan.
        #
        # TF showed that the LiDAR frame is rotated +90 degrees
        # relative to base_link. Therefore:
        #
        #     TurtleBot forward = approximately -90 degrees
        #     in the LaserScan coordinate frame.
        #
        # We use the robot's front 180 degrees:
        #
        #   right:  -180 to -120 degrees
        #   center: -120 to  -60 degrees
        #   left:    -60 to    0 degrees
        #
        # Objects behind the robot are ignored because they should
        # not affect forward obstacle avoidance.

        # The simulated LiDAR also has a minimum valid range
        # around 0.164 m. Objects closer than that can fall into
        # the sensor's blind region, which is why the bumper
        # remains the highest-priority collision behavior.

        # Used to prevent one stale LiDAR scan from immediately
        # triggering another escape after an escape completes.
        self.waiting_for_new_scan_after_escape = False
        self.escape_completion_scan_number = -1

        # ---------------------------------------------------------
        # Escape behavior state
        # ---------------------------------------------------------

        self.escape_active = False
        self.escape_end_time = None

        # pi radians = 180 degrees.
        # The assignment requires the robot to escape roughly
        # 180 +/- 30 degrees away from symmetric obstacles.
        self.escape_angle = math.pi

        # Approximate duration needed to rotate 180 degrees:
        #
        # duration = angle / angular velocity
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

    # -------------------------------------------------------------
    # Sensor / input callbacks
    # -------------------------------------------------------------

    def scan_callback(self, msg):
        """
        Process the front 180 degrees of the LiDAR.

        Because the LiDAR frame is rotated +90 degrees relative
        to base_link, -90 degrees in /scan corresponds to the
        TurtleBot's forward direction.
        """

        self.scan_received = True
        self.scan_number += 1

        left_ranges = []
        center_ranges = []
        right_ranges = []

        for i, distance in enumerate(msg.ranges):

            # Ignore invalid readings.
            if math.isnan(distance) or math.isinf(distance):
                continue

            # Ignore distances outside the valid range reported
            # by the LiDAR sensor.
            if distance < msg.range_min or distance > msg.range_max:
                continue

            angle = (
                msg.angle_min
                + (i * msg.angle_increment)
            )

            angle_degrees = math.degrees(angle)

            # Front-right:
            # Robot-relative -90 through -30 degrees.
            # LiDAR-relative -180 through -120 degrees.
            if -180.0 <= angle_degrees < -120.0:
                right_ranges.append(distance)

            # Front-center:
            # Robot-relative -30 through +30 degrees.
            # LiDAR-relative -120 through -60 degrees.
            elif -120.0 <= angle_degrees <= -60.0:
                center_ranges.append(distance)

            # Front-left:
            # Robot-relative +30 through +90 degrees.
            # LiDAR-relative -60 through 0 degrees.
            elif -60.0 < angle_degrees <= 0.0:
                left_ranges.append(distance)

        # Store the closest obstacle detected in each region.
        # infinity means that no valid obstacle was detected there.
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

        # After an escape finishes, require at least one new scan
        # before another escape may begin.
        if (
            self.waiting_for_new_scan_after_escape
            and self.scan_number > self.escape_completion_scan_number
        ):
            self.waiting_for_new_scan_after_escape = False

    def one_foot_callback(self, msg):
        """
        Request a random turn after the distance tracker reports
        approximately one foot of movement.
        """

        if msg.data:
            self.random_turn_requested = True

            self.get_logger().info(
                'Random turn requested after 1 foot'
            )

    def keyboard_callback(self, msg):
        """
        Store the newest keyboard movement command.
        """

        self.keyboard_command = msg
        self.keyboard_active = True
        self.last_keyboard_time = self.current_time_seconds()

    def bumper_callback(self, msg):
        """
        Detect contact involving the TurtleBot's bumper.
        """

        for contact in msg.contacts:
            collision1 = contact.collision1.name
            collision2 = contact.collision2.name

            if (
                'turtlebot4::bumper::bumper_collision' in collision1
                or
                'turtlebot4::bumper::bumper_collision' in collision2
            ):
                # Repeated contact messages update this timestamp.
                # As long as messages keep arriving, the bumper
                # behavior remains active.
                self.bumper_active = True
                self.last_bumper_time = self.current_time_seconds()

                return

    # -------------------------------------------------------------
    # Main behavior arbitration
    # -------------------------------------------------------------

    def control_loop(self):
        """
        Select and execute the highest-priority active behavior.
        """

        current_time = self.current_time_seconds()

        # ---------------------------------------------------------
        # Priority 1: Bumper halt
        # ---------------------------------------------------------

        if self.bumper_active:

            # Continue halting while bumper messages are still
            # arriving recently.
            if (
                self.last_bumper_time is not None
                and
                current_time - self.last_bumper_time
                <= self.bumper_clear_timeout
            ):
                self.stop_robot()
                return

            # Physical contact has cleared.
            # LiDAR behaviors will decide what to do next.
            self.bumper_active = False
            self.last_bumper_time = None

        # ---------------------------------------------------------
        # Priority 2: Keyboard control
        # ---------------------------------------------------------

        if self.keyboard_active:

            if (
                self.last_keyboard_time is not None
                and
                current_time - self.last_keyboard_time
                <= self.keyboard_timeout
            ):
                self.cmd_publisher.publish(
                    self.keyboard_command
                )
                return

            # No recent keyboard input, so allow autonomous
            # behaviors to resume.
            self.keyboard_active = False

        # Autonomous behavior cannot operate until a real LiDAR
        # scan has been received.
        if not self.scan_received:
            return

        # ---------------------------------------------------------
        # Priority 3: Escape
        # ---------------------------------------------------------

        # Escape is a fixed-action pattern.
        # Once it begins, it continues even if the original
        # obstacle stimulus changes.
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
        # Priority 4: Avoid
        # ---------------------------------------------------------

        # Avoid is reflexive: it runs only while an asymmetric
        # obstacle remains within the detection distance.
        if obstacle_present:
            self.avoid_obstacle()
            return

        # ---------------------------------------------------------
        # Priority 5: Random turn
        # ---------------------------------------------------------

        if self.random_turn_active:
            self.continue_random_turn()
            return

        if self.random_turn_requested:
            self.start_random_turn()
            return

        # ---------------------------------------------------------
        # Priority 6: Forward
        # ---------------------------------------------------------

        self.drive_forward()

    # -------------------------------------------------------------
    # Obstacle processing
    # -------------------------------------------------------------

    def obstacle_in_front(self):
        """
        Return True when any front LiDAR region contains an
        obstacle within the required 1-foot distance.
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
        Determine whether nearby obstacles are roughly symmetric.
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

        # A symmetric obstacle should appear on both sides
        # of the robot's forward direction.
        if not left_close or not right_close:
            return False

        difference = abs(
            self.left_distance
            - self.right_distance
        )

        # Left and right readings within 8 cm of each other
        # are considered roughly symmetric.
        if difference <= self.symmetry_tolerance:
            return True

        # If the center is blocked as well, allow slightly more
        # variation between the left and right measurements.
        if (
            center_close
            and difference <= self.symmetry_tolerance * 2.0
        ):
            return True

        return False

    # -------------------------------------------------------------
    # Forward behavior
    # -------------------------------------------------------------

    def drive_forward(self):
        """
        Lowest-priority autonomous behavior.
        """

        command = Twist()

        command.linear.x = self.forward_speed
        command.angular.z = 0.0

        self.cmd_publisher.publish(command)

    # -------------------------------------------------------------
    # Avoid behavior
    # -------------------------------------------------------------

    def avoid_obstacle(self):
        """
        Reflexively turn away from the closer side of an
        asymmetric obstacle.
        """

        command = Twist()

        command.linear.x = 0.0

        if self.left_distance < self.right_distance:
            # Obstacle is closer on the left.
            # Negative angular velocity turns right.
            command.angular.z = -self.turn_speed

        elif self.right_distance < self.left_distance:
            # Obstacle is closer on the right.
            # Positive angular velocity turns left.
            command.angular.z = self.turn_speed

        else:
            # If both sides happen to match but the obstacle was
            # not classified as symmetric, choose a consistent
            # direction rather than remaining stationary.
            command.angular.z = self.turn_speed

        self.cmd_publisher.publish(command)

    # -------------------------------------------------------------
    # Escape behavior
    # -------------------------------------------------------------

    def start_escape(self):
        """
        Begin an approximately 180-degree fixed-action turn.
        """

        self.escape_active = True

        current_time = self.current_time_seconds()

        self.escape_end_time = (
            current_time
            + self.escape_duration
        )

        self.get_logger().info(
            'Behavior: ESCAPE started'
        )

    def continue_escape(self):
        """
        Continue the fixed-action escape until the approximate
        180-degree turn has completed.
        """

        current_time = self.current_time_seconds()

        if current_time >= self.escape_end_time:

            self.escape_active = False
            self.escape_end_time = None

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

    # -------------------------------------------------------------
    # Random-turn behavior
    # -------------------------------------------------------------

    def start_random_turn(self):
        """
        Start a uniformly sampled turn between -15 and +15
        degrees as required by the project.
        """

        self.random_turn_requested = False
        self.random_turn_active = True

        # Uniformly choose an angle in the project's required
        # +/-15 degree range.
        angle_degrees = random.uniform(
            -15.0,
            15.0
        )

        self.random_turn_angle = math.radians(
            angle_degrees
        )

        # Positive angular velocity turns left.
        # Negative angular velocity turns right.
        if self.random_turn_angle >= 0.0:
            self.random_turn_direction = 1.0
        else:
            self.random_turn_direction = -1.0

        # The TurtleBot accepts angular velocity rather than an
        # absolute target angle, so approximate the required
        # rotation using:
        #
        # duration = angle / angular velocity
        turn_duration = (
            abs(self.random_turn_angle)
            / self.turn_speed
        )

        current_time = self.current_time_seconds()

        self.random_turn_end_time = (
            current_time
            + turn_duration
        )

        self.get_logger().info(
            f'Behavior: RANDOM TURN started '
            f'({angle_degrees:.1f} degrees)'
        )

    def continue_random_turn(self):
        """
        Continue the random turn until its calculated duration
        has completed.
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
            self.random_turn_direction
            * self.turn_speed
        )

        self.cmd_publisher.publish(command)

    # -------------------------------------------------------------
    # Utility functions
    # -------------------------------------------------------------

    def current_time_seconds(self):
        """
        Return the ROS clock time in seconds.
        """

        return (
            self.get_clock().now().nanoseconds
            / 1_000_000_000.0
        )

    def stop_robot(self):
        """
        Publish zero linear and angular velocity.
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