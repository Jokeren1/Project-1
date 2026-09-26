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

    Full behavior priority during integration:
    bumper > keyboard > escape > avoid > random turn > forward
    """

    def __init__(self):
        super().__init__('reactive_controller')

        # TODO: Confirm TurtleBot 4 LiDAR topic during integration.
        self.scan_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Temporary autonomous velocity output.
        # TODO: Coordinate final topic with keyboard/bumper integration.
        self.cmd_publisher = self.create_publisher(
            Twist,
            '/reactive_cmd_vel',
            10
        )

        self.latest_scan = None

        # Project requirement: obstacles within 1 ft.
        self.obstacle_distance = 0.3048

        # TODO: Tune these values during Gazebo testing.
        self.forward_speed = 0.15
        self.turn_speed = 0.5

        # State variables used by fixed-action escape/random-turn behaviors.
        self.escape_active = False
        self.random_turn_active = False

        self.get_logger().info('Reactive controller initialized')

    def scan_callback(self, msg):
        """Store and process the most recent LiDAR scan."""
        self.latest_scan = msg

        # TODO:
        # 1. Extract front-facing LiDAR measurements.
        # 2. Compare left/right obstacle distances.
        # 3. Detect approximately symmetric obstacles.
        # 4. Select escape, avoid, random-turn, or forward behavior.

    def drive_forward(self):
        """Lowest-priority autonomous behavior."""
        command = Twist()
        command.linear.x = self.forward_speed
        self.cmd_publisher.publish(command)

    def avoid_obstacle(self):
        """Reflexively turn away from an asymmetric obstacle."""
        # TODO: Determine turn direction from LiDAR measurements.
        pass

    def start_escape(self):
        """
        Begin fixed-action escape behavior.

        Escape should continue once triggered until the robot has turned
        approximately 180 degrees, even if the original obstacle disappears.
        """
        self.escape_active = True

        # TODO: Track completed rotation and stop around 180 degrees.

    def start_random_turn(self):
        """
        Begin a uniformly sampled turn between -15 and +15 degrees after
        approximately 1 ft of forward movement.
        """
        self.random_turn_active = True

        # TODO: Receive distance event and generate random target angle.


def main(args=None):
    rclpy.init(args=args)

    controller = ReactiveController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
