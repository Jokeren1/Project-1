import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts


class BumperTracker(Node):

    def __init__(self):
        super().__init__('bumper_tracker')

        self.subscription = self.create_subscription(
            Contacts,
            '/bumper_contact',
            self.bumper_callback,
            10
        )

        self.get_logger().info('Bumper tracker started')

    def bumper_callback(self, msg):
        for contact in msg.contacts:
            collision1 = contact.collision1.name
            collision2 = contact.collision2.name

            if (
                'turtlebot4::bumper::bumper_collision' in collision1
                or 'turtlebot4::bumper::bumper_collision' in collision2
            ):
                self.get_logger().info('BUMPER COLLISION DETECTED')
                return


def main(args=None):
    rclpy.init(args=args)

    node = BumperTracker()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
