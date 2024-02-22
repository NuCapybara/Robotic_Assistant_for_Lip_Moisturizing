import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class Robot_Care(Node):

    def __init__(self):
        super().__init__('robot_care')
        self.lip_pose_subscriber = self.create_subscription(Point, 'lip_pose', self.pose_listener_callback, 10)
        self.lip_pose_test = None

    def pose_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)
        if self.lip_pose_test is not None:
            self.get_logger().info('set lip_pose_test: "%s"' % self.lip_pose_test)
            self.lip_pose_test = [msg.x, msg.y, msg.z]

def main(args=None):
    """
    Run the Robot_care node.

    Args:
    ----
        args: Command-line arguments.

    Returns
    -------
        None

    """
    rclpy.init(args=args)
    node = Robot_Care()
    rclpy.spin(node)
    rclpy.shutdown()