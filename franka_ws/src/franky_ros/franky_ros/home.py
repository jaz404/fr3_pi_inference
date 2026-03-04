import time

import rclpy
from franky_msgs.msg import CartesianMove
from rclpy.node import Node


class FrankyTestClient(Node):
    def __init__(self):
        super().__init__("franky_home")

        home = CartesianMove()
        home.relative = False
        home.pose.position.x, home.pose.position.y, home.pose.position.z = (
            float(0.3082970380783081),
            float(0.007530250586569309),
            float(0.4771515130996704),
        )
        (
            home.pose.orientation.x,
            home.pose.orientation.y,
            home.pose.orientation.z,
            home.pose.orientation.w,
        ) = [float(v) for v in [1, 0, 0, 0]]

        self.pub_joint_pos = self.create_publisher(
            CartesianMove, "fr3/cartesian_pose_cmd", 10
        )

        self.get_logger().info("Sending Home Position Command...")
        self.pub_joint_pos.publish(home)


def main(args=None):
    rclpy.init(args=args)
    tester = FrankyTestClient()
    # Keep alive briefly to ensure message delivery
    time.sleep(1.0)
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
