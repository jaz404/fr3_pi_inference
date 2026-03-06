import time

import rclpy
from franky_msgs.msg import GripperMove, JointMove, JointVelocity
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from std_msgs.msg import Empty, Float64, Float64MultiArray


class FrankyTestClient(Node):
    def __init__(self):
        super().__init__("franky_test_client")

        self.pub_joint_pos = self.create_publisher(JointMove, "fr3/joint_pos_cmd", 10)
        self.pub_joint_vel = self.create_publisher(
            JointVelocity, "fr3/joint_vel_cmd", 10
        )
        self.pub_gripper = self.create_publisher(GripperMove, "fr3/gripper_move", 10)
        self.join_motion_pub = self.create_publisher(Empty, "fr3/join_motion", 10)
        # self.pub_cart_vel = self.create_publisher(Twist, "fr3/cartesian_vel_cmd", 10)

        time.sleep(1.0)

    def send_joint_position(self, arr):
        msg = JointMove()
        msg.positions = arr
        msg.relative = False
        self.get_logger().info("Sending Joint Position Command...")
        self.pub_joint_pos.publish(msg)

    def send_joint_velocity(self, arr):
        msg = JointVelocity()
        msg.vels = arr
        msg.duration_millis = 1000
        self.get_logger().info("Sending Joint Velocity Command...")
        self.pub_joint_vel.publish(msg)

    def send_gripper_move(self, width):
        msg = GripperMove()
        msg.width = width
        msg.speed = 0.05  # m/s
        self.get_logger().info(f"Sending Gripper Move: {width}m")
        self.pub_gripper.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    tester = FrankyTestClient()

    tester.join_motion_pub.publish(Empty())  # Start joint motion mode
    tester.send_joint_position([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    # tester.send_joint_position([0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0])
    # tester.send_joint_velocity([0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # time.sleep(3.0)
    # tester.send_joint_velocity([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    # tester.send_gripper_move(0.04)  # Open halfway

    # Keep alive briefly to ensure message delivery
    time.sleep(1.0)
    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
