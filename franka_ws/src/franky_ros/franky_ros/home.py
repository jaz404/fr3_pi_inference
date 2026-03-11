import time

import rclpy
from franky_msgs.msg import GripperMove, JointMove, JointVelocity, GripperGrasp
from franky_msgs.srv import GoHome
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, Empty

# BUG: this service sometimes doesent startup if you initiate franka and need to press button. so just rerun everythign
class FrankyHomeService(Node):
    def __init__(self):
        super().__init__("franky_home_server")

        self.pub_joint_pos = self.create_publisher(JointMove, "fr3/joint_pos_cmd", 10)
        # self.pub_gripper = self.create_publisher(GripperGrasp, "fr3/gripper_grasp", 10)
        self.pub_gripper = self.create_publisher(GripperMove, "fr3/gripper_move", 10)
        self.join_motion_pub = self.create_publisher(Empty, "fr3/join_motion", 10)

        self.srv = self.create_service(GoHome, 'go_home', self.go_home)

        time.sleep(1.0)

    def go_home(self, request, response):
        self.join_motion_pub.publish(Empty())

        j_msg = JointMove()
        j_msg.positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
        j_msg.relative = False
        
        self.get_logger().info("Franky go home")
        self.pub_joint_pos.publish(j_msg)

        msg = GripperMove()
        msg.width = 0.08
        msg.speed = 0.05
        self.pub_gripper.publish(msg)

        response.success = True

        return response


def main(args=None):
    rclpy.init(args=args)
    server = FrankyHomeService()

    rclpy.spin(server)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

