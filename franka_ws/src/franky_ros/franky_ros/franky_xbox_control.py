import time

import numpy as np
import rclpy
from franky_msgs.msg import CartesianMove, GripperGrasp, GripperMove, JointMove
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float64MultiArray


class FrankyXboxControl(Node):
    def __init__(self):
        super().__init__("franky_xbox_control")

        qos_fast = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.demo_start = CartesianMove()
        self.demo_start.relative = False
        (
            self.demo_start.pose.position.x,
            self.demo_start.pose.position.y,
            self.demo_start.pose.position.z,
        ) = (
            float(0.3082970380783081),
            float(0.007530250586569309),
            float(0.4771515130996704),
        )
        (
            self.demo_start.pose.orientation.x,
            self.demo_start.pose.orientation.y,
            self.demo_start.pose.orientation.z,
            self.demo_start.pose.orientation.w,
        ) = [float(v) for v in [1, 0, 0, 0]]

        self.pub_cart_pose = self.create_publisher(
            CartesianMove, "fr3/cartesian_pose_cmd", qos_fast
        )
        self.pub_joint_pos = self.create_publisher(JointMove, "fr3/joint_pos_cmd", 10)
        # self.pub_gripper = self.create_publisher(GripperMove, "fr3/gripper_move", qos_fast)
        self.pub_gripper = self.create_publisher(
            GripperGrasp, "fr3/gripper_grasp", qos_fast
        )

        self.sub_joy = self.create_subscription(Joy, "joy", self.joy_callback, qos_fast)
        self.create_timer(0.1, self.control_loop)

        time.sleep(1.0)
        self.get_logger().info(
            "Franky Xbox Control Node Started. Listening for xbox input..."
        )

        self.joy = None
        self.grip_width = 0.08

    def joy_callback(self, msg):
        self.joy = msg

    def control_loop(self):
        if not self.joy:
            return

        if self.joy.axes[7] == 1.0:
            self.pub_cart_pose.publish(self.demo_start)
            return

        # 1. Cartesian (Left Stick + Bumpers)
        x = self.joy.axes[1] * 0.02  # Left Stick Y -> X
        y = self.joy.axes[0] * 0.02  # Left Stick X -> -Y
        # z = (self.joy.buttons[5] - self.joy.buttons[4]) * 0.03  # RB - LB
        z = (self.joy.axes[5] - self.joy.axes[2]) * 0.01  # RB - LB

        # 2. Rotation (Right Stick) -> Yaw/Pitch
        roll = self.joy.axes[3] * 0.05  # this joint just kinda slow ngl
        pitch = self.joy.axes[4] * 0.04
        yaw = (self.joy.buttons[1] - self.joy.buttons[2]) * 0.04  # B(1) - X(2)

        # lock for now
        pitch = 0
        yaw = 0

        quat = Rotation.from_euler("xyz", [yaw, pitch, roll], degrees=False).as_quat()

        # Publish Move if inputs exist
        if any(abs(v) > 0.001 for v in [x, y, z, yaw, pitch, roll]):
            msg = CartesianMove()
            msg.relative = True
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = (
                float(x),
                float(y),
                float(z),
            )
            (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ) = [float(v) for v in quat]
            self.pub_cart_pose.publish(msg)

        # 3. Gripper (Triggers)
        # if self.joy.axes[5] < -0.5:

        if self.joy.buttons[5] == 1:
            grip_width = 0.0
        # elif self.joy.axes[2] < -0.5:
        elif self.joy.buttons[4] == 1:
            grip_width = 0.08
        else:
            grip_width = self.grip_width

        if grip_width != self.grip_width:
            self.grip_width = grip_width

            # g_msg = GripperMove()
            g_msg = GripperGrasp()
            g_msg.width = float(grip_width)
            g_msg.speed = 0.1
            g_msg.force = 0.01
            g_msg.epsilon_inner = 0.3
            g_msg.epsilon_outer = 0.3
            self.pub_gripper.publish(g_msg)


def main(args=None):
    rclpy.init(args=args)
    control = FrankyXboxControl()

    try:
        rclpy.spin(control)
    except KeyboardInterrupt:
        pass
    finally:
        control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
