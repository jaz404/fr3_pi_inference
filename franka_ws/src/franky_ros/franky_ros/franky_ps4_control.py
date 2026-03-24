import time

import numpy as np
import rclpy
from franky_msgs.msg import (
    CartesianMove,
    GripperGrasp,
    GripperMove,
    GripperState,
    JointMove,
)
from franky_msgs.srv import GoHome
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float64MultiArray


class FrankyPs4Control(Node):
    def __init__(self):
        super().__init__("franky_ps4_control")

        qos_fast = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.cli_home = self.create_client(GoHome, "go_home")
        while not self.cli_home.wait_for_service(timeout_sec=1.0):
            pass

        self.pub_cart_pose = self.create_publisher(
            CartesianMove, "fr3/cartesian_pose_cmd", qos_fast
        )
        self.pub_joint_pos = self.create_publisher(JointMove, "fr3/joint_pos_cmd", 10)
        self.pub_gripper_move = self.create_publisher(
            GripperMove, "fr3/gripper_move", qos_fast
        )
        self.pub_gripper = self.create_publisher(
            GripperGrasp, "fr3/gripper_grasp", qos_fast
        )
        self.sub_gripper = self.create_subscription(
            GripperState, "/fr3/gripper_state", self.grip_callback, qos_fast
        )

        self.sub_joy = self.create_subscription(Joy, "joy", self.joy_callback, qos_fast)
        self.create_timer(0.1, self.control_loop)

        time.sleep(1.0)
        self.get_logger().info(
            "Franky Ps4 Control Node Started. Listening for ps4 input..."
        )

        self.joy = None
        self.grip_width = 0.08

    def joy_callback(self, msg):
        self.joy = msg

    def grip_callback(self, grip_msg):
        self.grip_width = 0.08 if grip_msg.width > 0.07 else 0.0

    def control_loop(self):
        if not self.joy:
            return

        if self.joy.buttons[11] == 1.0:
            # this will actually block our xbox control so we dont need to do anything with future
            # but might want to make this a little more logical
            self.cli_home.call_async(GoHome.Request())

        # 1. Cartesian (Left Stick + Bumpers)
        x = self.joy.axes[1] * 0.02  # Left Stick Y -> X
        y = self.joy.axes[0] * 0.02  # Left Stick X -> -Y
        # z = (self.joy.buttons[5] - self.joy.buttons[4]) * 0.03  # RB - LB
        z = (self.joy.axes[5] - self.joy.axes[4]) * 0.01  # RB - LB

        # 2. Rotation (Right Stick) -> Yaw/Pitch
        roll = self.joy.axes[2] * 0.05  # this joint just kinda slow ngl
        pitch = self.joy.axes[3] * 0.04
        yaw = (self.joy.buttons[1] - self.joy.buttons[2]) * 0.04  # B(1) - X(2)

        # lock for now
        pitch = 0
        yaw = 0

        self.get_logger().info(f"{x:3f} {y:3f} {z:3f} | {yaw:3f} {pitch:3f} {roll:3f}")

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

        if self.joy.buttons[10] == 1:
            grip_width = 0.0
        # elif self.joy.axes[2] < -0.5:
        elif self.joy.buttons[9] == 1:
            grip_width = 0.08
        else:
            grip_width = self.grip_width

        if grip_width != self.grip_width:
            self.grip_width = grip_width

            if grip_width == 0.08:
                g_msg = GripperMove()
                g_msg.width = 0.08
                g_msg.speed = 0.1
                self.pub_gripper_move.publish(g_msg)
            else:
                g_msg = GripperGrasp()
                g_msg.width = 0.0
                g_msg.speed = 0.1
                g_msg.force = 5.0
                g_msg.epsilon_inner = 0.0
                g_msg.epsilon_outer = 0.08
                self.pub_gripper.publish(g_msg)


def main(args=None):
    rclpy.init(args=args)
    control = FrankyPs4Control()

    try:
        rclpy.spin(control)
    except KeyboardInterrupt:
        pass
    finally:
        control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
