#!/usr/bin/env python3
"""
pi0 policy server outside ROS <--> UDP (udp_to_franky_ros) <--> ROS topics (franky_bridge) <--> FR3
"""

import socket
import struct
import time
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from franky_msgs.msg import (
    JointMove,
    GripperMove,
    GripperGrasp,
    GripperState,
)

class UdpToFrankyRos(Node):
    def __init__(self):
        super().__init__("udp_to_franky_ros")

        # Parameters
        self.declare_parameter("listen_ip", "0.0.0.0")
        self.declare_parameter("listen_port", 9090)

        self.declare_parameter("state_dst_ip", "10.1.38.195")
        self.declare_parameter("state_dst_port", 9091)
        self.declare_parameter("state_rate_hz", 50.0)

        # joint scale may need adjustment 
        # taken from jun jin's script
        self.declare_parameter("joint_scale", 0.08) 

        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = self.get_parameter("listen_port").value

        self.state_dst_ip = self.get_parameter("state_dst_ip").value
        self.state_dst_port = self.get_parameter("state_dst_port").value
        self.state_dst = (self.state_dst_ip, self.state_dst_port)

        self.state_rate_hz = self.get_parameter("state_rate_hz").value
        self.joint_scale = self.get_parameter("joint_scale").value

        # ---------------------------------------------------------------------------
        # UDP formats
        # ---------------------------------------------------------------------------
        # PC1 -> robot
        # act[0:7] = normalized joint deltas
        # act[7]   = gripper command
        self.action_fmt = "<8d"
        self.action_packet_size = struct.calcsize(self.action_fmt)

        # robot -> PC1
        # x,y,z, roll,pitch,yaw, gripper, q1..q7
        self.state_fmt = "<14d"
        self.state_packet_size = struct.calcsize(self.state_fmt)

        # ---------------------------------------------------------------------------
        # UDP sockets
        # ---------------------------------------------------------------------------
        self.action_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.action_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.action_sock.bind((self.listen_ip, self.listen_port))
        self.action_sock.setblocking(False)

        self.state_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Latest robot state cache
        self.latest_joint_positions = None  # 7 joints
        self.latest_xyz = None              # x,y,z
        self.latest_rpy = None              # roll,pitch,yaw
        self.latest_gripper = 0.0           # 0=open, 1=closed (convention from pi0)

        # Publishers: commands to franky bridge
        self.joint_pub = self.create_publisher(
            JointMove,
            "fr3/joint_pos_cmd",
            10,
        )

        self.gripper_move_pub = self.create_publisher(
            GripperMove,
            "fr3/gripper_move",
            10,
        )

        self.gripper_grasp_pub = self.create_publisher(
            GripperGrasp,
            "fr3/gripper_grasp",
            10,
        )

        # Subscribers: state from franky bridge
        self.joint_sub = self.create_subscription(
            JointState,
            "fr3/joint_states",
            self.joint_state_callback,
            10,
        )

        self.ee_pose_sub = self.create_subscription(
            PoseStamped,
            "fr3/end_effector_pose",
            self.ee_pose_callback,
            10,
        )

        self.gripper_state_sub = self.create_subscription(
            GripperState,
            "fr3/gripper_state",
            self.gripper_state_callback,
            10,
        )

        # Gripper command state
        self.last_gripper_mode = None
        self.last_grip_time = 0.0
        self.gripper_cooldown = 0.5

        # values dervied from jun jin's scripts 
        # self.gripper_speed = 0.02
        # self.gripper_force = 20.0

        # Timers
        
        # 200 Hz action polling
        self.action_timer = self.create_timer(0.005, self.poll_udp_action)
        
        # 50 Hz state_rate_hz
        state_period = 1.0 / self.state_rate_hz
        self.state_timer = self.create_timer(state_period, self.send_state_udp)

        self.get_logger().info(
            f"Listening for UDP actions on {self.listen_ip}:{self.listen_port} as {self.action_fmt}"
        )
        self.get_logger().info(
            f"Sending UDP state to {self.state_dst_ip}:{self.state_dst_port} as {self.state_fmt}"
        )

    # ROS state callbacks
    def joint_state_callback(self, msg: JointState):
        if len(msg.position) < 7:
            self.get_logger().warn(
                f"JointState has only {len(msg.position)} positions, expected 7"
            )
            return

        self.latest_joint_positions = list(msg.position[:7])

    def ee_pose_callback(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation

        self.latest_xyz = [p.x, p.y, p.z]
        self.latest_rpy = self.quat_to_rpy(q.w, q.x, q.y, q.z)

    def gripper_state_callback(self, msg: GripperState):
        """
        Converts Franka gripper width to Pi0 convention:
            width = 0.08 m  -> 0.0 open
            width = 0.00 m  -> 1.0 closed
        """
        max_open = 0.08
        min_open = 0.0

        width = float(msg.width)
        width = max(min_open, min(max_open, width))

        self.latest_gripper = 1.0 - (width - min_open) / (max_open - min_open)

        self.latest_gripper_is_grasped = bool(msg.is_grasped)

    # UDP state publisher: ROS state -> PC1
    def send_state_udp(self):
        if self.latest_xyz is None:
            return

        if self.latest_rpy is None:
            return

        if self.latest_joint_positions is None:
            return

        x, y, z = self.latest_xyz
        roll, pitch, yaw = self.latest_rpy
        g = self.latest_gripper
        q1, q2, q3, q4, q5, q6, q7 = self.latest_joint_positions

        pkt = struct.pack(
            self.state_fmt,
            x, y, z,
            roll, pitch, yaw,
            g,
            q1, q2, q3, q4, q5, q6, q7,
        )

        try:
            self.state_sock.sendto(pkt, self.state_dst)
        except Exception as e:
            self.get_logger().error(f"UDP state send error: {e}")

    # UDP action receiver: PC1 -> ROS commands
    def poll_udp_action(self):
        latest_data = None

        # Drain socket so robot uses the newest action, not old queued actions.
        while True:
            try:
                data, _ = self.action_sock.recvfrom(2048)
                latest_data = data
            except BlockingIOError:
                break

        if latest_data is None:
            return

        if len(latest_data) < self.action_packet_size: # has to be 8
            self.get_logger().warn(
                f"Short UDP action packet: {len(latest_data)} bytes, expected {self.action_packet_size}"
            )
            return

        act = struct.unpack(self.action_fmt, latest_data[:self.action_packet_size])
        self.send_franky_commands(act)

    def send_franky_commands(self, act):
        # the policy will give joint deltas and not joint angles (makes sense to apply joint scaling here)
        # q_new = q_current + dq
        dq = [
            self.joint_scale * act[0],
            self.joint_scale * act[1],
            self.joint_scale * act[2],
            self.joint_scale * act[3],
            self.joint_scale * act[4],
            self.joint_scale * act[5],
            self.joint_scale * act[6],
        ]

        joint_msg = JointMove()
        joint_msg.positions = dq
        joint_msg.relative = True

        self.joint_pub.publish(joint_msg)

        self.get_logger().info(
            f"Published relative joint delta: {dq}, gripper={act[7]}"
        )

        self.handle_gripper(act[7])

    def handle_gripper(self, gripper_action):
        """
        Pi0 convention:
            gripper_action <= 0.0 -> open, width 0.08
            gripper_action >= 1.0 -> close/grasp, width 0.0
        """

        if gripper_action <= 0.0:
            target_width = 0.08
            mode = "open"
        elif gripper_action >= 1.0:
            target_width = 0.0
            mode = "close"
        else:
            return

        now = time.time()

        if mode == self.last_gripper_mode:
            return

        if now - self.last_grip_time < self.gripper_cooldown:
            return

        if target_width == 0.08:
            msg = GripperMove()
            msg.width = 0.08
            msg.speed = 0.1
            self.gripper_move_pub.publish(msg)
            self.get_logger().info("Published gripper open width=0.08")

        else:
            msg = GripperGrasp()
            msg.width = float(target_width)
            msg.speed = 0.1
            msg.force = 5.0
            msg.epsilon_inner = 0.0
            msg.epsilon_outer = 0.08
            self.gripper_grasp_pub.publish(msg)
            self.get_logger().info("Published gripper grasp width=0.0")

        self.last_gripper_mode = mode
        self.last_grip_time = now

    # Helpers
    def quat_to_rpy(self, w, x, y, z):
        '''
        converts quat to roll, pitch, yaw
        franky bridge publishes in quat so requires conversion 
        '''
        # roll
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        # pitch
        t2 = 2.0 * (w * y - z * x)
        t2 = max(-1.0, min(1.0, t2))
        pitch = math.asin(t2)

        # yaw
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return [roll, pitch, yaw]

    def destroy_node(self):
        try:
            self.action_sock.close()
            self.state_sock.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UdpToFrankyRos()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()