#!/usr/bin/env python3
'''
still needs looking at! 

idea is pi0 server runs outside of ROS <--> UDP <--> udp_to_franky <--> franky_bridge <--> fr3 

'''
import socket
import struct
import time

import rclpy
from rclpy.node import Node

from franky_msgs.msg import JointMove, GripperMove, GripperGrasp


class UdpToFrankyRos(Node):
    def __init__(self):
        super().__init__("udp_to_franky_ros")

        self.declare_parameter("listen_ip", "0.0.0.0")
        self.declare_parameter("listen_port", 9090)
        self.declare_parameter("joint_scale", 0.08)

        self.listen_ip = self.get_parameter("listen_ip").value
        self.listen_port = self.get_parameter("listen_port").value
        self.joint_scale = self.get_parameter("joint_scale").value

        self.fmt = "<8d"
        self.packet_size = struct.calcsize(self.fmt)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.listen_ip, self.listen_port))
        self.sock.setblocking(False)

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

        self.last_gripper_mode = None
        self.last_grip_time = 0.0
        self.gripper_cooldown = 0.5

        self.gripper_speed = 0.02
        self.gripper_force = 20.0

        self.timer = self.create_timer(0.005, self.poll_udp)

        self.get_logger().info(
            f"Listening for UDP Pi0 actions on {self.listen_ip}:{self.listen_port}"
        )

    def poll_udp(self):
        try:
            data, _ = self.sock.recvfrom(2048)
        except BlockingIOError:
            return

        if len(data) < self.packet_size:
            self.get_logger().warn(
                f"Short UDP packet: {len(data)} bytes, expected {self.packet_size}"
            )
            return

        act = struct.unpack(self.fmt, data[: self.packet_size])
        self.send_franky_commands(act)

    def send_franky_commands(self, act):
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
            f"Published joint delta: {dq}, gripper={act[7]}"
        )

        self.handle_gripper(act[7])

    def handle_gripper(self, gripper_action):
        if gripper_action <= 0.0:
            mode = "open"
        elif gripper_action >= 1.0:
            mode = "close"
        else:
            mode = self.last_gripper_mode

        now = time.time()

        if mode is None:
            return

        if mode == self.last_gripper_mode:
            return

        if now - self.last_grip_time < self.gripper_cooldown:
            return

        if mode == "open":
            msg = GripperMove()
            msg.width = 0.08
            msg.speed = self.gripper_speed
            self.gripper_move_pub.publish(msg)
            self.get_logger().info("Published gripper open")

        elif mode == "close":
            msg = GripperGrasp()
            msg.width = 0.0
            msg.speed = self.gripper_speed
            msg.force = self.gripper_force
            msg.epsilon_inner = 0.0
            msg.epsilon_outer = 1.0
            self.gripper_grasp_pub.publish(msg)
            self.get_logger().info("Published gripper close")

        self.last_gripper_mode = mode
        self.last_grip_time = now

    def destroy_node(self):
        self.sock.close()
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