#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer


class MultiCameraSyncNode(Node):
    def __init__(self):
        super().__init__("multi_camera_sync_node")

        self.declare_parameter("use_third_camera", False)
        self.use_third_camera = self.get_parameter("use_third_camera").value

        # Subscribers
        self.realsense_sub = Subscriber(
            self,
            Image,
            "/cameras/realsense/color/image_raw",
        )

        self.logitech_sub = Subscriber(
            self,
            Image,
            "/cameras/logitech/image_raw",
        )

        # Publishers for synced output
        self.realsense_synced_pub = self.create_publisher(
            Image,
            "/cameras/synced/realsense/image_raw",
            10,
        )

        self.logitech_synced_pub = self.create_publisher(
            Image,
            "/cameras/synced/logitech/image_raw",
            10,
        )

        subscribers = [
            self.realsense_sub,
            self.logitech_sub,
        ]

        if self.use_third_camera:
            self.camera_3_sub = Subscriber(
                self,
                Image,
                "/cameras/camera_3/image_raw",
            )

            self.camera_3_synced_pub = self.create_publisher(
                Image,
                "/cameras/synced/camera_3/image_raw",
                10,
            )

            subscribers.append(self.camera_3_sub)

        self.sync = ApproximateTimeSynchronizer(
            subscribers,
            queue_size=10,
            slop=0.05,
        )

        if self.use_third_camera:
            self.sync.registerCallback(self.synced_three_camera_callback)
        else:
            self.sync.registerCallback(self.synced_two_camera_callback)

        self.get_logger().info("Multi-camera sync node started")

    def synced_two_camera_callback(self, realsense_msg, logitech_msg):
        self.realsense_synced_pub.publish(realsense_msg)
        self.logitech_synced_pub.publish(logitech_msg)

        self.get_logger().info("Published synced RealSense + Logitech frames")

    def synced_three_camera_callback(self, realsense_msg, logitech_msg, camera_3_msg):
        self.realsense_synced_pub.publish(realsense_msg)
        self.logitech_synced_pub.publish(logitech_msg)
        self.camera_3_synced_pub.publish(camera_3_msg)

        self.get_logger().info("Published synced 3-camera frames")


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()