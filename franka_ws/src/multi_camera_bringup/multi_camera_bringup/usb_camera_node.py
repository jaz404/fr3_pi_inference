#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class UsbCameraPublisher(Node):
    "a simple usb camera publisher"
    def __init__(self):
        super().__init__("usb_camera_publisher")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("camera_name", "logitech")
        self.declare_parameter("frame_id", "logitech_frame")
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)

        self.camera_index = self.get_parameter("camera_index").value
        self.camera_name = self.get_parameter("camera_name").value
        self.frame_id = self.get_parameter("frame_id").value
        self.fps = self.get_parameter("fps").value
        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            self.get_logger().error(f"Could not open camera index {self.camera_index}")
            return

        # topic = f"/{self.camera_name}/image_raw"
        topic = f"/cameras/{self.camera_name}/image_raw"
        self.publisher = self.create_publisher(Image, topic, 10)

        timer_period = 1.0 / self.fps
        self.timer = self.create_timer(timer_period, self.publish_frame)

        self.get_logger().info(
            f"Publishing USB camera {self.camera_index} to {topic}"
        )

    def publish_frame(self):
        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warn("Failed to read frame")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.publisher.publish(msg)

    def destroy_node(self):
        if hasattr(self, "cap"):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()