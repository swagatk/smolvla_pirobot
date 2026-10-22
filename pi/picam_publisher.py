#!/usr/bin/env python3
"""Publish PiCamera2 frames as JPEG sensor_msgs/CompressedImage.

Runs on the Raspberry Pi (ROS2 Jazzy, system Python).

    python3 picam_publisher.py --ros-args -p width:=320 -p height:=240 -p fps:=15

Topic: /camera/image/compressed  (best-effort QoS, depth 1, so a slow WiFi
link drops old frames instead of queueing them).

JPEG over ROS keeps bandwidth to roughly 10-20 KB per 320x240 frame, which a
home LAN handles easily at 15 fps. Skip this node if you already publish the
camera; just point the recorder's `image_topic` at your topic (it must be a
CompressedImage, JPEG or PNG).
"""
import cv2
import rclpy
from libcamera import Transform
from picamera2 import Picamera2
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


class PiCamPublisher(Node):
    def __init__(self):
        super().__init__("picam_publisher")
        self.declare_parameter("width", 320)
        self.declare_parameter("height", 240)
        self.declare_parameter("fps", 15.0)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("hflip", False)
        self.declare_parameter("vflip", False)
        self.declare_parameter("topic", "/camera/image/compressed")
        self.declare_parameter("frame_id", "camera")

        w = self.get_parameter("width").value
        h = self.get_parameter("height").value
        fps = float(self.get_parameter("fps").value)
        self.quality = int(self.get_parameter("jpeg_quality").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.cam = Picamera2()
        # "RGB888" in libcamera is BGR byte order in memory, which is exactly
        # what cv2.imencode expects.
        cfg = self.cam.create_video_configuration(
            main={"size": (w, h), "format": "RGB888"},
            transform=Transform(
                hflip=int(self.get_parameter("hflip").value),
                vflip=int(self.get_parameter("vflip").value),
            ),
            controls={"FrameRate": fps},
            buffer_count=2,
        )
        self.cam.configure(cfg)
        self.cam.start()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(
            CompressedImage, self.get_parameter("topic").value, qos
        )
        self.timer = self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(f"Publishing {w}x{h} @ {fps} fps JPEG q={self.quality}")

    def tick(self):
        frame = self.cam.capture_array("main")  # blocks until the next frame
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            self.get_logger().warn("JPEG encode failed")
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = "jpeg"
        msg.data = buf.tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        self.cam.stop()
        super().destroy_node()


def main():
    rclpy.init()
    node = PiCamPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
