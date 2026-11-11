#!/usr/bin/env python3
"""Publish PiCamera2 frames as JPEG sensor_msgs/CompressedImage.

Runs on the Raspberry Pi (ROS2 Jazzy, system Python).

    ros2 run pirobot picam_publisher --ros-args -p width:=320 -p height:=240 -p fps:=15

Topic: /camera/image/compressed  (best-effort QoS, depth 1, so a slow WiFi
link drops old frames instead of queueing them).

JPEG over ROS keeps bandwidth to roughly 10-20 KB per 320x240 frame, which a
home LAN handles easily at 15 fps.

full_fov (default true): at small output sizes libcamera picks the sensor
mode closest in size, which on the IMX219 (Camera v2) is a 640x480 centre
crop with only ~40% of the lens' horizontal field of view. With full_fov the
node instead picks the smallest sensor mode that covers the whole sensor
(1640x1232 binned on the IMX219) and scales it down on the ISP, which is free.
"""
import cv2
import rclpy
from libcamera import Transform
from picamera2 import Picamera2
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


def full_fov_mode(cam, fps):
    """Smallest sensor mode whose crop covers the full sensor and reaches fps."""
    modes = cam.sensor_modes
    if not modes:
        return None
    full_w = max(m['crop_limits'][2] for m in modes)
    full_h = max(m['crop_limits'][3] for m in modes)
    ok = [m for m in modes
          if m['crop_limits'][2] >= full_w and m['crop_limits'][3] >= full_h
          and m['fps'] >= fps]
    if not ok:
        return None
    return min(ok, key=lambda m: (m['size'][0] * m['size'][1], -m['bit_depth']))


class PiCamPublisher(Node):

    def __init__(self):
        super().__init__('picam_publisher')
        dyn = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)
        self.declare_parameter('fps', 15.0, dyn)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('hflip', False)
        self.declare_parameter('vflip', False)
        self.declare_parameter('full_fov', True)
        self.declare_parameter('topic', '/camera/image/compressed')
        self.declare_parameter('frame_id', 'camera')

        w = int(self.get_parameter('width').value)
        h = int(self.get_parameter('height').value)
        fps = float(self.get_parameter('fps').value)
        self.quality = int(self.get_parameter('jpeg_quality').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.cam = Picamera2()
        sensor = {}
        if self.get_parameter('full_fov').value:
            mode = full_fov_mode(self.cam, fps)
            if mode is not None:
                sensor = {'output_size': mode['size'], 'bit_depth': mode['bit_depth']}
        # "RGB888" in libcamera is BGR byte order in memory, which is exactly
        # what cv2.imencode expects.
        cfg = self.cam.create_video_configuration(
            main={'size': (w, h), 'format': 'RGB888'},
            sensor=sensor,
            transform=Transform(
                hflip=int(self.get_parameter('hflip').value),
                vflip=int(self.get_parameter('vflip').value),
            ),
            controls={'FrameRate': fps},
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
            CompressedImage, self.get_parameter('topic').value, qos
        )
        self.timer = self.create_timer(1.0 / fps, self.tick)
        self.get_logger().info(
            f'Publishing {w}x{h} @ {fps} fps JPEG q={self.quality}, '
            f'sensor mode {sensor.get("output_size", "auto")}')

    def tick(self):
        frame = self.cam.capture_array('main')  # blocks until the next frame
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            self.get_logger().warn('JPEG encode failed')
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        if rclpy.ok(context=self.context):
            self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.cam.stop()
            self.cam.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PiCamPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
