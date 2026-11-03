#!/usr/bin/env python3
"""ROS2 side of laptop inference. System ROS2 Jazzy Python, no torch.

Subscribes to the Pi camera, asks smolvla_server.py for an action at a fixed
rate, snaps it to a discrete command and publishes it as Twist on /cmd_vel,
which the Pi's existing motor driver already executes.

    python3 smolvla_bridge.py
    ros2 topic pub --once /smolvla/task   std_msgs/msg/String "{data: 'follow the ball'}"
    ros2 topic pub --once /smolvla/enable std_msgs/msg/Bool   "{data: true}"
    ros2 topic pub --once /smolvla/enable std_msgs/msg/Bool   "{data: false}"   # stop

Starts DISABLED: nothing moves until you publish enable=true. Disabling, a
stale camera frame or a lost server all publish an immediate stop.
Close the keyboard teleop (or put both behind twist_mux) while this runs,
since both publish /cmd_vel.
"""
import time
from multiprocessing.connection import Client

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool, String

from pirobot_actions import snap, to_cmd_str


class SmolVLABridge(Node):
    def __init__(self):
        super().__init__("smolvla_bridge")
        p = self.declare_parameter
        dyn = ParameterDescriptor(dynamic_typing=True)
        p("image_topic", "/camera/image/compressed")
        p("cmd_topic", "/cmd_vel")
        p("pirobot_cmd_topic", "/pirobot/command")  # std_msgs/String for motion_subscriber.py
        p("rate", 10.0, dyn)            # Hz; use the dataset fps
        p("task", "move around while avoiding obstacles")
        p("server_port", 6010)
        p("server_authkey", "pirobot")
        p("image_timeout", 0.5, dyn)
        p("discrete", True)             # snap to forward/back/left/right/stop
        p("stop_radius", 0.25, dyn)
        g = lambda n: self.get_parameter(n).value

        self.task = g("task")
        self.image_timeout = float(g("image_timeout"))
        self.discrete = bool(g("discrete"))
        self.stop_radius = float(g("stop_radius"))
        self.addr = ("127.0.0.1", int(g("server_port")))
        self.authkey = str(g("server_authkey")).encode()
        self.conn = None
        self.enabled = False
        self.last_img = None            # (recv_time, jpeg bytes)
        self.prev = (0.0, 0.0)

        img_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CompressedImage, g("image_topic"), self.on_image, img_qos)
        self.create_subscription(String, "/smolvla/task", self.on_task, 10)
        self.create_subscription(Bool, "/smolvla/enable", self.on_enable, 10)
        self.cmd_pub = self.create_publisher(Twist, g("cmd_topic"), 10)
        pirobot_topic = g("pirobot_cmd_topic")
        self.pirobot_cmd_pub = self.create_publisher(String, pirobot_topic, 10) if pirobot_topic else None
        self.dbg_pub = self.create_publisher(String, "/smolvla/debug", 10)
        self.create_timer(1.0 / float(g("rate")), self.tick)
        self.get_logger().info(f"Ready (disabled). Task: '{self.task}'")

    def on_image(self, msg):
        self.last_img = (time.monotonic(), bytes(msg.data))

    def on_task(self, msg):
        self.task = msg.data.strip()
        self.get_logger().info(f"Task: '{self.task}'")

    def on_enable(self, msg):
        self.enabled = bool(msg.data)
        self.get_logger().info("ENABLED" if self.enabled else "disabled")
        if not self.enabled:
            self.publish(0.0, 0.0)
        elif self.connect():
            self.request({"op": "reset"})

    def connect(self):
        if self.conn is None:
            try:
                self.conn = Client(self.addr, authkey=self.authkey)
                self.get_logger().info("Connected to smolvla_server")
            except OSError as e:
                self.get_logger().error(f"smolvla_server not reachable: {e}", throttle_duration_sec=5.0)
                return False
        return True

    def request(self, req):
        try:
            self.conn.send(req)
            return self.conn.recv()
        except (OSError, EOFError) as e:
            self.get_logger().error(f"Lost smolvla_server: {e}")
            self.conn = None
            return None

    def publish(self, v, w, name="stop"):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.cmd_pub.publish(t)
        if self.pirobot_cmd_pub is not None:
            cmd_name = name if name != "raw" else snap(v, w, self.stop_radius)[0]
            self.pirobot_cmd_pub.publish(String(data=to_cmd_str(cmd_name)))
        self.prev = (float(v), float(w))

    def tick(self):
        if not self.enabled:
            return
        if self.last_img is None or time.monotonic() - self.last_img[0] > self.image_timeout:
            self.get_logger().warn("No fresh camera frame: stopping", throttle_duration_sec=2.0)
            self.publish(0.0, 0.0, "stop")
            return
        if not self.connect():
            self.publish(0.0, 0.0, "stop")
            return
        res = self.request({"op": "act", "jpeg": self.last_img[1], "task": self.task, "prev": list(self.prev)})
        if not res or "action" not in res:
            self.publish(0.0, 0.0, "stop")
            return
        v, w = res["action"]
        name = "raw"
        if self.discrete:
            name, v, w = snap(v, w, self.stop_radius)
        self.publish(v, w, name)
        self.dbg_pub.publish(String(data=f"{name} ({to_cmd_str(name)}) v={v:+.2f} w={w:+.2f} raw={res['action']} {res['ms']:.0f}ms"))


def main():
    rclpy.init()
    node = SmolVLABridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
