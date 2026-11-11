"""Keyboard teleop: arrow keys -> std_msgs/String on /pirobot/command.

Runs on the laptop (needs a display and python3-pygame). Publishes the held
key at a fixed rate ('UP', 'DOWN', 'LEFT', 'RIGHT', or 'STOP' when nothing is
held). The constant stream doubles as a heartbeat for the watchdog in
motion_subscriber, and gives episode_recorder one command per tick.

    ros2 run pirobot keyboard_publisher
    ros2 run pirobot keyboard_publisher --ros-args -p rate:=10.0

ESC or closing the window sends STOP and quits. Only this node quits; the
robot side keeps running.
"""
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String

from pirobot import key_press as kp


class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('keyboard_publisher')
        self.declare_parameter('topic', '/pirobot/command')
        self.declare_parameter('rate', 10.0, ParameterDescriptor(dynamic_typing=True))
        self.publisher_ = self.create_publisher(
            String, self.get_parameter('topic').value, 10)
        rate = float(self.get_parameter('rate').value)
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.last = None
        self.done = False
        self.get_logger().info(
            f'Publishing on {self.get_parameter("topic").value} at {rate} Hz. '
            'Focus the pygame window; arrows drive, ESC quits.')

    def send(self, cmd):
        self.publisher_.publish(String(data=cmd))

    def timer_callback(self):
        cmd = kp.fetch_command()
        if cmd == 'EXIT':
            self.done = True
            return
        self.send(cmd)
        if cmd != self.last:
            self.get_logger().info(f'Sending: "{cmd}"')
            kp.show(cmd)
            self.last = cmd


def main(args=None):
    # Let Ctrl-C raise KeyboardInterrupt instead of shutting the context
    # down, so a final STOP can still be published on the way out.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    kp.init()
    node = KeyboardPublisher()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            for _ in range(3):   # make sure the robot stops
                node.send('STOP')
                time.sleep(0.05)
            node.get_logger().info('Sent STOP, exiting')
        kp.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
