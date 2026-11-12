"""Robot motion: /pirobot/command (std_msgs/String) -> motors.

Runs on the Raspberry Pi.

    ros2 run pirobot motion_subscriber
    ros2 run pirobot motion_subscriber --ros-args -p timeout:=0.5

Commands: 'UP', 'DOWN', 'LEFT', 'RIGHT'; anything else (incl. 'STOP',
'EXIT') stops the motors.

Safety watchdog: if no command arrives for `timeout` seconds (WiFi drop,
teleop or bridge crashed) the motors are stopped. keyboard_publisher and
smolvla_bridge both publish continuously, so this never triggers in normal
use. timeout:=0 disables it.
"""
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from pirobot import motor_control as mc


class MotionSubscriber(Node):

    def __init__(self):
        super().__init__('motion_subscriber')
        self.declare_parameter('topic', '/pirobot/command')
        self.declare_parameter('timeout', 0.5, ParameterDescriptor(dynamic_typing=True))
        self.timeout = float(self.get_parameter('timeout').value)
        self.subscription = self.create_subscription(
            String, self.get_parameter('topic').value, self.listener_callback, 10)
        self.current = 'STOP'
        self.last_msg_time = None
        if self.timeout > 0:
            self.create_timer(0.1, self.watchdog)
        self.get_logger().info(
            f'Listening on {self.get_parameter("topic").value}, '
            f'watchdog {self.timeout:.2f} s')
        self.get_logger().info(
            f'Duty A={mc.DutyCycleA:g} B={mc.DutyCycleB:g} @ {mc.Frequency:g} Hz '
            f'(calibration: {mc.CALIB_FILE})')

    def apply(self, cmd):
        cmd = cmd if cmd in mc.ACTIVE_COMMANDS else 'STOP'
        if cmd != self.current:
            mc.exec_cmd(cmd)
            self.get_logger().info(f'Motion: {self.current} -> {cmd}')
            self.current = cmd

    def listener_callback(self, msg):
        self.last_msg_time = time.monotonic()
        self.apply(msg.data.strip().upper())

    def watchdog(self):
        if self.current == 'STOP' or self.last_msg_time is None:
            return
        if time.monotonic() - self.last_msg_time > self.timeout:
            self.get_logger().warn(
                f'No command for {self.timeout:.2f} s: stopping motors')
            self.apply('STOP')


def main(args=None):
    rclpy.init(args=args)
    mc.initialize_motors()
    node = None
    try:
        node = MotionSubscriber()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        mc.cleanup()   # motors off and GPIO released, whatever happened
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
