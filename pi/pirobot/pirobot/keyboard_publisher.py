"""
Keyboard Publisher
Recognizes key pressed
"""
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pirobot import key_press as kp
import sys

class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.publisher_ = self.create_publisher(
            String,
            '/pirobot/command',
            10)
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        msg = String()
        msg.data = kp.fetch_command()
        self.publisher_.publish(msg)
        self.get_logger().info('Sending: "%s"' % msg.data)
        if msg.data == 'EXIT':
            raise SystemExit

def main(args=None):
    kp.init()
    rclpy.init(args=args)
    keyboard_publisher = KeyboardPublisher()
    try:
        rclpy.spin(keyboard_publisher)
    except SystemExit:
        kp.stop()
    keyboard_publisher.destroy_node()
    kp.stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
