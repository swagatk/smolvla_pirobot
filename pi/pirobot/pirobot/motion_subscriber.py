"""
Subscriber code for Robot motion
"""
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pirobot import motor_control as mc

class MotionSubscriber(Node):
    def __init__(self):
        super().__init__('motion_subscriber')
        self.subscription = self.create_subscription(
            String,
            '/pirobot/command',
            self.listener_callback,
            10)
        self.subscription # prevents unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        cmd_str = msg.data
        mc.exec_cmd(cmd_str)

def main(args=None):
    rclpy.init(args=args)
    mc.initialize_motors()
    motion_subscriber = MotionSubscriber()
    rclpy.spin(motion_subscriber)
    motion_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
