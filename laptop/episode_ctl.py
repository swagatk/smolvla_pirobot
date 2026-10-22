#!/usr/bin/env python3
"""One-key control for episode_recorder.py. Run in its own terminal.

    r  start recording            s  stop and save
    d  discard current episode    1/2/3  choose task
    t  type a custom task         q  quit

Leading and trailing idle frames (robot stopped while you switch terminals)
are trimmed later by convert_to_lerobot.py, so there is no need to rush.
"""
import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

TASKS = {
    "1": "move around while avoiding obstacles",
    "2": "follow the wall",
    "3": "follow the ball",
}
NS = "/episode_recorder"


class Ctl(Node):
    def __init__(self):
        super().__init__("episode_ctl")
        self.cli = {k: self.create_client(Trigger, f"{NS}/{k}") for k in ("start", "stop", "discard")}
        self.task_pub = self.create_publisher(String, f"{NS}/task", 10)
        self.create_subscription(String, f"{NS}/status", self.on_status, 10)
        self.status = "waiting for recorder..."

    def on_status(self, msg):
        self.status = msg.data

    def call(self, name):
        c = self.cli[name]
        if not c.wait_for_service(timeout_sec=2.0):
            return f"{NS}/{name} not available (is episode_recorder running?)"
        fut = c.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        r = fut.result()
        return r.message if r else "no response"

    def set_task(self, text):
        self.task_pub.publish(String(data=text))
        return f"task -> '{text}'"


def getkey(timeout=0.2):
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    return sys.stdin.read(1) if r else None


def main():
    rclpy.init()
    node = Ctl()
    print(__doc__)
    old = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        last_status = None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            if node.status != last_status:
                print(f"[{node.status}]")
                last_status = node.status
            k = getkey()
            if k is None:
                continue
            if k == "q":
                break
            msg = None
            if k == "r":
                msg = node.call("start")
            elif k == "s":
                msg = node.call("stop")
            elif k == "d":
                msg = node.call("discard")
            elif k in TASKS:
                msg = node.set_task(TASKS[k])
            elif k == "t":
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
                text = input("task> ").strip()
                tty.setcbreak(sys.stdin.fileno())
                if text:
                    msg = node.set_task(text)
            if msg:
                print(msg)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
