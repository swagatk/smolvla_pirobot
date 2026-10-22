#!/usr/bin/env python3
"""Record teleop episodes (camera + cmd_vel + task text) on the laptop.

Runs with the system ROS2 Jazzy Python (no torch / lerobot needed). It writes
a simple raw format that `convert_to_lerobot.py` turns into a LeRobot dataset:

    <out_dir>/<episode_name>/
        meta.json        task, fps, frame count, dropped ticks, topics
        steps.jsonl      one line per frame: i, t, v, w, prev_v, prev_w, cmd, img
        frames/000000.jpg ...   the JPEG bytes exactly as received from the Pi

Sampling is driven by a fixed-rate timer (default 10 Hz). Each tick pairs the
newest camera frame with the command in force at that moment, so action[t] is
"what the operator was commanding when frame t was seen".

Usage:
    python3 episode_recorder.py --ros-args \
        -p out_dir:=$HOME/pirobot_data/raw -p task:="follow the wall"

Control it from another terminal with episode_ctl.py, or directly:
    ros2 service call /episode_recorder/start   std_srvs/srv/Trigger
    ros2 service call /episode_recorder/stop    std_srvs/srv/Trigger   # save
    ros2 service call /episode_recorder/discard std_srvs/srv/Trigger
    ros2 topic pub --once /episode_recorder/task std_msgs/msg/String "{data: 'follow the ball'}"
"""
import json
import os
import shutil
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pirobot_actions import label


class EpisodeRecorder(Node):
    def __init__(self):
        super().__init__("episode_recorder")
        p = self.declare_parameter
        p("image_topic", "/camera/image/compressed")
        p("cmd_topic", "/cmd_vel")
        p("cmd_stamped", False)       # True if your teleop publishes TwistStamped
        p("fps", 10.0)
        p("out_dir", os.path.expanduser("~/pirobot_data/raw"))
        p("task", "move around while avoiding obstacles")
        # 0 = hold the last command until a new one arrives (latching teleop).
        # >0 = treat the command as "stop" if nothing arrived for this long,
        #      which matches drivers that stop when cmd_vel goes quiet.
        p("cmd_timeout", 0.0)
        p("image_timeout", 0.3)       # skip a tick if the newest frame is older
        p("min_frames", 20)           # refuse to save shorter episodes

        g = lambda n: self.get_parameter(n).value
        self.fps = float(g("fps"))
        self.out_dir = g("out_dir")
        self.task = g("task")
        self.cmd_timeout = float(g("cmd_timeout"))
        self.image_timeout = float(g("image_timeout"))
        self.min_frames = int(g("min_frames"))
        os.makedirs(self.out_dir, exist_ok=True)

        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(CompressedImage, g("image_topic"), self.on_image, img_qos)
        if g("cmd_stamped"):
            self.create_subscription(TwistStamped, g("cmd_topic"), lambda m: self.on_cmd(m.twist), 10)
        else:
            self.create_subscription(Twist, g("cmd_topic"), self.on_cmd, 10)
        self.create_subscription(String, "~/task", self.on_task, 10)
        self.create_service(Trigger, "~/start", self.srv_start)
        self.create_service(Trigger, "~/stop", self.srv_stop)
        self.create_service(Trigger, "~/discard", self.srv_discard)
        self.status_pub = self.create_publisher(String, "~/status", 10)

        self.last_img = None          # (recv_time, bytes, format)
        self.last_cmd = (0.0, 0.0)
        self.last_cmd_time = None
        self.prev_cmd = (0.0, 0.0)
        self.ep = None                # dict while recording

        self.create_timer(1.0 / self.fps, self.tick)
        self.create_timer(1.0, self.publish_status)
        self.get_logger().info(f"Ready. Task: '{self.task}'. Output: {self.out_dir}")

    # ---- inputs -------------------------------------------------------
    def on_image(self, msg: CompressedImage):
        # Receive time on the laptop clock: avoids Pi/laptop clock skew.
        self.last_img = (time.monotonic(), bytes(msg.data), msg.format)

    def on_cmd(self, tw: Twist):
        self.last_cmd = (float(tw.linear.x), float(tw.angular.z))
        self.last_cmd_time = time.monotonic()

    def on_task(self, msg: String):
        if self.ep is not None:
            self.get_logger().warn("Task changed mid-episode; it applies from the next episode.")
        self.task = msg.data.strip()
        self.get_logger().info(f"Task set to: '{self.task}'")

    # ---- recording ----------------------------------------------------
    def tick(self):
        if self.ep is None:
            return
        now = time.monotonic()
        if self.last_img is None or now - self.last_img[0] > self.image_timeout:
            self.ep["dropped"] += 1
            return
        if self.cmd_timeout > 0 and (
            self.last_cmd_time is None or now - self.last_cmd_time > self.cmd_timeout
        ):
            v, w = 0.0, 0.0
        else:
            v, w = self.last_cmd

        i = self.ep["n"]
        ext = "png" if "png" in self.last_img[2].lower() else "jpg"
        fname = f"frames/{i:06d}.{ext}"
        with open(os.path.join(self.ep["dir"], fname), "wb") as f:
            f.write(self.last_img[1])
        step = {
            "i": i,
            "t": round(now - self.ep["t0"], 4),
            "v": v,
            "w": w,
            "prev_v": self.prev_cmd[0],
            "prev_w": self.prev_cmd[1],
            "cmd": label(v, w),
            "img": fname,
        }
        self.ep["steps"].write(json.dumps(step) + "\n")
        self.ep["n"] += 1
        self.prev_cmd = (v, w)

    def srv_start(self, req, res):
        if self.ep is not None:
            res.success, res.message = False, "Already recording"
            return res
        name = datetime.now().strftime("ep_%Y%m%d_%H%M%S")
        d = os.path.join(self.out_dir, name + ".partial")
        os.makedirs(os.path.join(d, "frames"))
        self.ep = {
            "name": name,
            "dir": d,
            "task": self.task,
            "t0": time.monotonic(),
            "n": 0,
            "dropped": 0,
            "steps": open(os.path.join(d, "steps.jsonl"), "w"),
        }
        self.prev_cmd = (0.0, 0.0)
        res.success, res.message = True, f"Recording {name}: '{self.task}'"
        self.get_logger().info(res.message)
        return res

    def srv_stop(self, req, res):
        if self.ep is None:
            res.success, res.message = False, "Not recording"
            return res
        ep, self.ep = self.ep, None
        ep["steps"].close()
        if ep["n"] < self.min_frames:
            shutil.rmtree(ep["dir"], ignore_errors=True)
            res.success, res.message = False, f"Only {ep['n']} frames; episode discarded"
            self.get_logger().warn(res.message)
            return res
        meta = {
            "task": ep["task"],
            "fps": self.fps,
            "num_frames": ep["n"],
            "dropped_ticks": ep["dropped"],
            "duration_s": round(time.monotonic() - ep["t0"], 2),
            "image_topic": self.get_parameter("image_topic").value,
            "cmd_topic": self.get_parameter("cmd_topic").value,
            "cmd_timeout": self.cmd_timeout,
        }
        with open(os.path.join(ep["dir"], "meta.json"), "w") as f:
            json.dump(meta, f, indent=2)
        final = os.path.join(self.out_dir, ep["name"])
        os.rename(ep["dir"], final)
        drop_pct = 100.0 * ep["dropped"] / max(1, ep["n"] + ep["dropped"])
        res.success = True
        res.message = f"Saved {ep['name']}: {ep['n']} frames, {drop_pct:.1f}% ticks dropped"
        self.get_logger().info(res.message)
        if drop_pct > 10:
            self.get_logger().warn("Many dropped ticks: check WiFi or lower the camera fps/resolution.")
        return res

    def srv_discard(self, req, res):
        if self.ep is None:
            res.success, res.message = False, "Not recording"
            return res
        ep, self.ep = self.ep, None
        ep["steps"].close()
        shutil.rmtree(ep["dir"], ignore_errors=True)
        res.success, res.message = True, f"Discarded {ep['name']}"
        self.get_logger().info(res.message)
        return res

    def publish_status(self):
        n_saved = sum(
            1 for d in os.listdir(self.out_dir)
            if d.startswith("ep_") and not d.endswith(".partial")
        )
        if self.ep is None:
            s = f"idle | task='{self.task}' | saved episodes={n_saved}"
        else:
            s = f"REC {self.ep['name']} | {self.ep['n']} frames | task='{self.ep['task']}'"
        self.status_pub.publish(String(data=s))

    def destroy_node(self):
        if self.ep is not None:
            self.ep["steps"].close()
            self.get_logger().warn(f"Shutdown while recording; partial episode left at {self.ep['dir']}")
        super().destroy_node()


def main():
    rclpy.init()
    node = EpisodeRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
