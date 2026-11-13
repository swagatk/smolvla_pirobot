#!/usr/bin/env python3
"""Single-key takeover for recording corrections (DAgger-style). System ROS2 Jazzy Python.

Replaces keyboard_publisher + episode_ctl during policy runs: one pygame
window drives the robot, toggles the policy and controls the recorder.

    T1 (lerobot venv): python smolvla_server.py --policy <ckpt>/pretrained_model --n-action-steps 5
    T2: python3 smolvla_bridge.py
    T3: python3 episode_recorder.py --ros-args -p out_dir:=$HOME/pirobot_data/raw_round2 -p cmd_timeout:=0.5 \
            -p pirobot_cmd_topic:=/takeover/command -p cmd_topic:=/takeover/unused
    T4: python3 takeover_teleop.py --ros-args -p task:="move around while avoiding obstacles"

The recorder must listen to /takeover/command, which only this tool publishes
(the human commands are also sent on /pirobot/command to drive the robot).
Then a policy command can never be recorded, whatever the timing. The tool
checks this wiring and refuses to record if the recorder subscribes to
/pirobot/command or /cmd_vel.

Keys (the window must have focus):
    SPACE   idle: start the policy | policy: take over | you driving: save, give back to policy
    arrows  policy: take over and steer at once | you driving: drive (nothing held = STOP)
    S       you driving: save and stay stopped | policy: pause the policy
    D       you driving: discard this correction and stay stopped
    ESC     quit (a correction being recorded is saved; the policy is disabled)

Do not run keyboard_publisher at the same time: it publishes STOP continuously
and would fight the policy. The window warns about any other publisher.
"""
import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from takeover_logic import DISCARDING, HUMAN, IDLE, POLICY, SAVING, STARTING, TakeoverLogic

COLORS = {IDLE: (60, 60, 60), POLICY: (20, 60, 140), STARTING: (150, 70, 0),
          HUMAN: (150, 30, 30), SAVING: (110, 90, 0), DISCARDING: (110, 90, 0)}
HELP = "SPACE take over/give back  arrows drive  S save  D discard  ESC quit"
RECORDER_NODE = "episode_recorder"
BRIDGE_NODE = "smolvla_bridge"


class PygameKeys:
    """Keyboard + status window. poll() -> (edge-triggered keys, held arrow or 'STOP')."""

    def __init__(self, size=(660, 240), caption="pirobot takeover"):
        import os
        os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
        import pygame
        self.pg = pygame
        pygame.init()
        self.win = pygame.display.set_mode(size)
        pygame.display.set_caption(caption)
        self.font = pygame.font.Font(None, 26)
        self.big = pygame.font.Font(None, 40)
        self.edge = {pygame.K_SPACE: "SPACE", pygame.K_s: "S", pygame.K_d: "D", pygame.K_ESCAPE: "ESC",
                     pygame.K_UP: "ARROW_UP", pygame.K_DOWN: "ARROW_DOWN",
                     pygame.K_LEFT: "ARROW_LEFT", pygame.K_RIGHT: "ARROW_RIGHT"}

    def poll(self):
        """Return (edge events since the last poll, currently held arrow or 'STOP')."""
        pg, events = self.pg, []
        for e in pg.event.get():
            if e.type == pg.QUIT:
                events.append("ESC")
            elif e.type == pg.KEYDOWN and e.key in self.edge:
                events.append(self.edge[e.key])
        keys = pg.key.get_pressed()
        held = "STOP"
        for name in ("LEFT", "RIGHT", "UP", "DOWN"):        # same priority as key_press.py
            if keys[getattr(pg, "K_" + name)]:
                held = name
                break
        return events, held

    def show(self, title, lines, color):
        """Redraw the window."""
        self.win.fill(color)
        self.win.blit(self.big.render(title, True, (240, 240, 240)), (12, 10))
        for i, text in enumerate(lines):
            self.win.blit(self.font.render(text, True, (230, 230, 230)), (12, 58 + 26 * i))
        self.pg.display.flip()

    def close(self):
        """Close the window."""
        self.pg.quit()


class TakeoverTeleop(Node):
    """ROS side of the takeover tool; all decisions are made by TakeoverLogic."""

    def __init__(self, keys):
        super().__init__("takeover_teleop")
        p = self.declare_parameter
        dyn = ParameterDescriptor(dynamic_typing=True)
        p("cmd_topic", "/pirobot/command")        # drives the robot (shared with the bridge)
        p("record_topic", "/takeover/command")    # human commands only; the recorder listens here
        p("enable_topic", "/smolvla/enable")
        p("bridge_task_topic", "/smolvla/task")
        p("recorder_ns", "/episode_recorder")
        p("rate", 10.0, dyn)                 # Hz; same as the recorder fps / keyboard_publisher
        p("task", "move around while avoiding obstacles")
        p("service_timeout", 3.0, dyn)
        p("start_delay_ticks", 3)            # ticks between takeover and the recorder start call
        p("status_holdoff", 1.5, dyn)        # s after a recorder reply before its status is trusted
        p("check_wiring", True)              # refuse to record if the recorder can see policy topics
        g = lambda n: self.get_parameter(n).value
        self.keys = keys
        self.logic = TakeoverLogic(start_delay_ticks=int(g("start_delay_ticks")))
        self.task = str(g("task"))
        self.cmd_topic = g("cmd_topic")
        self.record_topic = g("record_topic")
        self.service_timeout = float(g("service_timeout"))
        self.status_holdoff = float(g("status_holdoff"))
        self.check_wiring = bool(g("check_wiring"))
        ns = g("recorder_ns")

        self.cmd_pub = self.create_publisher(String, self.cmd_topic, 10)
        self.rec_pub = self.create_publisher(String, self.record_topic, 10)
        self.enable_pub = self.create_publisher(Bool, g("enable_topic"), 10)
        self.bridge_task_pub = self.create_publisher(String, g("bridge_task_topic"), 10)
        self.rec_task_pub = self.create_publisher(String, f"{ns}/task", 10)
        self.cli = {op: self.create_client(Trigger, f"{ns}/{op}") for op in ("start", "stop", "discard")}
        self.create_subscription(String, f"{ns}/status", self.on_status, 10)
        self.create_subscription(String, self.cmd_topic, self.on_robot_cmd, 10)

        self.rec_status, self.rec_status_t = "recorder: not seen yet", None
        self.last_answer_t = -1e9
        self.task_sent = {"bridge": False, "recorder": False}
        self.pending = None                  # (op, future, t0)
        self.last_msg = ""
        self.held = "STOP"
        self.last_sent = None                # last human command we published
        self.takeover_t = None               # when we last took over (for foreign-command detection)
        self.foreign = []                    # times of /pirobot/command messages we did not send
        self.policy_alarm = False
        self.done = False
        self.create_timer(1.0 / float(g("rate")), self.tick)
        self.get_logger().info(f"Ready (IDLE). Task: '{self.task}'. Focus the window; SPACE starts the policy.")

    # ---- status / checks --------------------------------------------
    def on_status(self, msg):
        self.rec_status, self.rec_status_t = msg.data, time.monotonic()

    def status_fresh(self):
        """A status message published after the recorder's last reply (so it reflects that reply)."""
        return (self.pending is None and self.rec_status_t is not None
                and self.rec_status_t > self.last_answer_t + self.status_holdoff
                and time.monotonic() - self.rec_status_t < 3.0)

    def recorder_busy(self):
        return self.status_fresh() and self.rec_status.startswith("REC")

    def bridge_ready(self):
        return self.enable_pub.get_subscription_count() > 0

    def node_names(self, topic, pubs=False):
        infos = self.get_publishers_info_by_topic(topic) if pubs else self.get_subscriptions_info_by_topic(topic)
        return {i.node_name for i in infos}

    def wiring_problem(self):
        """None if the recorder records only our topic, else a human-readable reason."""
        if not self.check_wiring:
            return None
        if RECORDER_NODE not in self.node_names(self.record_topic):
            return (f"recorder not subscribed to {self.record_topic} "
                    f"(run it with -p pirobot_cmd_topic:={self.record_topic})")
        for t in (self.cmd_topic, "/cmd_vel"):
            if RECORDER_NODE in self.node_names(t):
                return f"recorder also listens to {t}, where the policy publishes (set its topic to /takeover/unused)"
        return None

    def other_publishers(self):
        return self.node_names(self.cmd_topic, pubs=True) - {self.get_name(), BRIDGE_NODE}

    def on_robot_cmd(self, msg):
        """Count commands on /pirobot/command that we did not send, while the human drives."""
        if self.logic.state not in (STARTING, HUMAN) or self.takeover_t is None:
            return
        now = time.monotonic()
        if now - self.takeover_t < 0.5 or msg.data == self.last_sent:
            return                                   # grace for the bridge's own disable-STOP
        self.foreign = [t for t in self.foreign if now - t < 1.0] + [now]
        if len(self.foreign) >= 3 and not self.policy_alarm:
            self.policy_alarm = True
            self.enable_pub.publish(Bool(data=False))
            self.apply([("msg", "WARNING: another node is still driving /pirobot/command; re-sent disable")])

    # ---- task -----------------------------------------------------------
    def send_task(self, to_bridge=True, to_recorder=True):
        if to_bridge and self.bridge_task_pub.get_subscription_count() > 0:
            self.bridge_task_pub.publish(String(data=self.task))
            self.task_sent["bridge"] = True
        if to_recorder and self.rec_task_pub.get_subscription_count() > 0 and not self.logic.recording:
            self.rec_task_pub.publish(String(data=self.task))
            self.task_sent["recorder"] = True

    # ---- recorder calls ---------------------------------------------
    def call_recorder(self, op):
        if self.pending is not None:
            self.get_logger().error(f"recorder call '{op}' while '{self.pending[0]}' pending; treating as unanswered")
            self.apply(self.logic.on_rec_result(op, False, False, "another recorder call was pending",
                                                bridge_ready=self.bridge_ready()))
            return
        if op == "start":
            problem = self.wiring_problem()
            if problem:
                self.apply(self.logic.on_rec_result(op, True, False, problem))
                return
        c = self.cli[op]
        if not c.service_is_ready():
            # No call was made, so the recorder cannot have changed state.
            self.apply(self.logic.on_rec_result(op, True, False, "episode_recorder not running",
                                                bridge_ready=self.bridge_ready()))
            return
        self.pending = (op, c.call_async(Trigger.Request()), time.monotonic())

    def check_pending(self):
        if self.pending is None:
            return
        op, fut, t0 = self.pending
        if fut.done():
            self.pending = None
            self.last_answer_t = time.monotonic()
            r = fut.result()
            if r is None:
                self.apply(self.logic.on_rec_result(op, False, False, "no response", bridge_ready=self.bridge_ready()))
            else:
                task_ok = op != "start" or f": '{self.task}'" in r.message
                self.apply(self.logic.on_rec_result(op, True, bool(r.success), r.message,
                                                    task_ok=task_ok, bridge_ready=self.bridge_ready()))
        elif time.monotonic() - t0 > self.service_timeout:
            self.pending = None
            self.last_answer_t = time.monotonic()
            fut.cancel()                             # a late reply is ignored; the logic assumes "maybe"
            self.apply(self.logic.on_rec_result(op, False, False, f"timeout after {self.service_timeout:.0f}s",
                                                bridge_ready=self.bridge_ready()))

    # ---- effects --------------------------------------------------------
    def apply(self, effects):
        for eff in effects:
            kind = eff[0]
            if kind == "enable":
                if eff[1]:
                    self.send_task(to_recorder=False)      # bridge runs the task shown in the window
                else:
                    self.send_task(to_bridge=False)        # recorder gets the task well before any start
                    self.takeover_t, self.foreign, self.policy_alarm = time.monotonic(), [], False
                self.enable_pub.publish(Bool(data=bool(eff[1])))
                self.get_logger().info("policy ENABLED" if eff[1] else "policy disabled")
            elif kind == "cmd":
                self.last_sent = eff[1]
                self.cmd_pub.publish(String(data=eff[1]))
                if self.logic.state in (STARTING, HUMAN):
                    self.rec_pub.publish(String(data=eff[1]))
            elif kind == "rec":
                self.call_recorder(eff[1])
            elif kind == "msg":
                self.last_msg = eff[1]
                self.get_logger().info(eff[1])
            elif kind == "quit":
                self.done = True

    # ---- main loop ------------------------------------------------------
    def tick(self):
        if not all(self.task_sent.values()):
            self.send_task(to_bridge=not self.task_sent["bridge"], to_recorder=not self.task_sent["recorder"])
        self.check_pending()
        if self.status_fresh() and self.rec_status.startswith("idle"):
            self.logic.on_recorder_idle()
        events, self.held = self.keys.poll()
        for k in events:
            self.apply(self.logic.on_key(k, bridge_ready=self.bridge_ready(), recorder_busy=self.recorder_busy()))
        self.apply(self.logic.on_tick(self.held))
        self.draw()

    def draw(self):
        lg = self.logic
        title = {IDLE: "IDLE (robot stopped)", POLICY: "POLICY DRIVING",
                 STARTING: "YOU DRIVE (starting rec...)",
                 HUMAN: "YOU DRIVE  [REC]" if lg.recording else "YOU DRIVE  (NOT RECORDING)",
                 SAVING: "saving...", DISCARDING: "discarding..."}[lg.state]
        lines = [f"task: {self.task}",
                 f"key: {self.held if lg.state in (STARTING, HUMAN) else '-'}   "
                 f"corrections saved: {lg.saved}  discarded: {lg.discarded}",
                 self.rec_status[:72],
                 self.last_msg[:72]]
        others = self.other_publishers()
        if self.policy_alarm:
            lines.append("WARNING: policy still publishing after takeover!")
        elif others:
            lines.append(f"WARNING: also publishing {self.cmd_topic}: {', '.join(sorted(others))}")
        elif lg.recorder_unsure:
            lines.append("recorder state unknown: policy blocked (SPACE in IDLE sends stop)")
        elif not self.bridge_ready():
            lines.append("smolvla_bridge not seen on /smolvla/enable")
        else:
            lines.append(HELP)
        self.keys.show(title, lines, COLORS[lg.state])

    # ---- shutdown -------------------------------------------------------
    def shutdown_safely(self, stop_timeout=3.0):
        """Disable the policy, stop the robot, and close any recording that may be open (best effort)."""
        for _ in range(3):
            self.enable_pub.publish(Bool(data=False))
            self.cmd_pub.publish(String(data="STOP"))
            time.sleep(0.05)
        maybe_open = (self.logic.recording or self.logic.recorder_unsure or self.pending is not None
                      or self.rec_status.startswith("REC"))
        c = self.cli["stop"]
        if maybe_open and c.service_is_ready():
            fut = c.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=stop_timeout)
            r = fut.result() if fut.done() else None
            self.get_logger().info(f"recorder stop on exit: {r.message if r else 'no answer'}")


def run(node, max_quit_wait_s=10.0):
    """Spin until the logic says quit. Ctrl-C behaves like ESC (saves a correction in progress)."""
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        if rclpy.ok():
            try:
                node.apply(node.logic.on_key("ESC"))
                t0 = time.monotonic()
                while rclpy.ok() and not node.done and time.monotonic() - t0 < max_quit_wait_s:
                    node.check_pending()
                    rclpy.spin_once(node, timeout_sec=0.05)
            except KeyboardInterrupt:
                pass                                   # second Ctrl-C: go straight to shutdown
    finally:
        if rclpy.ok():
            node.shutdown_safely()
            node.get_logger().info("Policy disabled, STOP sent, exiting")


def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    keys = PygameKeys()
    node = TakeoverTeleop(keys)
    try:
        run(node)
    finally:
        keys.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
