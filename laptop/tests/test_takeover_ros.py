"""ROS integration tests for takeover_teleop.py (need ROS 2 Jazzy sourced).

    source /opt/ros/jazzy/setup.bash && cd laptop && python3 -m pytest -q tests/

The real EpisodeRecorder and SmolVLABridge classes run in this process,
together with a fake camera, a fake policy and the takeover node driven by
scripted keys. Everything is in one process on purpose: cross-process DDS
delivery on the WSL dev laptop is intermittent (discovery succeeds but data is
not delivered, depending on domain ID and leftover state), which would make
the test flaky for reasons unrelated to this code.

Safety: conftest.py puts the tests on their own ROS domain with localhost-only
discovery, so nothing here can reach the robot (domain 19).
"""
import json
import os
import sys
import time
from pathlib import Path

import pytest

LAPTOP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(LAPTOP))

rclpy = pytest.importorskip("rclpy")
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import CompressedImage  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

from takeover_logic import IDLE, POLICY  # noqa: E402
from takeover_teleop import TakeoverTeleop  # noqa: E402

TASK = "move around while avoiding obstacles"
pytestmark = pytest.mark.slow
GOOD_WIRING = "    pirobot_cmd_topic: '/takeover/command'\n    cmd_topic: '/takeover/unused'\n"


def test_isolated_from_robot_domain():
    assert os.environ["ROS_DOMAIN_ID"] != "19"
    assert os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] == "LOCALHOST"


class ScriptedKeys:
    """Stands in for PygameKeys: a list of (n_ticks, held, edge_events_on_first_tick)."""

    def __init__(self):
        self.script, self.i, self.left = [], -1, 0

    def run(self, script):
        self.script, self.i, self.left = list(script), -1, 0

    def busy(self):
        return self.left > 0 or self.i + 1 < len(self.script)

    def poll(self):
        if self.left == 0:
            if self.i + 1 >= len(self.script):
                return [], "STOP"
            self.i += 1
            n, held, events = self.script[self.i]
            self.left = n - 1
            return list(events), held
        self.left -= 1
        return [], self.script[self.i][1]

    def show(self, title, lines, color):
        pass


class FakeCamera(Node):
    def __init__(self):
        super().__init__("fake_cam")
        self.pub = self.create_publisher(CompressedImage, "/camera/image/compressed", 10)
        self.create_timer(1 / 15, self.tick)

    def tick(self):
        m = CompressedImage()
        m.format = "jpeg"
        m.data = b"\xff\xd8fakejpeg\xff\xd9"
        self.pub.publish(m)


class FakePolicy(Node):
    """Worse than smolvla_bridge: while enabled it publishes RIGHT on /pirobot/command and a
    Twist on /cmd_vel at 10 Hz; after disable it keeps going for `late_s` (an in-flight
    inference, exaggerated) and then sends STOP. `ignore_disable` never stops at all.
    Any of these commands leaking into a correction shows up as 'right'."""

    def __init__(self, late_s=0.4, ignore_disable=False):
        super().__init__("smolvla_bridge")       # same node name as the real bridge
        self.late_s, self.ignore_disable = late_s, ignore_disable
        self.enabled, self.disabled_at, self.enables, self.tasks = False, None, [], []
        self.pub = self.create_publisher(String, "/pirobot/command", 10)
        self.twist = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Bool, "/smolvla/enable", self.on_enable, 10)
        self.create_subscription(String, "/smolvla/task", lambda m: self.tasks.append(m.data), 10)
        # ~14 Hz, deliberately not 10 Hz: with every node on one executor, equal periods
        # phase-lock the timers and the human command always lands last, hiding leaks.
        self.create_timer(0.07, self.tick)

    def on_enable(self, m):
        self.enables.append(m.data)
        if m.data:
            self.enabled, self.disabled_at = True, None
        elif self.enabled and not self.ignore_disable:
            self.enabled, self.disabled_at = False, time.monotonic()

    def tick(self):
        late = self.disabled_at is not None and time.monotonic() - self.disabled_at < self.late_s
        if self.enabled or late:
            self.pub.publish(String(data="RIGHT"))
            t = Twist()
            t.angular.z = -0.5
            self.twist.publish(t)
        elif self.disabled_at is not None:
            self.pub.publish(String(data="STOP"))
            self.disabled_at = None


def spin_until(ex, cond, timeout):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        ex.spin_once(timeout_sec=0.02)
        if cond():
            return True
    return False


@pytest.fixture
def ros_ctx(tmp_path):
    """Factory: init rclpy with a params file for the real recorder and bridge."""
    started = []

    def init(wiring=GOOD_WIRING):
        out = tmp_path / "raw"
        params = tmp_path / "params.yaml"
        params.write_text(
            "episode_recorder:\n  ros__parameters:\n"
            f"    out_dir: '{out}'\n    cmd_timeout: 0.5\n    task: 'some other task'\n{wiring}"
            "smolvla_bridge:\n  ros__parameters:\n    server_port: 6999\n")
        rclpy.init(args=["--ros-args", "--params-file", str(params)])
        started.append(True)
        return out
    yield init
    if started:
        rclpy.try_shutdown()


def make(*nodes):
    ex = SingleThreadedExecutor()
    for n in nodes:
        ex.add_node(n)
    return ex


def wait_ready(ex, tool, need_recorder=True):
    def ready():
        ok = tool.bridge_ready() and tool.task_sent["bridge"]
        if need_recorder:
            ok = ok and all(c.service_is_ready() for c in tool.cli.values()) and tool.task_sent["recorder"] \
                and tool.rec_status.startswith("idle")
        return ok
    assert spin_until(ex, ready, 20), f"setup not ready: {tool.rec_status!r} {tool.task_sent}"


def episodes(out):
    return sorted(d for d in out.iterdir() if d.name.startswith("ep_")) if out.exists() else []


def read_episode(ep):
    meta = json.loads((ep / "meta.json").read_text())
    steps = [json.loads(line) for line in (ep / "steps.jsonl").read_text().splitlines()]
    return meta, steps


def destroy(*nodes):
    for n in nodes:
        n.destroy_node()


TAKEOVER_SCRIPT = [(10, "STOP", ["SPACE"]),     # start policy: fake policy drives RIGHT for 1 s
                   (20, "UP", []),              # arrow = takeover, drive forward 2 s
                   (10, "LEFT", []),            # turn 1 s
                   (5, "STOP", []),
                   (1, "STOP", ["SPACE"]),      # save + give back to policy
                   (15, "STOP", [])]            # policy drives RIGHT again


@pytest.mark.parametrize("takeover_key", ["arrow", "space"])
def test_takeover_records_only_human_commands(ros_ctx, takeover_key):
    from episode_recorder import EpisodeRecorder
    out = ros_ctx()
    keys = ScriptedKeys()
    rec, cam, policy = EpisodeRecorder(), FakeCamera(), FakePolicy(late_s=0.4)
    tool = TakeoverTeleop(keys)
    ex = make(rec, cam, policy, tool)
    try:
        wait_ready(ex, tool)
        assert spin_until(ex, lambda: rec.task == TASK, 5)
        script = list(TAKEOVER_SCRIPT)
        if takeover_key == "space":
            script[1:2] = [(1, "STOP", ["SPACE"]), (19, "UP", [])]
        keys.run(script)
        assert spin_until(ex, lambda: not keys.busy(), 20)
        assert spin_until(ex, lambda: tool.logic.state == POLICY, 5), tool.last_msg
        assert tool.logic.saved == 1 and policy.enabled and not tool.policy_alarm

        keys.run([(1, "STOP", ["ESC"])])
        assert spin_until(ex, lambda: tool.done, 10)
        tool.shutdown_safely(stop_timeout=1.0)
        spin_until(ex, lambda: False, 0.3)
    finally:
        destroy(tool, policy, cam, rec)

    assert policy.enables[:3] == [True, False, True] and policy.enables[-1] is False, policy.enables
    assert not policy.enabled and policy.tasks[-1] == TASK and set(policy.tasks) == {TASK}

    eps = episodes(out)
    assert len(eps) == 1, [d.name for d in out.iterdir()]
    meta, steps = read_episode(eps[0])
    cmds = [s["cmd"] for s in steps]
    assert meta["task"] == TASK
    assert "right" not in cmds, cmds                  # policy commands (late or not) never recorded
    assert cmds.count("forward") >= 12 and cmds.count("left") >= 6, cmds
    assert cmds[0] in ("forward", "stop"), cmds
    assert len(steps) == meta["num_frames"] and all((eps[0] / s["img"]).is_file() for s in steps)


def test_policy_ignoring_disable_is_flagged_and_not_recorded(ros_ctx):
    from episode_recorder import EpisodeRecorder
    out = ros_ctx()
    keys = ScriptedKeys()
    rec, cam, policy = EpisodeRecorder(), FakeCamera(), FakePolicy(ignore_disable=True)
    tool = TakeoverTeleop(keys)
    ex = make(rec, cam, policy, tool)
    try:
        wait_ready(ex, tool)
        keys.run([(5, "STOP", ["SPACE"]), (25, "UP", []), (1, "STOP", ["S"]), (5, "STOP", [])])
        assert spin_until(ex, lambda: not keys.busy(), 20)
        assert spin_until(ex, lambda: tool.logic.state == IDLE, 5)
        assert tool.policy_alarm
        assert policy.enables.count(False) >= 2          # disable was re-sent
    finally:
        destroy(tool, policy, cam, rec)
    eps = episodes(out)
    assert len(eps) == 1
    assert "right" not in [s["cmd"] for s in read_episode(eps[0])[1]]


def test_recorder_on_policy_topics_is_refused(ros_ctx):
    from episode_recorder import EpisodeRecorder
    out = ros_ctx(wiring="")                     # recorder defaults: /pirobot/command and /cmd_vel
    keys = ScriptedKeys()
    rec, cam, policy = EpisodeRecorder(), FakeCamera(), FakePolicy()
    tool = TakeoverTeleop(keys)
    ex = make(rec, cam, policy, tool)
    try:
        wait_ready(ex, tool)
        keys.run([(5, "STOP", ["SPACE"]), (15, "UP", [])])
        assert spin_until(ex, lambda: not keys.busy(), 20)
        assert not tool.logic.recording and "NOT RECORDING" in tool.last_msg and "recorder" in tool.last_msg
        keys.run([(1, "STOP", ["SPACE"]), (3, "STOP", [])])          # hand back: nothing to save
        assert spin_until(ex, lambda: not keys.busy(), 5)
        assert tool.logic.state == POLICY and tool.logic.saved == 0
    finally:
        destroy(tool, policy, cam, rec)
    assert episodes(out) == []


def test_discard_and_missing_recorder(ros_ctx):
    from episode_recorder import EpisodeRecorder
    out = ros_ctx()
    keys = ScriptedKeys()
    rec, cam, policy = EpisodeRecorder(), FakeCamera(), FakePolicy(late_s=0.0)
    tool = TakeoverTeleop(keys)
    ex = make(rec, cam, policy, tool)
    try:
        wait_ready(ex, tool)
        keys.run([(5, "STOP", ["SPACE"]), (25, "UP", []), (1, "STOP", ["D"]), (5, "STOP", [])])
        assert spin_until(ex, lambda: not keys.busy(), 20)
        assert spin_until(ex, lambda: tool.logic.state == IDLE, 5)
        assert tool.logic.discarded == 1 and not policy.enabled and episodes(out) == []

        ex.remove_node(rec)
        rec.destroy_node()                       # recorder gone: still drivable, nothing saved
        assert spin_until(ex, lambda: not tool.cli["start"].service_is_ready(), 10)
        keys.run([(1, "STOP", ["SPACE"]), (5, "STOP", []), (10, "UP", []), (1, "STOP", ["SPACE"]),
                  (3, "STOP", [])])
        assert spin_until(ex, lambda: not keys.busy(), 20)
        assert tool.logic.state == POLICY and policy.enabled and tool.logic.saved == 0
        assert not tool.logic.recorder_unsure
        assert "nothing was recorded" in tool.last_msg, tool.last_msg
    finally:
        destroy(tool, policy, cam)


def test_refuses_policy_without_bridge(ros_ctx):
    ros_ctx()
    keys = ScriptedKeys()
    tool = TakeoverTeleop(keys)
    ex = make(tool)
    try:
        keys.run([(3, "STOP", ["SPACE"])])
        assert spin_until(ex, lambda: not keys.busy(), 5)
        assert tool.logic.state == IDLE and "not listening" in tool.last_msg
    finally:
        tool.destroy_node()


def test_real_bridge_accepts_enable_and_task(ros_ctx):
    """Interface check against the real SmolVLABridge (no policy server running)."""
    from smolvla_bridge import SmolVLABridge
    ros_ctx()
    keys = ScriptedKeys()
    bridge = SmolVLABridge()
    tool = TakeoverTeleop(keys)
    ex = make(bridge, tool)
    try:
        wait_ready(ex, tool, need_recorder=False)
        assert spin_until(ex, lambda: bridge.task == TASK, 5)
        keys.run([(5, "STOP", ["SPACE"])])
        assert spin_until(ex, lambda: not keys.busy(), 5)
        assert spin_until(ex, lambda: bridge.enabled, 5)
        keys.run([(5, "STOP", ["SPACE"])])          # take over -> bridge disabled
        assert spin_until(ex, lambda: not keys.busy(), 5)
        assert spin_until(ex, lambda: not bridge.enabled, 5)
    finally:
        destroy(tool, bridge)
