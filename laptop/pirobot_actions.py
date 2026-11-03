"""Discrete motion commands shared by the recorder and the inference bridge.

Matches `pi/pirobot/pirobot/motor_control.py` and `motion_subscriber.py`:
- `motor_control.py` drives both motors with PWM `Frequency = 20` Hz and
  `DutyCycleA = 50`, `DutyCycleB = 50` (0.50 normalised duty cycle) for all
  four active movements (`UP`, `DOWN`, `LEFT`, `RIGHT`), and `Stop = 0` (0.0).
- `motion_subscriber.py` listens on `/pirobot/command` (`std_msgs/msg/String`)
  for `'UP'`, `'DOWN'`, `'LEFT'`, `'RIGHT'`, `'STOP'`.
"""
import math

# Normalised PWM duty cycle from motor_control.py (DutyCycleA = DutyCycleB = 50%)
DUTY_CYCLE = 0.50

# name -> (linear_x, angular_z) scaled to the 50% duty cycle in motor_control.py
COMMANDS = {
    "stop": (0.0, 0.0),
    "forward": (DUTY_CYCLE, 0.0),
    "back": (-DUTY_CYCLE, 0.0),
    "left": (0.0, DUTY_CYCLE),
    "right": (0.0, -DUTY_CYCLE),
}

# Mapping between /pirobot/command (motion_subscriber.py / motor_control.py) and COMMANDS
PIROBOT_CMD_TO_NAME = {
    "STOP": "stop",
    "UP": "forward",
    "DOWN": "back",
    "LEFT": "left",
    "RIGHT": "right",
}
NAME_TO_PIROBOT_CMD = {v: k for k, v in PIROBOT_CMD_TO_NAME.items()}


def from_cmd_str(cmd_str: str) -> tuple[float, float]:
    """Convert a /pirobot/command String ('UP', 'DOWN', 'LEFT', 'RIGHT', 'STOP') to (v, w)."""
    name = PIROBOT_CMD_TO_NAME.get(cmd_str.strip().upper(), "stop")
    return COMMANDS[name]


def to_cmd_str(name: str) -> str:
    """Convert a discrete command name ('forward', 'back', 'left', 'right', 'stop') to /pirobot/command String."""
    return NAME_TO_PIROBOT_CMD.get(name, "STOP")


def label(v: float, w: float, eps: float = 1e-3) -> str:
    """Name of the discrete command a recorded Twist corresponds to."""
    if abs(v) < eps and abs(w) < eps:
        return "stop"
    if abs(w) >= eps and abs(v) < eps:
        return "left" if w > 0 else "right"
    if abs(v) >= eps and abs(w) < eps:
        return "forward" if v > 0 else "back"
    # Arcs (e.g. teleop keys like 'u'/'o') get a compound label.
    return ("forward" if v > 0 else "back") + ("_left" if w > 0 else "_right")


def snap(v: float, w: float, stop_radius: float = 0.25) -> tuple[str, float, float]:
    """Nearest discrete command to a continuous (v, w), in normalised units.

    Each axis is scaled by its max magnitude in COMMANDS so linear and
    angular errors are comparable. Outputs whose normalised magnitude is below
    `stop_radius` become "stop", which suppresses jitter around zero.
    """
    v_max = max(abs(c[0]) for c in COMMANDS.values()) or 1.0
    w_max = max(abs(c[1]) for c in COMMANDS.values()) or 1.0
    nv, nw = v / v_max, w / w_max
    if math.hypot(nv, nw) < stop_radius:
        return "stop", 0.0, 0.0
    best = min(
        COMMANDS.items(),
        key=lambda kv: (kv[1][0] / v_max - nv) ** 2 + (kv[1][1] / w_max - nw) ** 2,
    )
    return best[0], best[1][0], best[1][1]
