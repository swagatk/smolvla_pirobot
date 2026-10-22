"""Discrete motion commands shared by the recorder and the inference bridge.

The dataset stores the continuous Twist the teleop sent, (linear.x, angular.z),
as the 2-D action. These helpers label it with one of the five discrete
commands and, at inference time, snap the policy's continuous output back to
the nearest discrete command.

Edit COMMANDS so the speeds match what your keyboard teleop actually sends.
"""
import math

# name -> (linear_x m/s, angular_z rad/s)
COMMANDS = {
    "stop": (0.0, 0.0),
    "forward": (0.20, 0.0),
    "back": (-0.15, 0.0),
    "left": (0.0, 1.0),
    "right": (0.0, -1.0),
}


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
