"""Headless smoke test of the pygame window used by takeover_teleop.py."""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame = pytest.importorskip("pygame")
pytest.importorskip("rclpy")          # takeover_teleop imports rclpy at module level

from takeover_logic import HUMAN  # noqa: E402
from takeover_teleop import COLORS, PygameKeys  # noqa: E402


def test_window_polls_and_draws():
    keys = PygameKeys()
    try:
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_SPACE))
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_d))
        pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.K_x))   # unmapped: ignored
        events, held = keys.poll()
        assert events == ["SPACE", "D"] and held == "STOP"
        pygame.event.post(pygame.event.Event(pygame.QUIT))
        assert keys.poll()[0] == ["ESC"]                  # closing the window = ESC
        keys.show("YOU DRIVE  [REC]", ["a", "b", "c", "d", "e"], COLORS[HUMAN])
    finally:
        keys.close()
