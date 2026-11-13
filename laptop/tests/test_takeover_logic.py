"""Unit tests for takeover_logic.TakeoverLogic (no ROS, no pygame)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from takeover_logic import (DISCARDING, HUMAN, IDLE, POLICY, SAVING, STARTING,  # noqa: E402
                            TakeoverLogic)


def kinds(effects, kind):
    return [e for e in effects if e[0] == kind]


def no_msgs(effects):
    return [e for e in effects if e[0] != "msg"]


def node_tick(lg, keys=(), held="STOP", **kw):
    """Replay what TakeoverTeleop.tick does in one tick: key events first, then on_tick."""
    out = []
    for k in keys:
        out += lg.on_key(k, **kw)
    return out + lg.on_tick(held)


def to_policy(lg):
    eff = node_tick(lg, ["SPACE"])
    assert lg.state == POLICY and ("enable", True) in eff
    return eff


def run_until_start(lg, held="UP", max_ticks=10):
    """Tick until the start call goes out; return the per-tick effects."""
    ticks = []
    for _ in range(max_ticks):
        eff = node_tick(lg, held=held)
        ticks.append(eff)
        if ("rec", "start") in eff:
            return ticks
    raise AssertionError("start never requested")


def to_recording(lg, first="UP"):
    """POLICY -> takeover by arrow -> recorder start accepted."""
    eff = node_tick(lg, held=first)
    assert no_msgs(eff) == [("enable", False), ("cmd", first)]   # disable comes first
    assert lg.state == STARTING
    ticks = run_until_start(lg, held=first)
    assert len(ticks) >= 2                                       # >= 2 ticks of our cmd before start
    assert all(no_msgs(t) == [("cmd", first)] for t in ticks[:-1])
    lg.on_rec_result("start", True, True, "Recording ep_x: 'task'")
    assert lg.state == HUMAN and lg.recording


def test_starts_idle_and_publishes_nothing():
    lg = TakeoverLogic()
    assert lg.state == IDLE
    assert lg.on_tick("UP") == [] and lg.on_tick("STOP") == []


def test_policy_not_started_without_bridge_or_while_recorder_busy():
    lg = TakeoverLogic()
    eff = lg.on_key("SPACE", bridge_ready=False)
    assert lg.state == IDLE and not kinds(eff, "enable")
    eff = lg.on_key("SPACE", recorder_busy=True)
    assert lg.state == IDLE and not kinds(eff, "enable")


def test_policy_mode_is_silent_until_an_arrow():
    lg = TakeoverLogic()
    to_policy(lg)
    for _ in range(5):
        assert lg.on_tick("STOP") == []


def test_arrow_takeover_record_and_hand_back():
    lg = TakeoverLogic()
    to_policy(lg)
    to_recording(lg, "LEFT")
    assert lg.on_tick("UP") == [("cmd", "UP")]
    eff = lg.on_key("SPACE")
    assert lg.state == SAVING and no_msgs(eff) == [("cmd", "STOP"), ("rec", "stop")]
    assert not kinds(eff, "enable")
    assert lg.on_tick("UP") == []
    eff = lg.on_rec_result("stop", True, True, "Saved ep_x")
    assert lg.state == POLICY and ("enable", True) in eff and lg.saved == 1


def test_space_takeover_in_node_order_delays_start():
    """Finding: key + tick run in the same node tick; start must still wait >= 2 later ticks."""
    lg = TakeoverLogic()
    to_policy(lg)
    eff = node_tick(lg, ["SPACE"])
    assert lg.state == STARTING and not kinds(eff, "rec")
    assert no_msgs(eff)[:2] == [("enable", False), ("cmd", "STOP")]
    ticks = run_until_start(lg, held="STOP")
    assert len(ticks) >= 2 and not any(kinds(t, "rec") for t in ticks[:-1])


def test_arrow_tap_takes_over():
    lg = TakeoverLogic()
    to_policy(lg)
    eff = node_tick(lg, ["ARROW_DOWN"], held="STOP")      # released before the poll
    assert lg.state == STARTING and no_msgs(eff)[:2] == [("enable", False), ("cmd", "DOWN")]
    assert node_tick(TakeoverLogic(), ["ARROW_UP"]) == []  # ignored when idle


def test_s_saves_and_stays_idle():
    lg = TakeoverLogic()
    to_policy(lg)
    to_recording(lg)
    lg.on_key("S")
    eff = lg.on_rec_result("stop", True, True, "Saved")
    assert lg.state == IDLE and not kinds(eff, "enable") and lg.saved == 1


def test_s_in_policy_pauses():
    lg = TakeoverLogic()
    to_policy(lg)
    eff = lg.on_key("S")
    assert lg.state == IDLE and ("enable", False) in eff and ("cmd", "STOP") in eff


def test_discard_goes_idle():
    lg = TakeoverLogic()
    to_policy(lg)
    to_recording(lg)
    eff = lg.on_key("D")
    assert lg.state == DISCARDING and ("rec", "discard") in eff
    eff = lg.on_rec_result("discard", True, True, "Discarded")
    assert lg.state == IDLE and not kinds(eff, "enable") and lg.discarded == 1 and lg.saved == 0


def test_keys_ignored_while_start_pending():
    lg = TakeoverLogic()
    to_policy(lg)
    node_tick(lg, held="UP")
    run_until_start(lg)
    for k in ("SPACE", "S", "D"):
        eff = lg.on_key(k)
        assert not kinds(eff, "enable") and not kinds(eff, "rec") and lg.state == STARTING
    lg.on_rec_result("start", True, True, "Recording")
    assert ("rec", "stop") in lg.on_key("SPACE")


def test_space_before_start_requested_hands_back_without_recording():
    lg = TakeoverLogic()
    to_policy(lg)
    node_tick(lg, ["SPACE"])                 # take over
    eff = node_tick(lg, ["SPACE"])           # give back before the start call went out
    assert lg.state == POLICY and ("enable", True) in eff and not kinds(eff, "rec")


def test_refused_start_is_safe_to_hand_back():
    """A refusal the node makes itself (recorder not running / wrong wiring) means not recording."""
    lg = TakeoverLogic()
    to_policy(lg)
    node_tick(lg, held="UP")
    run_until_start(lg)
    eff = lg.on_rec_result("start", True, False, "episode_recorder not running")
    assert lg.state == HUMAN and not lg.recording and not lg.recorder_unsure
    assert any("NOT RECORDING" in m[1] for m in kinds(eff, "msg"))
    assert lg.on_tick("RIGHT") == [("cmd", "RIGHT")]
    eff = lg.on_key("SPACE")
    assert not kinds(eff, "rec") and ("enable", True) in eff and lg.state == POLICY


def test_unanswered_start_never_hands_back_without_stop():
    """Finding 1: a start that timed out may have started recording."""
    lg = TakeoverLogic()
    to_policy(lg)
    node_tick(lg, held="UP")
    run_until_start(lg)
    lg.on_rec_result("start", False, False, "timeout after 3s")
    assert lg.recorder_unsure and lg.state == HUMAN
    eff = lg.on_key("SPACE")
    assert not kinds(eff, "enable") and ("rec", "stop") in eff and lg.state == SAVING
    eff = lg.on_rec_result("stop", True, True, "Saved ep_y")       # it had started after all
    assert lg.state == POLICY and ("enable", True) in eff and not lg.recorder_unsure
    assert any("did not start" in m[1] for m in kinds(eff, "msg"))


def test_already_recording_reply_is_unsure():
    lg = TakeoverLogic()
    to_policy(lg)
    node_tick(lg, held="UP")
    run_until_start(lg)
    lg.on_rec_result("start", True, False, "Already recording")
    assert lg.recorder_unsure
    eff = lg.on_key("SPACE")
    assert not kinds(eff, "enable") and ("rec", "stop") in eff


def test_unanswered_stop_blocks_policy_until_resolved():
    """Finding 2: after an unanswered stop, SPACE in IDLE must resolve, not enable."""
    lg = TakeoverLogic()
    to_policy(lg)
    to_recording(lg)
    lg.on_key("SPACE")
    eff = lg.on_rec_result("stop", False, False, "")
    assert lg.state == IDLE and not kinds(eff, "enable") and lg.recorder_unsure
    eff = lg.on_key("SPACE")
    assert not kinds(eff, "enable") and ("rec", "stop") in eff and lg.state == SAVING
    eff = lg.on_rec_result("stop", True, False, "Not recording")
    assert lg.state == IDLE and not kinds(eff, "enable") and not lg.recorder_unsure
    assert ("enable", True) in lg.on_key("SPACE")                  # now it may start


def test_unanswered_stop_retry_unanswered_again_stays_blocked():
    lg = TakeoverLogic()
    to_policy(lg)
    to_recording(lg)
    lg.on_key("SPACE")
    lg.on_rec_result("stop", False, False, "")
    lg.on_key("SPACE")
    eff = lg.on_rec_result("stop", False, False, "")
    assert lg.state == IDLE and lg.recorder_unsure and not kinds(eff, "enable")


def test_fresh_idle_status_clears_unsure():
    lg = TakeoverLogic()
    to_policy(lg)
    to_recording(lg)
    lg.on_key("SPACE")
    lg.on_rec_result("stop", False, False, "")
    lg.on_recorder_idle()
    assert not lg.recorder_unsure and ("enable", True) in lg.on_key("SPACE")


def test_wrong_task_is_discarded():
    lg = TakeoverLogic()
    to_policy(lg)
    node_tick(lg, held="UP")
    run_until_start(lg)
    eff = lg.on_rec_result("start", True, True, "Recording ep_z: 'other'", task_ok=False)
    assert lg.state == DISCARDING and ("rec", "discard") in eff
    eff = lg.on_rec_result("discard", True, True, "Discarded")
    assert lg.state == IDLE and not kinds(eff, "enable")


def test_hand_back_requires_bridge():
    lg = TakeoverLogic()
    to_policy(lg)
    to_recording(lg)
    lg.on_key("SPACE")
    eff = lg.on_rec_result("stop", True, True, "Saved", bridge_ready=False)
    assert lg.state == IDLE and not kinds(eff, "enable") and lg.saved == 1


def test_rejected_short_episode_still_hands_back():
    lg = TakeoverLogic()
    to_policy(lg)
    to_recording(lg)
    lg.on_key("SPACE")
    eff = lg.on_rec_result("stop", True, False, "Only 5 frames; episode discarded")
    assert lg.state == POLICY and ("enable", True) in eff and lg.saved == 0


def test_esc_while_recording_saves_then_quits():
    lg = TakeoverLogic()
    to_policy(lg)
    to_recording(lg)
    eff = lg.on_key("ESC")
    assert eff[0] == ("enable", False) and ("rec", "stop") in eff and not kinds(eff, "quit")
    eff = lg.on_rec_result("stop", True, True, "Saved")
    assert kinds(eff, "quit") and not kinds(eff, "enable") and lg.saved == 1


def test_esc_while_unsure_resolves_then_quits():
    lg = TakeoverLogic()
    to_policy(lg)
    to_recording(lg)
    lg.on_key("SPACE")
    lg.on_rec_result("stop", False, False, "")
    eff = lg.on_key("ESC")
    assert ("rec", "stop") in eff and not kinds(eff, "quit")
    assert kinds(lg.on_rec_result("stop", True, False, "Not recording"), "quit")


def test_esc_while_start_pending_waits_then_saves():
    lg = TakeoverLogic()
    to_policy(lg)
    node_tick(lg, held="UP")
    run_until_start(lg)
    assert not kinds(lg.on_key("ESC"), "quit")
    assert ("rec", "stop") in lg.on_rec_result("start", True, True, "Recording")
    assert kinds(lg.on_rec_result("stop", True, True, "Saved"), "quit")


def test_esc_after_unanswered_start_sends_stop():
    lg = TakeoverLogic()
    to_policy(lg)
    node_tick(lg, held="UP")
    run_until_start(lg)
    lg.on_key("ESC")
    eff = lg.on_rec_result("start", False, False, "timeout")
    assert ("rec", "stop") in eff and not kinds(eff, "quit")


def test_esc_in_policy_disables_and_quits():
    lg = TakeoverLogic()
    to_policy(lg)
    eff = lg.on_key("ESC")
    assert eff[0] == ("enable", False) and ("cmd", "STOP") in eff and kinds(eff, "quit")


def test_stale_results_are_ignored():
    lg = TakeoverLogic()
    assert lg.on_rec_result("start", True, True) == []
    assert lg.on_rec_result("stop", True, True) == []
    to_policy(lg)
    assert lg.on_rec_result("stop", True, True) == [] and lg.state == POLICY


def test_interference_while_recording_discards():
    lg = TakeoverLogic()
    to_policy(lg)
    to_recording(lg)
    eff = lg.on_policy_interference()
    assert ("enable", False) in eff and ("rec", "discard") in eff and lg.state == DISCARDING
    eff = lg.on_rec_result("discard", True, True, "Discarded")
    assert lg.state == IDLE and not kinds(eff, "enable") and lg.saved == 0 and lg.discarded == 1


def test_interference_while_start_pending_discards_after_start():
    lg = TakeoverLogic()
    to_policy(lg)
    node_tick(lg, held="UP")
    run_until_start(lg)
    assert not kinds(lg.on_policy_interference(), "rec")
    eff = lg.on_rec_result("start", True, True, "Recording")
    assert ("rec", "discard") in eff and lg.state == DISCARDING


def test_interference_before_start_goes_idle_and_is_ignored_elsewhere():
    lg = TakeoverLogic()
    to_policy(lg)
    node_tick(lg, ["SPACE"])
    eff = lg.on_policy_interference()
    assert lg.state == IDLE and ("cmd", "STOP") in eff and not kinds(eff, "rec")
    assert lg.on_policy_interference() == []          # not driving: nothing to do
