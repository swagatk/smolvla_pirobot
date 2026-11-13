"""State machine for single-key policy takeover (DAgger-style corrections).

Pure Python (no ROS, no pygame) so it can be unit-tested. `takeover_teleop.py`
feeds it key events, ticks and recorder replies, and carries out the effects
it returns, in order.

States
    IDLE       policy disabled, robot stopped, nothing recorded
    POLICY     smolvla_bridge drives; this tool publishes nothing
    STARTING   human has control; recording is requested `start_delay_ticks`
               ticks after the takeover (never in the takeover tick itself), so
               the policy's last command has been flushed first. Save, discard
               and hand-back keys wait until the recorder answered the start.
    HUMAN      human drives with the arrow keys (recording, unless the
               recorder could not be started)
    SAVING     waiting for the recorder to answer stop; robot stopped
    DISCARDING waiting for the recorder to answer discard; robot stopped

Effects (tuples)
    ("enable", bool)       publish /smolvla/enable
    ("cmd", str)           publish one human command ('UP', ..., 'STOP')
    ("rec", "start"|"stop"|"discard")   call the recorder service
    ("msg", str)           show a message to the operator
    ("quit",)              the node may exit now

Safety rules
    - On takeover the policy is disabled before anything else.
    - `recorder_unsure` is set whenever the recorder might be recording without
      the tool knowing (unanswered start/stop/discard, or "Already recording").
      While it is set the policy is never enabled; the only way out is an
      answered stop/discard (SPACE in IDLE sends one) or the node reporting a
      fresh "idle" recorder status (`on_recorder_idle`).
    - Handing back to the policy happens only after the recorder answered the
      save, only if the bridge is listening, and never while unsure.
    - Recorder replies that arrive after the tool gave up waiting are ignored
      by the node, which is why an unanswered call sets `recorder_unsure`.
"""

IDLE, POLICY, STARTING, HUMAN, SAVING, DISCARDING = (
    "IDLE", "POLICY", "STARTING", "HUMAN", "SAVING", "DISCARDING")

ARROWS = ("UP", "DOWN", "LEFT", "RIGHT")


class TakeoverLogic:
    """See the module docstring."""

    def __init__(self, start_delay_ticks=3):
        self.start_delay_ticks = max(2, int(start_delay_ticks))
        self.state = IDLE
        self.recording = False         # the tool started a recording that is still open
        self.recorder_unsure = False   # the recorder might be recording without us knowing
        self.after_save = None         # POLICY or IDLE, decided when the save was asked for
        self.start_requested = False
        self.start_ticks = 0
        self.quitting = False
        self.quit_discard = False      # interference seen while a start was pending
        self.saved = 0                 # corrections saved this session
        self.discarded = 0

    # ---- key events (edge-triggered) ---------------------------------
    def on_key(self, key, bridge_ready=True, recorder_busy=False):
        """key: 'SPACE', 'S', 'D', 'ESC' or 'ARROW_<UP|DOWN|LEFT|RIGHT>' (a key press, so short taps count)."""
        s = self.state
        if key == "ESC":
            return self._quit()
        if s in (SAVING, DISCARDING):
            return [("msg", "waiting for the recorder...")]
        if s == STARTING and self.start_requested and key in ("SPACE", "S", "D"):
            return [("msg", "waiting for the recorder to start...")]
        if key.startswith("ARROW_"):
            if s == POLICY:
                return self._takeover(key[len("ARROW_"):])
            return []
        if key == "SPACE":
            if s == IDLE:
                return self._start_policy(bridge_ready, recorder_busy)
            if s == POLICY:
                return self._takeover("STOP")
            if s in (STARTING, HUMAN):
                return self._finish("stop", then=POLICY, bridge_ready=bridge_ready)
        if key == "S":
            if s == POLICY:
                self.state = IDLE
                return [("enable", False), ("cmd", "STOP"), ("msg", "policy paused")]
            if s in (STARTING, HUMAN):
                return self._finish("stop", then=IDLE)
        if key == "D" and s in (STARTING, HUMAN):
            return self._finish("discard", then=IDLE)
        return []

    # ---- periodic tick ----------------------------------------------
    def on_tick(self, held):
        """held: the arrow currently held ('UP', ...) or 'STOP'. Called at the control rate."""
        s = self.state
        if s == POLICY:
            if held in ARROWS:                          # grab control by just steering
                return self._takeover(held)
            return []
        if s == STARTING:
            self.start_ticks += 1
            if not self.start_requested and self.start_ticks >= self.start_delay_ticks:
                self.start_requested = True
                return [("cmd", held), ("rec", "start")]
            return [("cmd", held)]
        if s == HUMAN:
            return [("cmd", held)]
        return []

    def on_policy_interference(self):
        """Commands we did not send keep reaching the robot while the human drives.
        The recording's labels would not match what the robot did, so it is discarded."""
        if self.state not in (STARTING, HUMAN):
            return []
        out = [("enable", False), ("msg", "another node kept driving the robot: correction discarded")]
        if self.recording or self.recorder_unsure:
            return out + self._finish("discard", then=IDLE)
        if self.start_requested:
            self.quit_discard = True          # discard once the pending start is answered
            return out
        self.state = IDLE
        return out + [("cmd", "STOP")]

    def on_recorder_idle(self):
        """The node saw a fresh 'idle' status from the recorder (and no call is pending)."""
        if self.state in (IDLE, POLICY, HUMAN) and not self.recording:
            self.recorder_unsure = False

    # ---- recorder replies -------------------------------------------
    def on_rec_result(self, op, answered, success, message="", task_ok=True, bridge_ready=True):
        """answered=False: no reply (timeout), so the call may or may not have taken effect.
        A refusal the node makes itself (service not available, wrong wiring) is answered=True."""
        if op == "start":
            if self.state != STARTING or not self.start_requested:
                return []
            self.start_requested = False
            self.state = HUMAN
            if answered and success:
                self.recording = True
                if self.quit_discard:
                    self.quit_discard = False
                    return [("msg", "discarding (interference during start)")] + self._finish("discard", then=IDLE)
                if not task_ok:
                    out = [("msg", f"recorder used a different task ({message}); discarding")]
                    return out + self._finish("discard", then=IDLE)
                if self.quitting:
                    return self._finish("stop", then=IDLE)
                return [("msg", f"recording: {message}")]
            self.recording = False
            if not answered or "already recording" in message.lower():
                self.recorder_unsure = True
            if self.quit_discard:
                self.quit_discard = False
                return self._finish("discard", then=IDLE)
            if self.quitting:
                return self._finish("stop", then=IDLE)   # sends stop if unsure, else quits
            reason = message or "recorder not answering"
            return [("msg", f"NOT RECORDING ({reason}); you are still driving")]

        if op in ("stop", "discard"):
            if self.state not in (SAVING, DISCARDING):
                return []
            was_recording = self.recording
            self.recording = False
            then, self.after_save = self.after_save, None
            if not answered:
                self.recorder_unsure = True
                self.state = IDLE
                out = [("cmd", "STOP"),
                       ("msg", f"recorder did not answer {op}; policy stays OFF (SPACE retries the stop)")]
                return out + ([("quit",)] if self.quitting else [])
            self.recorder_unsure = False                # it answered, so it is not recording now
            out = [("msg", message or op)]
            if op == "stop" and success:
                self.saved += 1
                if not was_recording:
                    out.append(("msg", f"saved an episode this tool did not start; check it: {message}"))
            if op == "discard" and success:
                self.discarded += 1
            if self.quitting:
                self.state = IDLE
                return out + [("quit",)]
            if then == POLICY:
                if bridge_ready:
                    self.state = POLICY
                    return out + [("enable", True), ("msg", "policy driving")]
                out.append(("msg", "smolvla_bridge not listening; staying stopped"))
            self.state = IDLE
            return out
        return []

    # ---- helpers ------------------------------------------------------
    def _start_policy(self, bridge_ready, recorder_busy):
        if self.recorder_unsure:
            # Resolve first: an answered stop clears the doubt. The policy is not started yet.
            self.state = SAVING
            self.after_save = IDLE
            return [("cmd", "STOP"), ("rec", "stop"),
                    ("msg", "recorder state unknown: stopping it first; press SPACE again after")]
        if recorder_busy:
            return [("msg", "episode_recorder reports it is recording; stop it before starting the policy")]
        if not bridge_ready:
            return [("msg", "smolvla_bridge is not listening on /smolvla/enable; policy not started")]
        self.state = POLICY
        return [("enable", True), ("msg", "policy driving")]

    def _takeover(self, first_cmd):
        self.state = STARTING
        self.recording = False
        self.start_requested = False
        self.start_ticks = 0
        self.quit_discard = False
        return [("enable", False), ("cmd", first_cmd), ("msg", "TAKEOVER: you are driving")]

    def _finish(self, op, then, bridge_ready=True):
        if self.recording or self.recorder_unsure:
            self.state = SAVING if op == "stop" else DISCARDING
            self.after_save = then
            return [("cmd", "STOP"), ("rec", op)]
        # Nothing was recorded and the recorder is known not to be recording.
        out = [("cmd", "STOP")]
        if self.quitting:
            self.state = IDLE
            return out + [("quit",)]
        if then == POLICY and bridge_ready:
            self.state = POLICY
            return out + [("enable", True), ("msg", "policy driving (nothing was recorded)")]
        self.state = IDLE
        if then == POLICY:
            return out + [("msg", "smolvla_bridge not listening; stopped (nothing was recorded)")]
        return out + [("msg", "stopped (nothing was recorded)")]

    def _quit(self):
        self.quitting = True
        if self.state in (SAVING, DISCARDING) or (self.state == STARTING and self.start_requested):
            return [("enable", False), ("cmd", "STOP")]   # quit once the recorder answers
        if self.recording or self.recorder_unsure:
            return [("enable", False)] + self._finish("stop", then=IDLE)   # keep / resolve the episode
        self.state = IDLE
        return [("enable", False), ("cmd", "STOP"), ("quit",)]
