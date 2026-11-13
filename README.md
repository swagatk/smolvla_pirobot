# SmolVLA on the Pi robot: data collection, fine-tuning, deployment

Target: one SmolVLA policy that drives the Raspberry Pi 4 robot from its PiCamera
image plus a language instruction ("move around while avoiding obstacles",
"follow the wall", "follow the ball"). It runs on the GPU laptop and sends
discrete motion commands to the Pi over ROS2 Jazzy on the LAN.

Code was written against **lerobot 0.6.1** (current release, checked against
its source). What was tested: the converter ran end to end on synthetic
episodes and the resulting dataset loads in LeRobot. Not run: the ROS nodes
(no robot here, syntax-checked only), and training / the policy server
(Hugging Face downloads were blocked in the build environment, so
`smolvla_base` could not be fetched). Expect small fixes on first contact.

## 1. Architecture

```
 Raspberry Pi 4 (ROS2 Jazzy)                 GPU laptop (ROS2 Jazzy, Ubuntu 24.04)
 ┌──────────────────────────┐   LAN/DDS     ┌───────────────────────────────────────────┐
 │ pirobot/picam_publisher  │──────────────▶│ COLLECT: episode_recorder.py  (system py)  │
 │  /camera/image/compressed│               │          episode_ctl.py (keys r/s/d/1-3)   │
 │                          │               │          → raw episodes on disk            │
 │ pirobot/motion_subscriber│◀──────────────│ pirobot/keyboard_publisher (arrow keys)    │
 │  /pirobot/command String │               │  → /pirobot/command  UP/DOWN/LEFT/RIGHT/STOP│
 └──────────────────────────┘               │ CONVERT: convert_to_lerobot.py  (venv)     │
                                            │ TRAIN:   train_smolvla.sh       (venv)     │
                                            │ RUN:     smolvla_bridge.py (system py) ◀─┐ │
                                            │          ↕ localhost socket              │ │
                                            │          smolvla_server.py (venv, GPU) ──┘ │
                                            └───────────────────────────────────────────┘
```

Why the split into two Python environments: lerobot 0.6 needs Python ≥ 3.12
(fine, Jazzy is 3.12) but also **numpy ≥ 2**, while Jazzy's apt packages are
built against numpy 1.x. Loading rclpy message modules and lerobot in one
process is asking for "module compiled against NumPy 1.x" crashes. So every
ROS node uses the system Python with no torch, and everything touching
torch/lerobot runs in a venv with no ROS. The bridge and the policy server
talk over a localhost socket using only the standard library.

## 2. Data format and design decisions

| Item | Choice | Why |
|---|---|---|
| Camera | 320×240 JPEG, 15 fps, best-effort QoS | ~15 KB/frame, easy on WiFi; SmolVLA resizes to 512×512 anyway |
| Sample rate | 10 Hz timer on the laptop | Enough for a slow ground robot; recorder pairs each tick with the newest frame |
| Action | `[linear_x, angular_z]` (float32, 2), mapped from `/pirobot/command` by `pirobot_actions.py` (UP → (0.5, 0), LEFT → (0, 0.5), ...) | SmolVLA predicts continuous actions; the bridge snaps them back to forward/back/left/right/stop |
| State | zeros (default) or previous command | See "copycat" note in `convert_to_lerobot.py` |
| Camera key | `observation.images.camera1` | Matches `smolvla_base`'s first camera, so no `--rename_map`; the other two cameras are simply absent |
| Task | one string per episode | Recorded with the episode, editable later with `--task-map` |
| Timestamps | laptop receive time | Avoids Pi/laptop clock skew |

## 3. Setup

**Network (both machines).** Same subnet, same `ROS_DOMAIN_ID` (the Pi uses 19), e.g. in `~/.bashrc`:
```bash
export ROS_DOMAIN_ID=19
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```
Check with `ros2 topic hz /camera/image/compressed` on the laptop. If discovery
fails on your router (multicast blocked), use a Fast DDS discovery server.

**`pirobot` ROS package (both machines).** `pi/pirobot` holds the camera
publisher, the keyboard teleop and the motor node. Build it on the Pi *and* on
the laptop (the laptop only runs `keyboard_publisher` from it):
```bash
cp -r pi/pirobot ~/ros2_ws/src/       # or symlink it
cd ~/ros2_ws && colcon build --packages-select pirobot && source install/setup.bash
```
Pi needs `python3-picamera2`, `python3-opencv`, `python3-rpi.gpio` (apt);
the laptop needs `python3-pygame`.

**Pi: camera + motors** (one command):
```bash
ros2 launch pirobot robot.launch.py            # 320x240 @ 15 fps, watchdog 0.5 s
ros2 launch pirobot robot.launch.py hflip:=true vflip:=true   # camera upside down
```
or the nodes separately: `ros2 run pirobot picam_publisher` and
`ros2 run pirobot motion_subscriber`. The camera uses the full sensor field of
view (`full_fov:=false` reverts to libcamera's default 640x480 centre crop).
The camera takes ~10 s to start.

**Safety watchdog.** `motion_subscriber` stops the motors if no command
arrives for 0.5 s (`-p timeout:=...`), so a WiFi drop or a crashed teleop or
bridge stops the robot. `keyboard_publisher` and `smolvla_bridge` publish
continuously, so it never triggers in normal driving. Ctrl-C on the motor node
always stops the motors and releases the GPIO pins.

**Laptop, ROS side:** nothing extra beyond ROS2 Jazzy (`std_srvs`, `sensor_msgs`, `geometry_msgs`).

**Laptop, lerobot venv:**
```bash
python3 -m venv ~/venvs/lerobot && source ~/venvs/lerobot/bin/activate
pip install "lerobot[dataset,smolvla]==0.6.1"     # pulls torch with CUDA
sudo apt install ffmpeg                            # video encoding for the dataset
hf auth login                                      # only needed to push to the Hub
```
Do not `source /opt/ros/jazzy/setup.bash` in this terminal.

## 4. Calibrate Motors

With the same duty cycle on both motors, the two motors run at slightly
different speeds, so `UP` drives in a curve. `calibrate_motors` lets you trim
each motor's duty cycle until the robot goes straight. Do it once before
collecting data: the policy learns whatever the motors actually do, so keep the
calibration the same for recording and deployment.

On the Pi, in a plain terminal (SSH is fine). Stop `robot.launch.py` /
`motion_subscriber` first; they use the same GPIO pins.
```bash
source ~/ros2_ws/install/setup.bash
ros2 run pirobot calibrate_motors --frequency 100
```
Put the robot on a straight reference (tape line, floor tiles, a wall edge),
press `f`, and watch which way it drifts.

| Key | Action |
|---|---|
| `f` / `b` | Drive forward / backward for the test duration (default 3 s). Any key stops at once. |
| `a` / `z` | Motor A (left) +1 / −1 (`A` / `Z`: ±5) |
| `k` / `m` | Motor B (right) +1 / −1 (`K` / `M`: ±5) |
| `]` / `[` | Both motors +1 / −1: changes speed, keeps the balance |
| `+` / `-` | Test duration ±0.5 s |
| `w` | Save |
| `q` | Quit |

- Drifts **right**: the left motor (A) is faster. Press `z` or `k`.
- Drifts **left**: the right motor (B) is faster. Press `m` or `a`.
- If those are reversed on your robot, the motors are wired the other way
  round; use the opposite keys.

`w` writes `~/.config/pirobot/motor_calib.json` (or the path in
`$PIROBOT_MOTOR_CALIB`) with `duty_a`, `duty_b` and `frequency`.
`motion_subscriber` loads it at startup and logs the values, e.g.
`Duty A=48 B=56 @ 100 Hz`. Without the file it uses 50/50 at 20 Hz.

Tips:
- **Use a higher PWM frequency.** At 20 Hz the software PWM pulses are slow
  enough to make motion jerky and inconsistent, which makes the trim unstable.
  Try `--frequency 100` (saved with the duty cycles). Recalibrate whenever you
  change the frequency.
- **Calibrate on the floor you will record on, with a charged battery.** The
  imbalance depends on surface and battery voltage; re-check if drift returns.
- **Use runs of at least 3–4 s.** Small drift only shows over a distance.
- **Check backward with `b`.** One pair of values is used for both directions;
  backward may drift differently.
- **It is open-loop.** Without wheel encoders some drift remains. For SmolVLA
  that is acceptable: what matters is that the motors behave the same during
  recording and deployment.

## 5. Collecting data

Edit `laptop/pirobot_actions.py` → `COMMANDS` so the speeds match your teleop.

On the Pi: `ros2 launch pirobot robot.launch.py`. Then three terminals on the
laptop (ROS and `~/ros2_ws/install/setup.bash` sourced):
```bash
# T1: recorder. cmd_timeout matches the robot's watchdog: if commands stop
# arriving, the recorded action becomes "stop", just like the robot.
python3 episode_recorder.py --ros-args -p out_dir:=$HOME/pirobot_data/raw -p cmd_timeout:=0.5
# T2: episode control: 1/2/3 pick the task, r start, s save, d discard
python3 episode_ctl.py
# T3: teleop. A small pygame window opens; click it, then hold the arrow keys.
#     Nothing held = STOP. ESC (or closing the window) sends STOP and quits.
ros2 run pirobot keyboard_publisher
```
Keys only reach the window that has focus: press `r` in T2, click the teleop
window and drive, then go back to T2 for `s` or `d`.

Leading/trailing idle frames while you switch terminals are trimmed automatically
at conversion time.

### How much, and how to drive

Behaviour cloning only learns what you show it, so consistency and coverage
matter more than raw hours.

| Task | Episodes (first round) | Length | Driving guidance |
|---|---|---|---|
| Avoid obstacles | 60–100 | 30–60 s | Drive forward by default; always turn the **same way** for the same kind of obstacle layout (e.g. away from the nearer side). Move obstacles between episodes. |
| Follow the wall | 60–100 | 30–60 s | Pick one side (e.g. wall on the right) and say so in the task text. Include episodes that start away from or angled to the wall, so it learns to approach. |
| Follow the ball | 60–100 | 20–60 s | Turn to centre the ball, drive when centred, stop when close. Include episodes where the ball starts off-screen and you rotate to search, and some where it moves. |

Also:
- **Recoveries.** Deliberately drift into a bad state (too close to the wall, ball at the image edge), then correct. Without these the policy has never seen how to get out.
- **Variety.** Different rooms, lighting, floor, times of day, ball positions and distractors. The camera image is the only input.
- **Smooth keys.** Avoid rapid key mashing; the policy will imitate jitter.
- **Discard bad episodes** with `d` rather than keeping crashes.

Roughly 1.5–3 h of driving in total for a first model. Check `dropped ticks`
in the recorder log; above ~10% means the WiFi or camera rate needs fixing.

## 6. Convert and train (lerobot venv)

```bash
cd laptop
python convert_to_lerobot.py --raw ~/pirobot_data/raw \
    --repo-id <hf_user>/pirobot_nav --root ~/pirobot_data/lerobot
lerobot-dataset-viz --repo-id <hf_user>/pirobot_nav --root ~/pirobot_data/lerobot --episode-index 0   # sanity check

BATCH=16 STEPS=20000 bash train_smolvla.sh <hf_user>/pirobot_nav ~/pirobot_data/lerobot
```
Training starts from `lerobot/smolvla_base` with the vision encoder frozen and
only the action expert trained (SmolVLA's defaults), in bfloat16 (SmolVLA's
own weight dtype; `use_amp` is off because lerobot's fp16 AMP scaler crashes on
bfloat16 gradients),
colour/affine augmentation on (no horizontal flip, which would invert steering),
and 10% of episodes held out for eval loss. On an 8 GB laptop GPU use `BATCH=8`.
If you have more data than the laptop can chew through, the same script runs
on a cloud GPU unchanged.

### Pushing the dataset to Hugging Face (optional)

Training reads the local copy (`--root`), so pushing is not required. It is
useful as a backup, or to train on another machine / cloud GPU.

**The dataset is public by default**, and anyone can watch the camera
footage. Push it as private unless you want it public.

1. Log in once with a token that has **write** access (huggingface.co →
   Settings → Access Tokens):
   ```bash
   hf auth login
   hf auth whoami        # the repo id must start with this user (or an org you can write to)
   ```
2. Push an already converted dataset:
   ```bash
   python - <<'EOF'
   from pathlib import Path
   from lerobot.datasets.lerobot_dataset import LeRobotDataset
   ds = LeRobotDataset("<hf_user>/pirobot_nav", root=Path("~/pirobot_data/lerobot").expanduser())
   print(ds.num_episodes, "episodes,", ds.num_frames, "frames")
   ds.push_to_hub(private=True)       # drop private=True to make it public
   EOF
   ```
   The repo id must be the same `--repo-id` used for conversion. If it is
   wrong, re-run `convert_to_lerobot.py` with the correct one. This uploads
   the data, videos and metadata, and tags the dataset version.

   Alternatively, add `--push` to `convert_to_lerobot.py` to push straight
   after conversion. That flag always pushes **public**; use the snippet above
   for a private dataset.
3. Check it at `https://huggingface.co/datasets/<hf_user>/pirobot_nav`, or
   from any machine:
   ```bash
   lerobot-dataset-viz --repo-id <hf_user>/pirobot_nav --episode-index 0
   ```

To train from the Hub copy (e.g. on a cloud GPU), omit the dataset root and the
dataset is downloaded automatically. For a private dataset, run `hf auth login`
on that machine first:
```bash
BATCH=16 STEPS=20000 bash train_smolvla.sh <hf_user>/pirobot_nav
```

### Training on Google Colab

`colab/train_smolvla_colab.ipynb` runs the same fine-tuning on a Colab GPU
(written for an A100). It downloads the dataset from the Hugging Face Hub
(default `swagatk/pirobot_teleop`) pinned to one commit, trains with batch 32
for 6k steps (action expert only, vision encoder frozen), computes held-out eval
loss every 500 steps and checkpoints every 1000 steps. Open it in Colab, choose
`Runtime → Change runtime type → A100` (an L4 works too), then `Run all`.

Why 6k steps: the first run (20k steps, batch 32, Colab L4, 2026-11-12, log
`train_20261112_134204.log`) had its lowest held-out eval loss at step 2000
(0.705), stayed near that up to ~6k, then rose to about 1.0 by 14k–20k while
the training loss kept falling. With ~34 min of driving it overfits quickly;
more data is the real fix. That run took 3 h 05 min on the L4 (0.53 s/step,
GPU-bound).

- Checkpoints go to `MyDrive/smolvla_pirobot/<RUN_NAME>/checkpoints/` (all
  kept by default, ~1.3 GB each; `KEEP_ON_DRIVE=N` keeps the newest N). Training writes to `/content` and a background thread copies each
  finished checkpoint to Drive, because LeRobot's `checkpoints/last` is a
  symlink and Drive cannot store symlinks.
- If the session drops, `Run all` again with the same `RUN_NAME`: it restores
  the newest Drive checkpoint and resumes.
- When training ends (or crashes) the runtime disconnects itself to save
  compute units. Stopping the cell by hand does not disconnect. If an earlier
  cell fails (e.g. the install), the runtime stays up until Colab's idle timeout.
- `run_info.json`, `train_*.log` and `pip_freeze_*.txt` next to the checkpoints
  record the dataset commit, config, command, GPU and package versions.
- A private dataset needs a Colab secret named `HF_TOKEN`.

Tested: the notebook ran end to end on a Colab L4 (20k-step configuration);
the checkpoint sync/prune/resume helpers are tested against a fake checkpoint
tree, `use_amp=false` training and resume were checked locally with lerobot
0.6.1, and the progress-line filter was checked against the first run's log.
The 6k-step configuration itself has not been run yet.

## 7. Run on the robot

```bash
# T1 (lerobot venv): policy server on the GPU
python smolvla_server.py --policy ~/pirobot_runs/smolvla_pirobot/checkpoints/last/pretrained_model --n-action-steps 5
# T2 (ROS sourced): bridge. Close keyboard_publisher first; both publish /pirobot/command.
python3 smolvla_bridge.py
# T3: choose the task, then enable / disable
ros2 topic pub --once /smolvla/task   std_msgs/msg/String "{data: 'follow the ball'}"
ros2 topic pub --once /smolvla/enable std_msgs/msg/Bool   "{data: true}"
ros2 topic echo /smolvla/debug        # chosen command, raw output, latency
ros2 topic pub --once /smolvla/enable std_msgs/msg/Bool   "{data: false}"
```
SmolVLA predicts a 50-step chunk (5 s at 10 Hz). `--n-action-steps 5` executes
only the first 0.5 s before re-planning from a new image, which is what makes
ball following reactive. Lower it for more reactivity if the GPU keeps up
(latency is in `/smolvla/debug`); raise it if the motion is jerky.

`-p discrete:=false` on the bridge sends the raw continuous command instead of
snapping. Changing the task mid-run resets the action queue.

## 8. Improving the model (second round)

1. Run the policy, and when it makes a mistake, take over **while recording**
   and save that correction as a new episode (DAgger-style). These episodes
   are worth far more than more clean driving. `takeover_teleop.py` does this
   from one window (below).
2. If the three tasks interfere, check the per-task episode counts printed by
   the converter and balance them.
3. Paraphrase task strings with `--task-map` (or record a few episodes with
   alternative wordings) if you want it to respond to varied phrasing.

### Recording corrections with `takeover_teleop.py`

One pygame window replaces `keyboard_publisher` and `episode_ctl` during policy
runs. Do **not** run `keyboard_publisher` at the same time: it publishes STOP
continuously and would fight the policy (the window warns, by node name, about
any other publisher on `/pirobot/command`).

```bash
# T1 (lerobot venv): policy server
python smolvla_server.py --policy <run>/checkpoints/006000/pretrained_model --n-action-steps 5
# T2: bridge
python3 smolvla_bridge.py
# T3: recorder. It must record /takeover/command (only the takeover window publishes
#     there), NOT /pirobot/command or /cmd_vel, where the policy also publishes.
#     A separate folder keeps round-2 episodes apart from round 1.
python3 episode_recorder.py --ros-args -p out_dir:=$HOME/pirobot_data/raw_round2 -p cmd_timeout:=0.5 \
    -p pirobot_cmd_topic:=/takeover/command -p cmd_topic:=/takeover/unused
# T4: takeover window (sends this task to both the bridge and the recorder)
python3 takeover_teleop.py --ros-args -p task:="move around while avoiding obstacles"
```

| Key | Policy driving | You driving | Idle |
|---|---|---|---|
| `SPACE` | take over (robot stops) | **save**, give control back to the policy | start the policy |
| arrows | take over and steer at once | drive (nothing held = STOP) | – |
| `S` | pause the policy | save and stay stopped | – |
| `D` | – | discard this correction, stay stopped | – |
| `ESC` | quit | save the correction, then quit | quit |

Window colour shows who is driving: blue = policy, red = you (with `[REC]`
when recording), grey = idle.

How policy commands are kept out of corrections:
- Your commands go to the robot on `/pirobot/command` *and* to the recorder on
  `/takeover/command`, which nothing else publishes. A policy action that was
  still being computed when you took over can reach the robot for a moment, but
  it can never be recorded. The window refuses to record (`NOT RECORDING`,
  with the reason) unless the recorder is subscribed to `/takeover/command`,
  not to `/pirobot/command` or `/cmd_vel`, and has `cmd_timeout > 0` (so idle
  time is labelled STOP). `test_old_wiring_would_record_policy_commands` shows
  what goes wrong otherwise: the policy's late commands land in the first
  frames of the correction.
- On takeover the policy is disabled first; recording starts 0.2–0.3 s later.
- If commands you did not send keep arriving on `/pirobot/command` while the
  policy should be off, the window shows a red warning, re-sends the disable
  every tick while they continue, and **discards** the correction in progress
  (its labels would not match what the robot did).

When the policy is re-enabled:
- Only after the recorder has *answered* the save, and only if the bridge is
  listening.
- If a recorder call goes unanswered (or it replies "Already recording"), the
  recorder might still be recording, so the policy is blocked. `SPACE` in IDLE
  then sends a stop first; press `SPACE` again once it has answered. A fresh
  `idle` status from the recorder also clears the block.
- If the recorder is not running you can still drive; the window says
  `NOT RECORDING` and handing back saves nothing.

The task in the window is re-sent to the recorder at each takeover and to the
bridge at each enable. If the recorder still starts with a different task (it
reports the task in its reply), that correction is discarded at once. On exit
(ESC, closing the window, Ctrl-C, or a crash) the tool disables the policy,
sends STOP and asks the recorder to stop any open episode; STOP is the last
command it sends.

Known limitation: after an unanswered recorder call, a fresh `idle` status
clears the block. If the delayed call reaches the recorder *after* that status
(a DDS delay of more than ~1.5 s), the recorder can start an episode the tool
does not know about. Thanks to `/takeover/command` it would contain only STOP
labels, never policy commands; delete it if you find one.

Good corrections start just before the failure (take over as soon as you see
it going wrong, e.g. heading into a corner), drive out of it the way you want
the policy to, and end once it is clear (a few seconds). Each one is saved as
its own episode. Prefer taking over with an **arrow** (it starts steering at
once). `SPACE` stops the robot first, so the episode begins with STOP frames
while you react: `convert_to_lerobot.py` keeps `--keep-idle` (default 5, i.e.
0.5 s) of them at each end, and STOP labels at the failure moment teach the
policy to stop there. Use `--keep-idle 1` when converting corrections.

Tests (no robot needed; ROS 2 Jazzy sourced, pygame and pytest installed):
```bash
cd laptop && python3 -m pytest -q tests/
```
They run the real recorder and bridge classes in-process with a fake camera
and a fake policy that keeps publishing for 0.4 s after it is disabled (or
never stops). `tests/conftest.py` puts them on an isolated, localhost-only ROS
domain (88, or `$PIROBOT_TEST_ROS_DOMAIN`, never 19), so they cannot reach the
robot.

## Files

| File | Runs on | Purpose |
|---|---|---|
| `pi/pirobot/launch/robot.launch.py` | Pi | Starts camera + motion nodes |
| `pi/pirobot/pirobot/picam_publisher.py` | Pi | PiCamera2 → `CompressedImage` JPEG |
| `pi/pirobot/pirobot/motion_subscriber.py` | Pi | `/pirobot/command` → motors, with watchdog |
| `pi/pirobot/pirobot/motor_control.py` | Pi | GPIO PWM motor driver; loads the motor calibration |
| `pi/pirobot/pirobot/calibrate_motors.py` | Pi | Interactive duty-cycle trim so `UP` drives straight |
| `pi/pirobot/pirobot/keyboard_publisher.py` | laptop | Arrow keys → `/pirobot/command` |
| `laptop/pirobot_actions.py` | laptop | Discrete command table, labelling and snapping |
| `laptop/episode_recorder.py` | laptop, ROS | Synchronised frame + command + task recorder |
| `laptop/episode_ctl.py` | laptop, ROS | One-key start/save/discard/task control |
| `laptop/convert_to_lerobot.py` | laptop, venv | Raw episodes → LeRobot v3 dataset |
| `laptop/train_smolvla.sh` | laptop, venv | `lerobot-train` fine-tuning of `smolvla_base` |
| `colab/train_smolvla_colab.ipynb` | Colab | Same fine-tuning on a Colab GPU, Drive checkpoints, auto-resume and auto-disconnect |
| `laptop/smolvla_server.py` | laptop, venv | Loads the checkpoint, serves actions on localhost |
| `laptop/smolvla_bridge.py` | laptop, ROS | Camera → server → snapped `/pirobot/command`, with enable/stop |
| `laptop/takeover_teleop.py` | laptop, ROS | One-window policy takeover + correction recording (DAgger) |
| `laptop/takeover_logic.py` | laptop | Takeover state machine (pure Python, unit-tested) |
| `laptop/tests/` | laptop | Tests for the takeover tool (`python3 -m pytest -q tests/`) |
