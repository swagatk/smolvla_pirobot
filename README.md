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
 │ picam_publisher.py       │──────────────▶│ COLLECT: episode_recorder.py  (system py)  │
 │  /camera/image/compressed│               │          episode_ctl.py (keys r/s/d/1-3)   │
 │                          │               │          → raw episodes on disk            │
 │ your motor driver        │◀──────────────│ your keyboard teleop → /cmd_vel            │
 │  subscribes /cmd_vel     │               │                                            │
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
| Action | `[linear_x, angular_z]` from `/cmd_vel` (float32, 2) | SmolVLA predicts continuous actions; the bridge snaps them back to forward/back/left/right/stop |
| State | zeros (default) or previous command | See "copycat" note in `convert_to_lerobot.py` |
| Camera key | `observation.images.camera1` | Matches `smolvla_base`'s first camera, so no `--rename_map`; the other two cameras are simply absent |
| Task | one string per episode | Recorded with the episode, editable later with `--task-map` |
| Timestamps | laptop receive time | Avoids Pi/laptop clock skew |

## 3. Setup

**Network (both machines).** Same subnet, same `ROS_DOMAIN_ID`, e.g. in `~/.bashrc`:
```bash
export ROS_DOMAIN_ID=42
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
```
Check with `ros2 topic hz /camera/image/compressed` on the laptop. If discovery
fails on your router (multicast blocked), use a Fast DDS discovery server.

**Pi.** Copy `pi/picam_publisher.py`. Needs `python3-picamera2` and `python3-opencv` (apt).
```bash
python3 picam_publisher.py --ros-args -p width:=320 -p height:=240 -p fps:=15
```
Add `-p hflip:=true -p vflip:=true` if the camera is mounted upside down.

**Pi safety watchdog (strongly recommended).** Make sure your motor driver stops
the motors if no `/cmd_vel` arrives for ~0.5 s. If WiFi drops during autonomous
driving, this is the only thing that stops the robot.

**Laptop, ROS side:** nothing extra beyond ROS2 Jazzy (`std_srvs`, `sensor_msgs`, `geometry_msgs`).

**Laptop, lerobot venv:**
```bash
python3 -m venv ~/venvs/lerobot && source ~/venvs/lerobot/bin/activate
pip install "lerobot[dataset,smolvla]==0.6.1"     # pulls torch with CUDA
sudo apt install ffmpeg                            # video encoding for the dataset
hf auth login                                      # only needed to push to the Hub
```
Do not `source /opt/ros/jazzy/setup.bash` in this terminal.

## 4. Collecting data

Edit `laptop/pirobot_actions.py` → `COMMANDS` so the speeds match your teleop.

Three terminals on the laptop (ROS sourced), plus the Pi camera node:
```bash
# T1: recorder.  cmd_timeout:=0 assumes your teleop latches (one key press keeps
# the robot moving). If your robot stops when keys are released, set it to 0.3.
python3 episode_recorder.py --ros-args -p out_dir:=$HOME/pirobot_data/raw -p cmd_timeout:=0.0
# T2: episode control: 1/2/3 pick the task, r start, s save, d discard
python3 episode_ctl.py
# T3: your existing keyboard teleop
```
Set `-p cmd_stamped:=true` if your teleop publishes `TwistStamped`, and
`-p cmd_topic:=...` if it is not `/cmd_vel`.

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

## 5. Convert and train (lerobot venv)

```bash
cd laptop
python convert_to_lerobot.py --raw ~/pirobot_data/raw \
    --repo-id <hf_user>/pirobot_nav --root ~/pirobot_data/lerobot
lerobot-dataset-viz --repo-id <hf_user>/pirobot_nav --root ~/pirobot_data/lerobot --episode-index 0   # sanity check

BATCH=16 STEPS=20000 bash train_smolvla.sh <hf_user>/pirobot_nav ~/pirobot_data/lerobot
```
Training starts from `lerobot/smolvla_base` with the vision encoder frozen and
only the action expert trained (SmolVLA's defaults), mixed precision on,
colour/affine augmentation on (no horizontal flip, which would invert steering),
and 10% of episodes held out for eval loss. On an 8 GB laptop GPU use `BATCH=8`.
If you have more data than the laptop can chew through, the same script runs
on a cloud GPU unchanged.

## 6. Run on the robot

```bash
# T1 (lerobot venv): policy server on the GPU
python smolvla_server.py --policy ~/pirobot_runs/smolvla_pirobot/checkpoints/last/pretrained_model --n-action-steps 5
# T2 (ROS sourced): bridge. Close the teleop first; both publish /cmd_vel.
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

## 7. Improving the model (second round)

1. Run the policy, and when it makes a mistake, disable it, take over with the
   teleop **while recording**, and save that correction as a new episode
   (DAgger-style). These episodes are worth far more than more clean driving.
2. If the three tasks interfere, check the per-task episode counts printed by
   the converter and balance them.
3. Paraphrase task strings with `--task-map` (or record a few episodes with
   alternative wordings) if you want it to respond to varied phrasing.

## Files

| File | Runs on | Purpose |
|---|---|---|
| `pi/picam_publisher.py` | Pi | PiCamera2 → `CompressedImage` JPEG |
| `laptop/pirobot_actions.py` | laptop | Discrete command table, labelling and snapping |
| `laptop/episode_recorder.py` | laptop, ROS | Synchronised frame + cmd_vel + task recorder |
| `laptop/episode_ctl.py` | laptop, ROS | One-key start/save/discard/task control |
| `laptop/convert_to_lerobot.py` | laptop, venv | Raw episodes → LeRobot v3 dataset |
| `laptop/train_smolvla.sh` | laptop, venv | `lerobot-train` fine-tuning of `smolvla_base` |
| `laptop/smolvla_server.py` | laptop, venv | Loads the checkpoint, serves actions on localhost |
| `laptop/smolvla_bridge.py` | laptop, ROS | Camera → server → snapped `/cmd_vel`, with enable/stop |
