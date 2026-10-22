#!/usr/bin/env python3
"""SmolVLA policy server. Runs in the lerobot venv on the GPU laptop, no ROS.

lerobot 0.6 needs numpy>=2, while ROS2 Jazzy's apt packages are built against
numpy 1.x, so the policy and rclpy are kept in separate processes. The ROS
side (smolvla_bridge.py) sends JPEG frames + the task text over a localhost
socket and gets back one (linear_x, angular_z) action per request.

    python smolvla_server.py --policy ~/pirobot_runs/smolvla_pirobot/checkpoints/last/pretrained_model

Protocol (multiprocessing.connection, pickled dicts):
    -> {"op": "act", "jpeg": bytes, "task": str, "prev": [v, w]}
                                                     <- {"action": [v, w], "ms": float}
       "prev" is the command the bridge actually published last tick (used
       only with --state last_cmd).
    -> {"op": "reset"}                               <- {"ok": True}
    -> {"op": "config", "n_action_steps": int}       <- {"ok": True}
"""
import argparse
import io
import time
from multiprocessing.connection import Listener

import numpy as np
import torch
from PIL import Image

from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
from lerobot.policies.utils import prepare_observation_for_inference

IMG_KEY = "observation.images.camera1"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", required=True, help="checkpoint dir (…/pretrained_model) or HF repo id")
    ap.add_argument("--port", type=int, default=6010)
    ap.add_argument("--authkey", default="pirobot")
    ap.add_argument("--n-action-steps", type=int, default=5,
                    help="actions executed per predicted chunk (chunk is 50); "
                         "smaller = more reactive, more GPU calls")
    ap.add_argument("--size", type=int, nargs=2, metavar=("W", "H"), default=None,
                    help="resize incoming frames to the training size if they differ")
    ap.add_argument("--state", choices=["zeros", "last_cmd"], default="zeros",
                    help="must match convert_to_lerobot.py --state")
    args = ap.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    policy = SmolVLAPolicy.from_pretrained(args.policy)
    policy.config.n_action_steps = args.n_action_steps
    policy.to(device).eval()
    pre, post = make_pre_post_processors(
        policy.config,
        pretrained_path=args.policy,
        preprocessor_overrides={"device_processor": {"device": device.type}},
    )
    policy.reset()
    print(f"Loaded {args.policy} on {device}; n_action_steps={args.n_action_steps}")

    last_action = np.zeros(2, np.float32)
    last_task = None
    with Listener(("127.0.0.1", args.port), authkey=args.authkey.encode()) as listener:
        while True:
            print(f"Waiting for bridge on 127.0.0.1:{args.port} ...")
            with listener.accept() as conn:
                print("Bridge connected")
                policy.reset()
                while True:
                    try:
                        req = conn.recv()
                    except (EOFError, ConnectionResetError):
                        print("Bridge disconnected")
                        break
                    op = req.get("op")
                    if op == "reset":
                        policy.reset()
                        last_action[:] = 0
                        conn.send({"ok": True})
                        continue
                    if op == "config":
                        policy.config.n_action_steps = int(req["n_action_steps"])
                        policy.reset()
                        conn.send({"ok": True})
                        continue
                    if op != "act":
                        conn.send({"error": f"unknown op {op}"})
                        continue

                    t0 = time.perf_counter()
                    task = req["task"]
                    if task != last_task:   # new instruction: drop the queued chunk
                        policy.reset()
                        last_task = task
                    img = Image.open(io.BytesIO(req["jpeg"])).convert("RGB")
                    if args.size and img.size != tuple(args.size):
                        img = img.resize(tuple(args.size), Image.BILINEAR)
                    if args.state == "last_cmd":
                        state = np.asarray(req.get("prev", last_action), np.float32)
                    else:
                        state = np.zeros(2, np.float32)
                    obs = prepare_observation_for_inference(
                        {IMG_KEY: np.asarray(img, dtype=np.uint8), "observation.state": state},
                        device, task=task, robot_type="pirobot_diffdrive",
                    )
                    with torch.inference_mode():
                        action = policy.select_action(pre(obs))
                        action = post(action)
                    a = action.squeeze(0).float().cpu().numpy()[:2]
                    last_action[:] = a
                    conn.send({"action": a.tolist(), "ms": (time.perf_counter() - t0) * 1e3})


if __name__ == "__main__":
    main()
