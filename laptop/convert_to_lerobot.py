#!/usr/bin/env python3
"""Convert raw episodes from episode_recorder.py into a LeRobot (v3) dataset.

Run inside the lerobot venv (no ROS needed). Tested against lerobot 0.6.1.

    python convert_to_lerobot.py --raw ~/pirobot_data/raw \
        --repo-id <hf_user>/pirobot_nav --root ~/pirobot_data/lerobot

Features written:
    observation.images.camera1   video (H, W, 3)  name matches smolvla_base's
                                                  first camera, so no rename_map
    observation.state            float32 (2,)     see --state
    action                       float32 (2,)     [linear_x, angular_z] from cmd_vel
    task                         the language instruction recorded with the episode

--state zeros     (default) constant zeros. The robot has no proprioception that
                  matters for these tasks, and feeding the previous command
                  invites the "copycat" failure where the policy just repeats
                  its last action instead of looking at the image.
--state last_cmd  previous tick's command. Try it if motion is too jerky.
"""
import argparse
import io
import json
from pathlib import Path

import numpy as np
from PIL import Image

from lerobot.datasets.lerobot_dataset import LeRobotDataset


def load_episode(ep_dir: Path):
    meta = json.loads((ep_dir / "meta.json").read_text())
    steps = [json.loads(l) for l in (ep_dir / "steps.jsonl").read_text().splitlines() if l.strip()]
    return meta, steps


def trim_idle(steps, keep: int):
    """Drop leading/trailing 'stop' frames, keeping `keep` of them at each end."""
    moving = [i for i, s in enumerate(steps) if s["cmd"] != "stop"]
    if not moving:
        return []
    lo = max(0, moving[0] - keep)
    hi = min(len(steps), moving[-1] + 1 + keep)
    return steps[lo:hi]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--raw", required=True, type=Path)
    ap.add_argument("--repo-id", required=True)
    ap.add_argument("--root", type=Path, default=None, help="local output dir (default $HF_LEROBOT_HOME/repo_id)")
    ap.add_argument("--state", choices=["zeros", "last_cmd"], default="zeros")
    ap.add_argument("--size", type=int, nargs=2, metavar=("W", "H"), default=None,
                    help="resize frames (default: keep recorded size)")
    ap.add_argument("--keep-idle", type=int, default=5, help="idle frames kept at each end; -1 = no trimming")
    ap.add_argument("--task-map", type=Path, default=None,
                    help="optional JSON {old_task: new_task} to fix or merge task strings")
    ap.add_argument("--push", action="store_true", help="push to the Hugging Face Hub when done")
    args = ap.parse_args()

    task_map = json.loads(args.task_map.read_text()) if args.task_map else {}
    ep_dirs = sorted(d for d in args.raw.iterdir() if d.is_dir() and d.name.startswith("ep_")
                     and not d.name.endswith(".partial") and (d / "meta.json").exists())
    if not ep_dirs:
        raise SystemExit(f"No finished episodes in {args.raw}")

    # Frame size and fps from the first episode.
    meta0, steps0 = load_episode(ep_dirs[0])
    fps = int(round(meta0["fps"]))
    if args.size:
        W, H = args.size
    else:
        W, H = Image.open(ep_dirs[0] / steps0[0]["img"]).size

    features = {
        "observation.images.camera1": {
            "dtype": "video",
            "shape": (H, W, 3),
            "names": ["height", "width", "channels"],
        },
        "observation.state": {
            "dtype": "float32",
            "shape": (2,),
            "names": ["state_0", "state_1"] if args.state == "zeros" else ["prev_linear_x", "prev_angular_z"],
        },
        "action": {
            "dtype": "float32",
            "shape": (2,),
            "names": ["linear_x", "angular_z"],
        },
    }
    ds = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=fps,
        features=features,
        root=args.root,
        robot_type="pirobot_diffdrive",
        use_videos=True,
        image_writer_threads=4,
    )

    totals = {}
    for ep_dir in ep_dirs:
        meta, steps = load_episode(ep_dir)
        if int(round(meta["fps"])) != fps:
            print(f"skip {ep_dir.name}: fps {meta['fps']} != {fps}")
            continue
        if args.keep_idle >= 0:
            steps = trim_idle(steps, args.keep_idle)
        if len(steps) < 10:
            print(f"skip {ep_dir.name}: too short after trimming")
            continue
        task = task_map.get(meta["task"], meta["task"])
        for s in steps:
            img = Image.open(io.BytesIO((ep_dir / s["img"]).read_bytes())).convert("RGB")
            if img.size != (W, H):
                img = img.resize((W, H), Image.BILINEAR)
            state = (np.zeros(2, np.float32) if args.state == "zeros"
                     else np.array([s["prev_v"], s["prev_w"]], np.float32))
            ds.add_frame({
                "observation.images.camera1": np.asarray(img, dtype=np.uint8),
                "observation.state": state,
                "action": np.array([s["v"], s["w"]], np.float32),
                "task": task,
            })
        ds.save_episode()
        totals[task] = totals.get(task, 0) + 1
        print(f"{ep_dir.name}: {len(steps)} frames  '{task}'")

    ds.finalize()
    print("\nEpisodes per task:")
    for t, n in totals.items():
        print(f"  {n:4d}  {t}")
    print(f"Dataset at {ds.root}")
    if args.push:
        ds.push_to_hub()


if __name__ == "__main__":
    main()
