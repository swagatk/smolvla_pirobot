#!/usr/bin/env bash
# Fine-tune SmolVLA (lerobot/smolvla_base) on the Pi-robot dataset.
# Run inside the lerobot venv on the GPU laptop.
#
#   bash train_smolvla.sh <hf_user>/pirobot_nav ~/pirobot_data/lerobot
#
# BATCH: 64 is the upstream default for an A100. On a laptop GPU start at 8
# (8 GB VRAM) or 16 (12-16 GB) and raise it until memory runs out.
# STEPS: 20k is upstream's default (~4 h on an A100). With a few hours of
# driving data, 10-20k steps is a sensible range; watch the eval loss.
set -euo pipefail

REPO_ID=${1:?usage: train_smolvla.sh <repo_id> [dataset_root]}
ROOT=${2:-}
BATCH=${BATCH:-16}
STEPS=${STEPS:-20000}
OUT=${OUT:-$HOME/pirobot_runs/smolvla_pirobot}

ROOT_ARG=()
[[ -n "$ROOT" ]] && ROOT_ARG=(--dataset.root="$ROOT")

lerobot-train \
  --policy.path=lerobot/smolvla_base \
  --policy.device=cuda \
  --policy.use_amp=true \
  --policy.push_to_hub=false \
  --dataset.repo_id="$REPO_ID" \
  "${ROOT_ARG[@]}" \
  --dataset.image_transforms.enable=true \
  --dataset.eval_split=0.1 \
  --eval_steps=1000 \
  --batch_size="$BATCH" \
  --steps="$STEPS" \
  --save_freq=5000 \
  --log_freq=100 \
  --num_workers=4 \
  --output_dir="$OUT" \
  --job_name=smolvla_pirobot \
  --wandb.enable=false

# Final weights: $OUT/checkpoints/last/pretrained_model  (pass this to smolvla_server.py)
