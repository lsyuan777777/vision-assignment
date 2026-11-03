# HerKules vision assignment: detection and predictive aiming

[中文](README.md) · [English](README.en.md)

This repository implements **both route A (detection)** and **route B (predictive aiming)**.
The assignment only requires one. Route A uses Python, CPU ONNX Runtime and OpenCV;
route B uses Rust and the unmodified official simulation/evaluation infrastructure.

## Run route A from the repository root

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r detection/requirements.txt
for clip in small large; do
  .venv/bin/python detection/detect.py \
    --model detection/assets/0526.onnx \
    --video "detection/assets/$clip.mp4" \
    --calibration detection/assets/calibration.yaml \
    --output "detection/results/$clip.jsonl" \
    --annotated-video "detection/results/${clip}_annotated.mp4" \
    --preview-dir "detection/results/${clip}_previews"
done
.venv/bin/python -m unittest discover -s detection/tests -v
.venv/bin/python detection/validate.py \
  detection/results/small.jsonl detection/results/large.jsonl \
  --calibration detection/assets/calibration.yaml
```

Tested with Python 3.12.3. Model, videos and calibration are included in `detection/assets/`.
No training, GPU, ROS, or standalone ffmpeg installation is needed.

## Method and results

Centered 114 letterboxing, RGB normalization, float16 NCHW input, objectness sigmoid,
all-color decoding and class-agnostic AABB NMS are implemented explicitly.
Original corner correspondence is preserved. Pattern IDs select small/large physical geometry.
IPPE and LM refinement yield candidate poses; the best finite positive-depth fit is selected.
Fits exceeding 5 px Euclidean RMS retain their detections with null pose fields.

| Video | Frames | Detections | Valid poses | Null poses | Median / P95 RMS |
| --- | ---: | ---: | ---: | ---: | --- |
| small.mp4 | 360 | 396 | 396 | 0 | 0.119 / 0.302 px |
| large.mp4 | 360 | 151 | 148 | 3 | 0.275 / 1.822 px |

All 13 unit tests pass. The independent validator checks all 720 output records, timestamps,
finite values, class names, positive depth, physical dimensions and reprojection RMS.
No detection or 3D ground truth is supplied, so these are consistency checks, not accuracy claims.

See [the detailed implementation guide](detection/README.md),
[original assignment](detection/ASSIGNMENT.md), and [validation report](detection/results/validation.json).
The code was developed with AI assistance. The official template is
[HKUAstar/vision-training-assignment](https://github.com/HKUAstar/vision-training-assignment) at `c9fa5e9`.
## Route B

Requires Rust 1.98.1 or newer (tested with 1.98.1).

```bash
cd aiming
cargo test --locked --no-default-features
cargo run --locked --no-default-features --example evaluate
cargo run --locked --no-default-features -- \
  --offline --config scenarios/moving.json --record target/moving.mcap
cargo run --locked -- --replay target/moving.mcap
```

The controller compensates observation age, launch delay and flight time, selects a plate,
solves a bounded fixed-point intercept without changing the plate index, and gates firing
using the final 3D normal and projectile velocity relative to the plate.

Each official 20-second stationary, spinning and moving scenario produces **80 shots,
80 hits, 0 misses and 0 invalid commands**, after all projectiles settle.
All 24 headless tests pass, including 27 extra combinations of motion, spin and initial phase.
These results apply to the supplied idealized simulator, not to real hardware.
See [the route B guide](aiming/README.en.md) and [evaluation report](aiming/results/evaluation.json).

The public submission repository is [lsyuan777777/vision-assignment](https://github.com/lsyuan777777/vision-assignment).
Its URL still needs to be submitted on the training website.
