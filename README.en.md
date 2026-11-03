# HerKules vision assignment: detection and pose estimation

[中文](README.md) · [English](README.en.md)

This solution implements **route A (detection)** with Python, CPU ONNX Runtime and OpenCV.
The untouched `aiming/` directory belongs to the alternative route, which is not required.

## Run from the repository root

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
To submit, publish your solution to your own public GitHub repository and submit its URL on the training website.
