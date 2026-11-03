#!/usr/bin/env python3
"""CPU armor detection and planar pose estimation for the HerKules assignment."""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import onnxruntime as ort
import yaml


COLORS = ("blue", "red", "neutral", "purple")
PATTERNS = ("sentry", "1", "2", "3", "4", "5", "outpost", "base_small", "base_big")


@dataclass(frozen=True)
class Calibration:
    matrix: np.ndarray
    distortion: np.ndarray
    width: int
    height: int

    @classmethod
    def load(cls, path: Path) -> Calibration:
        with path.open(encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        k = np.asarray(data["camera_matrix"], dtype=np.float64)
        d = np.asarray(data["distort_coeffs"], dtype=np.float64).reshape(-1)
        width, height = int(data["image_width"]), int(data["image_height"])
        if (k.shape != (3, 3) or d.size not in (4, 5, 8, 12, 14)
                or not np.isfinite(k).all() or not np.isfinite(d).all()
                or k[0, 0] <= 0 or k[1, 1] <= 0
                or not np.allclose(k[2], [0, 0, 1]) or min(width, height) <= 0):
            raise ValueError("Invalid camera calibration")
        return cls(k, d, width, height)


@dataclass(frozen=True)
class Letterbox:
    scale_x: float
    scale_y: float
    left: int
    top: int

    def restore(self, points: np.ndarray) -> np.ndarray:
        # Use the actual rounded resize dimensions, including for odd image sizes.
        return ((np.asarray(points, dtype=np.float64) - [self.left, self.top])
                / [self.scale_x, self.scale_y])


def preprocess(bgr: np.ndarray) -> tuple[np.ndarray, Letterbox]:
    """Centered 114 letterbox -> RGB / 255 -> contiguous float16 NCHW."""
    height, width = bgr.shape[:2]
    ratio = min(640 / width, 640 / height)
    new_w, new_h = max(1, round(width * ratio)), max(1, round(height * ratio))
    left, top = (640 - new_w) // 2, (640 - new_h) // 2
    resized = cv2.resize(bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((640, 640, 3), 114, dtype=np.uint8)
    canvas[top:top + new_h, left:left + new_w] = resized
    rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
    tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1)[None], dtype=np.float16)
    return tensor, Letterbox(new_w / width, new_h / height, left, top)


def nms(corners: np.ndarray, scores: np.ndarray, threshold: float) -> list[int]:
    """Class-agnostic, floating-point AABB NMS, ranked by objectness only."""
    if len(scores) == 0:
        return []
    low, high = corners.min(axis=1), corners.max(axis=1)
    area = np.prod(np.maximum(high - low, 0), axis=1)
    order = np.argsort(-scores, kind="stable")
    keep = []
    while order.size:
        best = int(order[0])
        keep.append(best)
        rest = order[1:]
        overlap = np.maximum(np.minimum(high[best], high[rest])
                             - np.maximum(low[best], low[rest]), 0)
        intersection = np.prod(overlap, axis=1)
        union = area[best] + area[rest] - intersection
        iou = np.divide(intersection, union, out=np.zeros_like(union), where=union > 0)
        order = rest[iou <= threshold]
    return keep


def decode(output: np.ndarray, transform: Letterbox,
           confidence_threshold: float = 0.5, nms_threshold: float = 0.5) -> list[dict]:
    if output.ndim != 3 or output.shape[0] != 1 or output.shape[2] != 22:
        raise ValueError(f"Expected model output [1, N, 22], got {output.shape}")
    rows = output[0]
    rows = rows[np.isfinite(rows).all(axis=1)]
    # logaddexp implements sigmoid without overflow for large negative logits.
    scores = np.exp(-np.logaddexp(0.0, -rows[:, 8].astype(np.float64)))
    selected = scores >= confidence_threshold
    rows, scores = rows[selected], scores[selected]
    corners = transform.restore(rows[:, :8].reshape(-1, 4, 2))
    result = []
    for index in nms(corners, scores, nms_threshold):
        result.append({
            "color": COLORS[int(np.argmax(rows[index, 9:13]))],
            "pattern": PATTERNS[int(np.argmax(rows[index, 13:22]))],
            "confidence": float(scores[index]),
            # Keep the model's UL, LL, LR, UR correspondence; do not sort or clip.
            "corners_px": corners[index].tolist(),
            "corners_camera_m": None,
            "reprojection_error_px": None,
        })
    return result


def object_points(pattern: str) -> np.ndarray:
    if pattern not in PATTERNS:
        raise ValueError(f"Unknown pattern: {pattern}")
    width = 0.230 if pattern in ("1", "base_big") else 0.135
    height = 0.056
    return np.array([[-width / 2, -height / 2, 0], [-width / 2, height / 2, 0],
                     [width / 2, height / 2, 0], [width / 2, -height / 2, 0]],
                    dtype=np.float64)


def estimate_pose(corners_px: np.ndarray, pattern: str, calibration: Calibration,
                  max_error_px: float = 5.0) -> tuple[list | None, float | None]:
    """Choose the best positive-depth IPPE/LM candidate; reject bad fits."""
    observed = np.ascontiguousarray(corners_px, dtype=np.float64)
    if observed.shape != (4, 2) or not np.isfinite(observed).all():
        return None, None
    contour = observed.astype(np.float32)
    if not cv2.isContourConvex(contour) or abs(cv2.contourArea(contour)) < 1.0:
        return None, None
    model = object_points(pattern)
    k, d = calibration.matrix, calibration.distortion
    try:
        success, rotations, translations, _ = cv2.solvePnPGeneric(
            model, observed, k, d, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return None, None
    if not success:
        return None, None
    best_points, best_error = None, float("inf")
    for rvec, tvec in zip(rotations, translations):
        if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
            continue
        candidates = [(rvec, tvec)]
        try:
            candidates.append(cv2.solvePnPRefineLM(
                model, observed, k, d, rvec.copy(), tvec.copy()))
        except cv2.error:
            pass  # The original IPPE candidate can still be valid.
        for rotation, translation in candidates:
            if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
                continue
            matrix, _ = cv2.Rodrigues(rotation)
            camera_points = model @ matrix.T + translation.reshape(1, 3)
            if not np.isfinite(camera_points).all() or np.any(camera_points[:, 2] <= 0):
                continue
            projected, _ = cv2.projectPoints(model, rotation, translation, k, d)
            error = float(np.sqrt(np.mean(np.sum(
                (projected.reshape(4, 2) - observed) ** 2, axis=1))))
            if np.isfinite(error) and error < best_error and error <= max_error_px:
                best_points, best_error = camera_points.tolist(), error
    return (best_points, best_error) if best_points is not None else (None, None)


class Detector:
    def __init__(self, model: Path, calibration: Calibration, confidence: float = 0.5,
                 nms_iou: float = 0.5, max_error_px: float = 5.0, threads: int = 2):
        options = ort.SessionOptions()
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        self.session = ort.InferenceSession(str(model), sess_options=options,
                                            providers=["CPUExecutionProvider"])
        inputs, outputs = self.session.get_inputs(), self.session.get_outputs()
        if (len(inputs) != 1 or inputs[0].shape != [1, 3, 640, 640]
                or inputs[0].type != "tensor(float16)"
                or len(outputs) != 1 or outputs[0].shape != [1, 25200, 22]):
            raise ValueError("Model does not match the assignment's ONNX interface")
        self.input_name, self.output_name = inputs[0].name, outputs[0].name
        self.calibration = calibration
        self.confidence, self.nms_iou, self.max_error_px = confidence, nms_iou, max_error_px

    def detect(self, frame: np.ndarray) -> list[dict]:
        if frame.shape[:2] != (self.calibration.height, self.calibration.width):
            raise ValueError("Video resolution differs from calibration; supply matching intrinsics")
        tensor, transform = preprocess(frame)
        output = self.session.run([self.output_name], {self.input_name: tensor})[0]
        detections = decode(output, transform, self.confidence, self.nms_iou)
        for detection in detections:
            points, error = estimate_pose(detection["corners_px"], detection["pattern"],
                                          self.calibration, self.max_error_px)
            detection["corners_camera_m"] = points
            detection["reprojection_error_px"] = error
        return detections


def annotate(frame: np.ndarray, record: dict, calibration: Calibration) -> np.ndarray:
    image = frame.copy()
    palette = {"blue": (255, 180, 50), "red": (70, 80, 255),
               "neutral": (220, 220, 220), "purple": (255, 90, 220)}
    for detection_index, det in enumerate(record["detections"]):
        color = palette[det["color"]]
        points = np.rint(det["corners_px"]).astype(np.int32)
        cv2.polylines(image, [points], True, color, 2, cv2.LINE_AA)
        for index, point in enumerate(points):
            cv2.circle(image, tuple(point), 3, color, -1, cv2.LINE_AA)
            cv2.putText(image, str(index), tuple(point + [4, -4]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
        label = f"#{detection_index}  {det['color']} / {det['pattern']}  confidence={det['confidence']:.2f}"
        if det["corners_camera_m"] is not None:
            camera_points = np.asarray(det["corners_camera_m"])
            projected, _ = cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3),
                                              calibration.matrix, calibration.distortion)
            for point in np.rint(projected.reshape(4, 2)).astype(int):
                cv2.circle(image, tuple(point), 5, (60, 255, 100), 1, cv2.LINE_AA)
            label += f"  z={camera_points[:, 2].mean():.2f}m  rms={det['reprojection_error_px']:.2f}px"
        else:
            label += "  pose unavailable"
        x = int(np.clip(points[:, 0].mean(), 0, frame.shape[1] - 40))
        y = int(np.clip(points[:, 1].min() - 15, 24, frame.shape[0] - 5))
        cv2.putText(image, f"#{detection_index}", (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.6, color, 2, cv2.LINE_AA)
        # A fixed legend avoids overlapping long labels on neighboring armors.
        legend_y = 65 + detection_index * 25
        cv2.putText(image, label, (20, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(image, label, (20, legend_y), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    color, 1, cv2.LINE_AA)
    title = f"Frame {record['frame_index']} | {record['timestamp_s']:.3f}s | {len(record['detections'])} detections"
    cv2.putText(image, title, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(image, title, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def run(args: argparse.Namespace) -> dict:
    inputs = [args.model.resolve(), args.video.resolve(), args.calibration.resolve()]
    outputs = [args.output.resolve()]
    if args.annotated_video:
        outputs.append(args.annotated_video.resolve())
    if any(path in inputs for path in outputs) or len(set(outputs)) != len(outputs):
        raise ValueError("Output paths must differ from each other and from all inputs")
    calibration = Calibration.load(args.calibration)
    detector = Detector(args.model, calibration, args.confidence, args.nms_iou,
                        args.max_reprojection_error, args.threads)
    capture = cv2.VideoCapture(str(args.video))
    writer = None
    started = time.perf_counter()
    frames, count, poses = 0, 0, 0
    try:
        if not capture.isOpened():
            raise ValueError(f"Cannot open video: {args.video}")
        fps = float(capture.get(cv2.CAP_PROP_FPS))
        if not np.isfinite(fps) or fps <= 0:
            raise ValueError("Video must declare a valid fixed frame rate")
        expected_frames = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if args.preview_dir:
            args.preview_dir.mkdir(parents=True, exist_ok=True)
        if args.annotated_video:
            args.annotated_video.parent.mkdir(parents=True, exist_ok=True)
            writer = cv2.VideoWriter(str(args.annotated_video), cv2.VideoWriter_fourcc(*"mp4v"),
                                     fps, (calibration.width, calibration.height))
            if not writer.isOpened():
                raise ValueError(f"Cannot create annotated video: {args.annotated_video}")
        with args.output.open("w", encoding="utf-8") as stream:
            while args.max_frames is None or frames < args.max_frames:
                ok, frame = capture.read()
                if not ok:
                    if expected_frames > 0 and frames < expected_frames:
                        raise ValueError(f"Video ended at frame {frames}; expected {expected_frames}")
                    break
                detections = detector.detect(frame)
                record = {"frame_index": frames, "timestamp_s": frames / fps,
                          "detections": detections}
                stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                preview = args.preview_dir and frames % args.preview_every == 0
                if writer is not None or preview:
                    overlay = annotate(frame, record, calibration)
                    if writer is not None:
                        writer.write(overlay)
                    if preview and not cv2.imwrite(str(args.preview_dir / f"frame_{frames:04d}.png"), overlay):
                        raise OSError("Could not save preview image")
                frames += 1
                count += len(detections)
                poses += sum(det["corners_camera_m"] is not None for det in detections)
                if frames % 60 == 0:
                    print(f"{frames}/{expected_frames} frames, {count} detections", file=sys.stderr)
        if frames == 0:
            raise ValueError("Video contains no decodable frames")
    finally:
        capture.release()
        if writer is not None:
            writer.release()
    return {"frames": frames, "fps": fps, "detections": count, "valid_poses": poses,
            "elapsed_s": round(time.perf_counter() - started, 3), "output": str(args.output)}


def fraction(value: str) -> float:
    result = float(value)
    if not 0 < result <= 1:
        raise argparse.ArgumentTypeError("Must be in (0, 1]")
    return result


def positive_int(value: str) -> int:
    result = int(value)
    if result <= 0:
        raise argparse.ArgumentTypeError("Must be a positive integer")
    return result


def positive_float(value: str) -> float:
    result = float(value)
    if not np.isfinite(result) or result <= 0:
        raise argparse.ArgumentTypeError("Must be a finite positive number")
    return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    for option in ("model", "video", "calibration", "output"):
        parser.add_argument(f"--{option}", type=Path, required=True)
    parser.add_argument("--confidence", type=fraction, default=0.5)
    parser.add_argument("--nms-iou", type=fraction, default=0.5)
    parser.add_argument("--max-reprojection-error", type=positive_float, default=5.0,
                        help="Reject poses above this RMS in original pixels (default: 5)")
    parser.add_argument("--threads", type=positive_int, default=2)
    parser.add_argument("--max-frames", type=positive_int, help="Limit frames for a smoke test")
    parser.add_argument("--annotated-video", type=Path, help="Optional annotated MP4")
    parser.add_argument("--preview-dir", type=Path, help="Optional sampled PNG directory")
    parser.add_argument("--preview-every", type=positive_int, default=90)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    try:
        result = run(args)
    except (OSError, ValueError, KeyError, cv2.error, ort.capi.onnxruntime_pybind11_state.Fail,
            ort.capi.onnxruntime_pybind11_state.NoSuchFile) as error:
        parser.exit(1, f"Error: {error}\n")
    print(json.dumps(result, ensure_ascii=False, allow_nan=False))


if __name__ == "__main__":
    main()
