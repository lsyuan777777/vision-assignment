#!/usr/bin/env python3
"""Independently validate every JSONL record, geometric size and reprojection RMS."""
import argparse
from collections import Counter
import json
import math
from pathlib import Path

import cv2
import numpy as np

try:
    from .detect import Calibration, COLORS, PATTERNS
except ImportError:
    from detect import Calibration, COLORS, PATTERNS


def require(condition, message):
    if not condition:
        raise ValueError(message)


def finite_number(value):
    return type(value) in (float, int) and math.isfinite(value)


def reject_constant(value):
    raise ValueError(f'Non-finite JSON constant: {value}')


def validate(path, calibration, expected_frames=360, fps=30.0):
    counts, colors, patterns, errors, depths = [], Counter(), Counter(), [], []
    failed = 0
    with Path(path).open(encoding='utf-8') as stream:
        for index, line in enumerate(stream):
            record = json.loads(line, parse_constant=reject_constant)
            require(set(record) == {'frame_index', 'timestamp_s', 'detections'}, f'Frame {index}: fields')
            require(type(record['frame_index']) is int and record['frame_index'] == index,
                    f'Frame {index}: non-sequential index')
            require(finite_number(record['timestamp_s']) and abs(record['timestamp_s'] - index / fps) < 1e-9,
                    f'Frame {index}: timestamp')
            require(isinstance(record['detections'], list), f'Frame {index}: detections must be an array')
            counts.append(len(record['detections']))
            for det in record['detections']:
                require(set(det) == {'color', 'pattern', 'confidence', 'corners_px',
                                     'corners_camera_m', 'reprojection_error_px'}, f'Frame {index}: detection fields')
                require(det['color'] in COLORS and det['pattern'] in PATTERNS, f'Frame {index}: class mapping')
                require(finite_number(det['confidence']) and 0 <= det['confidence'] <= 1,
                        f'Frame {index}: confidence')
                observed = np.asarray(det['corners_px'], dtype=float)
                require(observed.shape == (4, 2) and np.isfinite(observed).all(), f'Frame {index}: pixel corners')
                colors[det['color']] += 1
                patterns[det['pattern']] += 1
                if det['corners_camera_m'] is None:
                    require(det['reprojection_error_px'] is None, f'Frame {index}: null pair')
                    failed += 1
                    continue
                points = np.asarray(det['corners_camera_m'], dtype=float)
                require(points.shape == (4, 3) and np.isfinite(points).all() and (points[:, 2] > 0).all(),
                        f'Frame {index}: positive-depth finite 3D corners')
                width = 0.230 if det['pattern'] in ('1', 'base_big') else 0.135
                edges = np.roll(points, -1, axis=0) - points
                require(np.allclose(np.linalg.norm(edges, axis=1), [0.056, width, 0.056, width], atol=1e-7, rtol=0),
                        f'Frame {index}: physical size/corner order')
                require(np.allclose(np.sum(edges * np.roll(edges, 1, axis=0), axis=1), 0, atol=1e-9),
                        f'Frame {index}: rectangle orthogonality')
                projected, _ = cv2.projectPoints(points, np.zeros(3), np.zeros(3),
                                                 calibration.matrix, calibration.distortion)
                rms = float(np.sqrt(np.sum((projected.reshape(4, 2) - observed) ** 2) / 4))
                reported = det['reprojection_error_px']
                require(finite_number(reported) and reported >= 0 and abs(rms - reported) < 1e-7,
                        f'Frame {index}: reprojection RMS')
                errors.append(rms)
                depths.extend(points[:, 2].tolist())
    require(len(counts) == expected_frames, f'Expected {expected_frames} frames, got {len(counts)}')
    return {
        'file': str(path), 'frames': len(counts), 'fps': fps, 'detections': sum(counts),
        'frames_with_detections': sum(n > 0 for n in counts), 'empty_frames': counts.count(0),
        'valid_poses': len(errors), 'failed_poses': failed,
        'colors': dict(sorted(colors.items())), 'patterns': dict(sorted(patterns.items())),
        'reprojection_error_px': ({'median': float(np.median(errors)), 'p95': float(np.percentile(errors, 95)),
                                   'max': max(errors)} if errors else None),
        'depth_range_m': [min(depths), max(depths)] if depths else None,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('jsonl', type=Path, nargs='+')
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--expected-frames', type=int, default=360)
    parser.add_argument('--fps', type=float, default=30.0)
    args = parser.parse_args()
    calibration = Calibration.load(args.calibration)
    try:
        reports = [validate(path, calibration, args.expected_frames, args.fps) for path in args.jsonl]
    except (ValueError, KeyError, TypeError, OSError) as error:
        parser.exit(1, f'Validation failed: {error}\n')
    print(json.dumps(reports, ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
