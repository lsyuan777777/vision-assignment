"""Deterministic tests of the assignment's numerical and failure contracts."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from detection.detect import (
    Calibration, COLORS, PATTERNS, Letterbox, decode, estimate_pose,
    object_points, preprocess,
)


class PreprocessTests(unittest.TestCase):
    def test_rgb_float16_normalization_and_padding(self):
        bgr = np.zeros((1080, 1440, 3), dtype=np.uint8)
        bgr[:] = [0, 128, 255]
        tensor, transform = preprocess(bgr)
        self.assertEqual(tensor.shape, (1, 3, 640, 640))
        self.assertEqual(tensor.dtype, np.float16)
        self.assertTrue(tensor.flags.c_contiguous)
        np.testing.assert_allclose(tensor[0, :, 300, 300], [1, 128 / 255, 0], atol=3e-4)
        np.testing.assert_allclose(tensor[0, :, 0, 0], [114 / 255] * 3, atol=3e-4)
        np.testing.assert_allclose(transform.restore([[0, 80], [640, 560]]),
                                   [[0, 0], [1440, 1080]], atol=1e-10)

    def test_portrait_and_odd_sized_roundtrip_preserves_order(self):
        for height, width in ((1001, 777), (777, 1001), (640, 640)):
            with self.subTest(shape=(height, width)):
                _, transform = preprocess(np.zeros((height, width, 3), np.uint8))
                points = np.array([[90, 200], [30, 30], [180, 20], [220, 190]], float)
                encoded = points * [transform.scale_x, transform.scale_y] + [transform.left, transform.top]
                np.testing.assert_allclose(transform.restore(encoded), points, atol=1e-10)


class DecodeTests(unittest.TestCase):
    def setUp(self):
        self.transform = Letterbox(1, 1, 0, 0)

    @staticmethod
    def row(color=0, pattern=0, logit=2.0, shift=0):
        row = np.full(22, -10.0, dtype=np.float32)
        row[:8] = np.array([[10, 10], [10, 30], [40, 30], [40, 10]]).ravel() + shift
        row[8], row[9 + color], row[13 + pattern] = logit, 1.0, 2.0
        return row

    def test_all_color_and_pattern_mappings(self):
        rows = np.array([self.row(i % 4, i, shift=i * 100) for i in range(9)])
        detections = decode(rows[None], self.transform)
        self.assertEqual([x['pattern'] for x in detections], list(PATTERNS))
        self.assertEqual([x['color'] for x in detections], [COLORS[i % 4] for i in range(9)])

    def test_objectness_only_and_class_agnostic_nms(self):
        lower = self.row(color=1, logit=1.0)
        lower[9:22] *= 100  # Class logits must not determine confidence or NMS ranking.
        higher = self.row(color=2, logit=2.0)
        far = self.row(color=3, pattern=8, logit=0.0, shift=100)
        results = decode(np.array([[lower, higher, far]]), self.transform)
        self.assertEqual(len(results), 2)
        self.assertEqual(results[0]['color'], 'neutral')
        self.assertAlmostEqual(results[0]['confidence'], 1 / (1 + np.exp(-2)), places=12)
        self.assertEqual(results[1]['confidence'], 0.5)
        self.assertEqual(results[1]['pattern'], 'base_big')

    def test_no_corner_sorting_or_clipping(self):
        row = self.row()
        row[:8] = [25, -10, -10, 25, 25, 60, 60, 25]
        result = decode(row[None, None], self.transform)[0]
        np.testing.assert_array_equal(result['corners_px'], row[:8].reshape(4, 2))

    def test_empty_invalid_and_extreme_logits(self):
        low, high, invalid = self.row(logit=-1000), self.row(logit=1000), self.row()
        invalid[0] = np.nan
        with np.errstate(over='raise', invalid='raise'):
            self.assertEqual(decode(low[None, None], self.transform), [])
            result = decode(np.array([[low, high, invalid]]), self.transform)
        self.assertEqual(len(result), 1)
        self.assertEqual(result[0]['confidence'], 1.0)
        json.dumps(result, allow_nan=False)

    def test_bad_output_shape_fails_clearly(self):
        with self.assertRaises(ValueError):
            decode(np.zeros((1, 22, 25200)), self.transform)


class PoseTests(unittest.TestCase):
    def setUp(self):
        self.calibration = Calibration.load(Path(__file__).resolve().parents[1] / 'assets/calibration.yaml')

    def test_known_3d_pose_roundtrip_both_sizes_with_distortion(self):
        rng = np.random.default_rng(2026)
        for pattern, width in (('3', 0.135), ('1', 0.230), ('base_big', 0.230)):
            for _ in range(8):
                # Includes large in-plane rotation so sorting image points would fail.
                rvec = np.array([rng.uniform(-0.5, 0.5), rng.uniform(-0.6, 0.6), rng.uniform(-2.7, 2.7)])
                tvec = np.array([rng.uniform(-0.4, 0.4), rng.uniform(-0.2, 0.2), rng.uniform(1, 7)])
                model = object_points(pattern)
                observed, _ = cv2.projectPoints(model, rvec, tvec, self.calibration.matrix,
                                                self.calibration.distortion)
                points, error = estimate_pose(observed.reshape(4, 2), pattern, self.calibration)
                rotation, _ = cv2.Rodrigues(rvec)
                expected = model @ rotation.T + tvec
                np.testing.assert_allclose(points, expected, atol=1e-5)
                self.assertLess(error, 1e-5)
                self.assertAlmostEqual(np.linalg.norm(np.array(points)[0] - points[3]), width, places=9)

    def test_all_geometry_classes(self):
        for pattern in PATTERNS:
            points = object_points(pattern)
            width = 0.230 if pattern in ('1', 'base_big') else 0.135
            self.assertAlmostEqual(np.linalg.norm(points[0] - points[3]), width)
            self.assertAlmostEqual(np.linalg.norm(points[0] - points[1]), 0.056)

    def test_rms_is_per_corner_euclidean_and_bad_fit_rejected(self):
        observed, _ = cv2.projectPoints(object_points('3'), np.array([0.2, 0.3, 0.5]),
                                        np.array([0.1, 0.2, 2.0]), self.calibration.matrix,
                                        self.calibration.distortion)
        observed = observed.reshape(4, 2) + [[0.6, 0.9], [-0.8, 0.2], [0.4, -0.7], [-0.1, 0.4]]
        points, error = estimate_pose(observed, '3', self.calibration)
        self.assertIsNotNone(points)
        projected, _ = cv2.projectPoints(np.array(points), np.zeros(3), np.zeros(3),
                                        self.calibration.matrix, self.calibration.distortion)
        expected_error = np.sqrt(np.sum((projected.reshape(4, 2) - observed) ** 2) / 4)
        self.assertAlmostEqual(error, expected_error, places=10)
        self.assertEqual(estimate_pose(observed, '3', self.calibration, max_error_px=1e-6), (None, None))

    def test_degenerate_nonfinite_and_solver_failure_return_null(self):
        for points in (np.zeros((4, 2)), np.full((4, 2), np.nan),
                       np.array([[0, 0], [1, 1], [2, 2], [3, 3]])):
            self.assertEqual(estimate_pose(points, '3', self.calibration), (None, None))
        points = np.array([[0, 0], [0, 30], [60, 30], [60, 0]], float)
        with patch('detection.detect.cv2.solvePnPGeneric', side_effect=cv2.error('failed')):
            self.assertEqual(estimate_pose(points, '3', self.calibration), (None, None))

    def test_behind_camera_candidates_rejected(self):
        points = np.array([[0, 0], [0, 30], [60, 30], [60, 0]], float)
        rvec, tvec = np.zeros((3, 1)), np.array([[0.0], [0.0], [-2.0]])
        with patch('detection.detect.cv2.solvePnPGeneric', return_value=(1, [rvec], [tvec], None)), \
             patch('detection.detect.cv2.solvePnPRefineLM', side_effect=cv2.error('failed')):
            self.assertEqual(estimate_pose(points, '3', self.calibration), (None, None))


class CalibrationTests(unittest.TestCase):
    def test_invalid_matrix_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'camera.yaml'
            path.write_text('image_width: 1440\nimage_height: 1080\ncamera_matrix: [[0, 0, 0], [0, 0, 0], [0, 0, 1]]\ndistort_coeffs: [0, 0, 0, 0, 0]\n')
            with self.assertRaises(ValueError):
                Calibration.load(path)


if __name__ == '__main__':
    unittest.main()
