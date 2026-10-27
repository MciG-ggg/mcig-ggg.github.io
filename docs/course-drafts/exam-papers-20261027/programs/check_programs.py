import math
import unittest

import cv2
import numpy as np

from A24 import lane_candidates
from A25 import choose_path
from B24 import merge_lanes
from B25 import analyze_speed
from C24 import lane_center
from C25 import control_metrics


class ProgramChecks(unittest.TestCase):
    def test_a24_black(self):
        image = np.zeros((200, 200, 3), dtype=np.uint8)

        edges, segments = lane_candidates(image)

        self.assertEqual(edges.shape, (200, 200))
        self.assertEqual(segments.shape, (0, 4))
        self.assertEqual(edges.dtype, np.uint8)
        self.assertTrue(np.issubdtype(segments.dtype, np.integer))
        self.assertFalse(edges.any())

    def test_a24_synthetic_lanes(self):
        image = np.zeros((200, 200, 3), dtype=np.uint8)
        cv2.line(image, (30, 199), (95, 130), (255, 255, 255), 3)
        cv2.line(image, (170, 199), (105, 130), (255, 255, 255), 3)

        edges, segments = lane_candidates(image)

        self.assertTrue(edges.any())
        self.assertGreater(len(segments), 0)
        self.assertEqual(segments.shape[1], 4)
        self.assertFalse(edges[:120].any())

    def test_a24_roi_excludes_upper_edges(self):
        image = np.zeros((200, 200, 3), dtype=np.uint8)
        cv2.line(image, (0, 30), (199, 30), (255, 255, 255), 3)

        edges, segments = lane_candidates(image)

        self.assertFalse(edges.any())
        self.assertEqual(segments.shape, (0, 4))

    def test_a25_sample(self):
        paths = [[0, 0, 0], [0, 0.5, 0], [0, 1, 0], [0, 0.5, 0]]

        result = choose_path(paths, [-1, 0.4, -1], [1, 1, 1])

        self.assertEqual(result, (1, 1.25))

    def test_a25_empty_or_infeasible(self):
        lower, upper = [-1, -1], [1, 1]

        empty = choose_path([], lower, upper)
        infeasible = choose_path([[0, 2]], lower, upper)

        self.assertIsNone(empty)
        self.assertIsNone(infeasible)

    def test_a25_inclusive_bounds_and_tie(self):
        paths = [[-1, -1], [1, 1]]

        result = choose_path(paths, [-1, -1], [1, 1])

        self.assertEqual(result, (0, 2))

    def test_b24_sample(self):
        segments = [(20, 100, 40, 60), (22, 100, 42, 60),
                    (80, 100, 60, 60), (78, 100, 58, 60),
                    (0, 0, 10, 1), (50, 100, 50, 60)]

        result = merge_lanes(segments, 100, 60)

        self.assertEqual(result["left"], (21, 100, 41, 60))
        self.assertEqual(result["right"], (79, 100, 59, 60))

    def test_b24_empty(self):
        segments = []

        result = merge_lanes(segments, 100, 60)

        self.assertEqual(result, {"left": None, "right": None})

    def test_b24_single_side_and_filters(self):
        segments = [(20, 100, 40, 60), (1, 2, 1, 8), (0, 0, 10, 1)]

        result = merge_lanes(segments, 100, 60)

        self.assertEqual(result["left"], (20, 100, 40, 60))
        self.assertIsNone(result["right"])

    def test_b24_threshold_equality(self):
        segments = [(0, 0, 2, 1)]

        result = merge_lanes(segments, 10, 5)

        self.assertEqual(result["right"], (20, 10, 10, 5))

    def test_b25_sample(self):
        rows = [(0, 0, 2), (1, 3, 4), (2, 8, 6), (3, 16, 8)]

        result = analyze_speed(rows, 6, 1.5, 0.2)

        self.assertEqual(result, {
            "max_speed": 8, "max_abs_accel": 2,
            "overspeed_indices": [3],
            "excess_accel_intervals": [1, 2, 3],
            "inconsistent_intervals": [3],
        })

    def test_b25_repeated_time(self):
        rows = [(0, 0, 2), (0, 3, 4)]

        with self.assertRaises(ValueError):
            analyze_speed(rows, 6, 2, 0.2)

    def test_b25_reversed_time(self):
        rows = [(1, 0, 2), (0, 3, 4)]

        with self.assertRaises(ValueError):
            analyze_speed(rows, 6, 2, 0.2)

    def test_b25_nonuniform_time_and_equality(self):
        rows = [(0, 0, 2), (2, 8, 6)]

        result = analyze_speed(rows, 6, 2, 0)

        self.assertEqual(result["max_abs_accel"], 2)
        self.assertEqual(result["overspeed_indices"], [])
        self.assertEqual(result["excess_accel_intervals"], [])
        self.assertEqual(result["inconsistent_intervals"], [])

    def test_b25_braking(self):
        rows = [(0, 0, 6), (1, 4, 2)]

        result = analyze_speed(rows, 6, 3, 0)

        self.assertEqual(result["max_abs_accel"], 4)
        self.assertEqual(result["excess_accel_intervals"], [1])
        self.assertEqual(result["inconsistent_intervals"], [])

    def test_b25_distance_tolerance_equality(self):
        rows = [(0, 0, 2), (1, 2.25, 2)]

        result = analyze_speed(rows, 2, 0, 0.25)

        self.assertEqual(result["inconsistent_intervals"], [])
        self.assertEqual(result["overspeed_indices"], [])
        self.assertEqual(result["excess_accel_intervals"], [])

    def test_c24_sample(self):
        mask = np.zeros((5, 11), dtype=np.uint8)
        mask[2:, 2] = mask[2:, 7] = 1

        result = lane_center(mask)

        self.assertIsNotNone(result)
        np.testing.assert_allclose(list(result.values()), [2, 7, 4.5, 0.5])

    def test_c24_missing_side(self):
        mask = np.zeros((5, 11), dtype=np.uint8)
        mask[2:, 2] = 1

        result = lane_center(mask)

        self.assertIsNone(result)

    def test_c24_one_row_is_insufficient(self):
        mask = np.zeros((5, 11), dtype=np.uint8)
        mask[3, [1, 2, 3, 7, 8, 9]] = 1

        result = lane_center(mask)

        self.assertIsNone(result)

    def test_c24_center_column_is_ignored(self):
        mask = np.zeros((5, 11), dtype=np.uint8)
        mask[2:, 5] = 1

        result = lane_center(mask)

        self.assertIsNone(result)

    def test_c24_original_coordinates_and_image_edge(self):
        mask = np.zeros((6, 15), dtype=np.uint8)
        mask[[3, 4, 5], [2, 1, 0]] = 1
        mask[[3, 4, 5], [11, 12, 13]] = 1

        result = lane_center(mask)

        self.assertIsNotNone(result)
        np.testing.assert_allclose(list(result.values()), [0, 13, 6.5, 0.5], atol=1e-8)

    def test_c24_extrapolation_outside_image(self):
        mask = np.zeros((6, 15), dtype=np.uint8)
        mask[[3, 4], [1, 0]] = 1
        mask[[3, 4], [10, 11]] = 1

        result = lane_center(mask)

        self.assertIsNone(result)

    def test_c25_sample(self):
        times, speeds = [0, 1, 2, 3, 4], [0, 11, 9.8, 10.2, 10.1]

        result = control_metrics(times, speeds, 10, 0.5)

        self.assertAlmostEqual(result["rmse"], math.sqrt(20.218))
        self.assertEqual(result["max_abs_error"], 10)
        self.assertEqual(result["overshoot_pct"], 10)
        self.assertEqual(result["stable_time"], 2)

    def test_c25_never_stable(self):
        times, speeds = [0, 1, 2, 3], [0, 0, 0, 0]

        result = control_metrics(times, speeds, 10, 0.5)

        self.assertIsNone(result["stable_time"])
        self.assertEqual(result["overshoot_pct"], 0)

    def test_c25_reentry_with_short_suffix(self):
        times, speeds = [0, 1, 2, 3, 4], [10, 10, 11, 10, 10]

        result = control_metrics(times, speeds, 10, 0.5)

        self.assertIsNone(result["stable_time"])

    def test_c25_all_inside_and_boundary(self):
        times, speeds = [2, 4, 7], [9.5, 10, 10.5]

        result = control_metrics(times, speeds, 10, 0.5)

        self.assertEqual(result["stable_time"], 2)
        self.assertEqual(result["overshoot_pct"], 5)
        self.assertEqual(result["max_abs_error"], 0.5)

    def test_c25_short_record(self):
        times, speeds = [0, 1], [10, 10]

        result = control_metrics(times, speeds, 10, 0.5)

        self.assertIsNone(result["stable_time"])
        self.assertEqual(result["rmse"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
