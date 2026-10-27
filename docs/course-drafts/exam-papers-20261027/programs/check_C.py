import math

import numpy as np

from C24 import lane_center
from C25 import control_metrics


def check_programs():
    mask = np.zeros((5, 11), dtype=np.uint8)
    mask[2:, 2] = 1
    mask[2:, 7] = 1
    result = lane_center(mask)
    assert result is not None
    np.testing.assert_allclose(
        [result["left_x"], result["right_x"], result["center_x"],
         result["offset_px"]],
        [2.0, 7.0, 4.5, 0.5],
    )
    assert lane_center(np.zeros((5, 11), dtype=np.uint8)) is None

    invalid_mask = np.zeros((4, 11), dtype=np.uint8)
    try:
        lane_center(invalid_mask)
    except ValueError:
        pass
    else:
        raise AssertionError("undersized masks must raise ValueError")

    metrics = control_metrics(
        [0, 1, 2, 3, 4], [0, 11, 9.8, 10.2, 10.1], 10, 0.5,
    )
    assert math.isclose(metrics["rmse"], math.sqrt(20.218))
    assert metrics["max_abs_error"] == 10
    assert metrics["overshoot_pct"] == 10
    assert metrics["stable_time"] == 2
    assert control_metrics([0, 1], [10, 10], 10, 0.5)["stable_time"] is None

    for invalid_times, invalid_speeds in (
        ([0, 0], [10, 10]),
        ([0, 1], [10]),
        ([0, math.nan], [10, 10]),
        ([0, 1], [10, "fast"]),
    ):
        try:
            control_metrics(invalid_times, invalid_speeds, 10, 0.5)
        except ValueError:
            pass
        else:
            raise AssertionError("invalid control records must raise ValueError")


def check_numeric_answers():
    assert math.isclose(0.5 * 2 + 0.2 * (1 + 1 + 2) + 0.1 * (2 - 1), 1.9)
    assert math.isclose(0.4 * (2 - 1) + 0.2 * 2 + 0.1 * (2 - 2 + 0), 0.8)
    assert math.isclose(2 + 0.25 * (-1 + 0.8 * 5 - 2), 2.25)
    posterior = np.array([0.2, 0.5, 0.3]) * np.array([0.2, 0.6, 0.2])
    posterior /= posterior.sum()
    np.testing.assert_allclose(posterior, [0.1, 0.75, 0.15])
    np.testing.assert_allclose(np.roll([0.1, 0.2, 0.3, 0.4], 1),
                               [0.4, 0.1, 0.2, 0.3])
    assert math.isclose(3 + 0.5 * 2 + 0.25 * -1, 3.75)
    assert math.isclose(1.2 / 0.03, 40)
    assert math.isclose((1 + 2) / 2, 1.5)
    assert math.isclose(min(math.sqrt(2**2 + 1**2 + 2**2), 1, 2), 1)
    assert 26 < 30 and not 32 < 30
    prior = np.array([0.4, 0.3, 0.2, 0.1])
    predicted = np.roll(prior, 1)
    likelihood = np.array([0.1, 0.7, 0.1, 0.1])
    posterior = predicted * likelihood
    posterior /= posterior.sum()
    np.testing.assert_allclose(posterior, [1 / 34, 28 / 34, 3 / 34, 2 / 34])
    assert math.isclose(0.6 * 100 + 0.4 * 0, 60)
    assert math.isclose(10 - 8, 2)
    assert math.isclose(min(0.4 * 2, 0.5), 0.5)


if __name__ == "__main__":
    check_programs()
    check_numeric_answers()
    print("C paper checks passed")
