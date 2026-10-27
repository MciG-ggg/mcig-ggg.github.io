import numpy as np


def lane_center(mask):
    """Fit bottom-row lane positions from a binary H-by-W mask."""
    if not isinstance(mask, np.ndarray) or mask.ndim != 2:
        raise ValueError("mask must be a two-dimensional NumPy array")
    height, width = mask.shape
    if height < 5 or width < 5:
        raise ValueError("mask dimensions must be at least 5 by 5")

    center = (width - 1) / 2
    ys, xs = np.nonzero(mask[height // 2:, :])
    ys = ys + height // 2
    estimates = []
    for selected in (xs < center, xs > center):
        if np.unique(ys[selected]).size < 2:
            return None
        slope, intercept = np.polyfit(ys[selected], xs[selected], 1)
        estimates.append(float(slope * (height - 1) + intercept))

    left_x, right_x = estimates
    tolerance = 1e-8
    if not (-tolerance <= left_x < right_x <= width - 1 + tolerance):
        return None
    left_x = max(0.0, min(width - 1.0, left_x))
    right_x = max(0.0, min(width - 1.0, right_x))
    if left_x >= right_x:
        return None

    lane_center_x = (left_x + right_x) / 2
    return {
        "left_x": left_x,
        "right_x": right_x,
        "center_x": lane_center_x,
        "offset_px": center - lane_center_x,
    }


def demo():
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


if __name__ == "__main__":
    demo()
