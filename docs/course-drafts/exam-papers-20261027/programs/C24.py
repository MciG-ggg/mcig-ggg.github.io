import numpy as np


def lane_center(mask):
    height, width = mask.shape
    middle = (width - 1) / 2
    ys, xs = np.nonzero(mask[height // 2:, :])
    ys = ys + height // 2
    left = xs < middle
    right = xs > middle
    estimates = []
    for selected in (left, right):
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
    center_x = (left_x + right_x) / 2
    return {
        "left_x": left_x, "right_x": right_x,
        "center_x": center_x, "offset_px": middle - center_x,
    }
