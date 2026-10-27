import cv2
import numpy as np


def lane_candidates(bgr):
    height, width = bgr.shape[:2]
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    smooth = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(smooth, 50, 150)
    mask = np.zeros_like(edges)
    vertices = np.array([
        [0, height - 1],
        [width // 2, int(0.6 * height)],
        [width - 1, height - 1],
    ], dtype=np.int32)
    cv2.fillPoly(mask, [vertices], 255)
    roi_edges = cv2.bitwise_and(edges, mask)
    lines = cv2.HoughLinesP(
        roi_edges, 1, np.pi / 180, 20,
        minLineLength=20, maxLineGap=10,
    )
    if lines is None:
        return roi_edges, np.empty((0, 4), dtype=np.int32)
    return roi_edges, lines.reshape(-1, 4)
