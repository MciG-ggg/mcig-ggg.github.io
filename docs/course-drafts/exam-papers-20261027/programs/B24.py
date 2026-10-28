import numpy as np


def radius_filter(points, radius, min_neighbors):
    points = np.asarray(points, dtype=float)
    keep = []
    for index, point in enumerate(points):
        distances = np.linalg.norm(points - point, axis=1)
        neighbors = np.count_nonzero((distances <= radius) & (np.arange(len(points)) != index))
        if neighbors >= min_neighbors:
            keep.append(point)
    return np.asarray(keep, dtype=float).reshape(-1, 3)


def demo():
    points = np.array([[0, 0, 0], [0.5, 0, 0], [5, 5, 5], [0, 0.5, 0]])
    result = radius_filter(points, 0.8, 2)
    np.testing.assert_allclose(result, [[0, 0, 0], [0.5, 0, 0], [0, 0.5, 0]])
    assert radius_filter(points, 0.1, 1).shape == (0, 3)


if __name__ == "__main__":
    demo()
