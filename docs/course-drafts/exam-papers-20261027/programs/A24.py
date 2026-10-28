import numpy as np


def nearest_distance(query, points):
    query = np.asarray(query, dtype=float)
    points = np.asarray(points, dtype=float)
    return float(np.min(np.linalg.norm(points - query, axis=1)))


def demo():
    result = nearest_distance([0, 0, 0], [[2, 1, 2], [-1, 0, 0], [0, 0, 2]])
    assert result == 1.0


if __name__ == "__main__":
    demo()
