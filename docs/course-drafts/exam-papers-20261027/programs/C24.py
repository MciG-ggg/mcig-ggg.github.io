import numpy as np


def path_length(points):
    points = np.asarray(points, dtype=float)
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def demo():
    assert path_length(np.array([[0, 0], [3, 0], [3, 4]])) == 7.0
    assert path_length(np.array([[2, 5]])) == 0.0


if __name__ == "__main__":
    demo()
