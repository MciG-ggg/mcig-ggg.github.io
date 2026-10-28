"""Calculate minimum ADE and FDE across candidate trajectories."""

import numpy as np


def trajectory_errors(predicted, target):
    predicted = np.asarray(predicted, dtype=float)
    target = np.asarray(target, dtype=float)
    distances = np.linalg.norm(predicted - target[None, :, :], axis=2)
    ade = float(np.min(np.mean(distances, axis=1)))
    fde = float(np.min(distances[:, -1]))
    return ade, fde


def demo():
    predicted = [
        [[0.0, 0.0], [2.0, 0.0]],
        [[0.0, 1.0], [1.0, 2.0]],
    ]
    assert trajectory_errors(predicted, [[0.0, 0.0], [1.0, 0.0]]) == (0.5, 1.0)


if __name__ == "__main__":
    demo()
