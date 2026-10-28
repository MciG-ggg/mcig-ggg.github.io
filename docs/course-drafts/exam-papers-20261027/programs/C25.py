"""Compute the MPC tracking error over a prediction horizon."""

import numpy as np


def mpc_tracking_cost(predicted, reference):
    predicted = np.asarray(predicted, dtype=float)
    reference = np.asarray(reference, dtype=float)
    return float(np.sum((predicted - reference) ** 2))


def demo():
    predicted = np.array([[1, 2], [3, 4]])
    reference = np.array([[0, 2], [1, 4]])
    assert mpc_tracking_cost(predicted, reference) == 5.0


if __name__ == "__main__":
    demo()
