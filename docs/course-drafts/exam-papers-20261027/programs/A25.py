"""Compute weighted trajectory costs."""

import numpy as np


def social_trajectory_objective(smooth_costs, durations, weights, rho):
    smooth_costs = np.asarray(smooth_costs, dtype=float)
    durations = np.asarray(durations, dtype=float)
    weights = np.asarray(weights, dtype=float)
    path_costs = smooth_costs + rho * durations
    weighted_average = float(np.sum(weights * path_costs))
    return {"path_costs": path_costs, "weighted_average": weighted_average}


def demo():
    result = social_trajectory_objective(
        smooth_costs=[2.0, 1.0, 3.0],
        durations=[4.0, 2.0, 1.0],
        weights=[0.5, 0.3, 0.2],
        rho=0.5,
    )
    assert np.array_equal(result["path_costs"], [4.0, 2.0, 3.5])
    assert np.isclose(result["weighted_average"], 3.3)


if __name__ == "__main__":
    demo()
