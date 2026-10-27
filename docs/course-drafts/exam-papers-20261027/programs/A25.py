"""Select a sampled path that stays outside Gaussian social spaces."""

import numpy as np


def select_social_path(paths, durations, means, sigmas, weights, threshold, rho):
    paths = np.asarray(paths, dtype=float)
    durations = np.asarray(durations, dtype=float)
    means = np.asarray(means, dtype=float)
    sigmas = np.asarray(sigmas, dtype=float)
    weights = np.asarray(weights, dtype=float)
    best = None

    for index, path in enumerate(paths):
        delta = (path[:, None, :] - means[None, :, :]) / sigmas[None, :, :]
        exponent = -0.5 * np.sum(delta * delta, axis=2)
        density = np.sum(
            weights[None, :] * np.exp(exponent)
            / (2.0 * np.pi * np.prod(sigmas, axis=1))[None, :],
            axis=1,
        )
        if np.any(density > threshold):
            continue

        second_diff = path[2:] - 2.0 * path[1:-1] + path[:-2]
        smooth_cost = np.sum(second_diff * second_diff)
        cost = float(smooth_cost + rho * durations[index])
        if best is None or cost < best[1]:
            best = (index, cost)

    return best


if __name__ == "__main__":
    paths = np.array([
        [[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
        [[2.0, 0.0], [2.0, 1.0], [2.0, 2.0]],
        [[3.0, 0.0], [2.0, 0.0], [3.0, 0.0]],
    ])
    result = select_social_path(
        paths=paths,
        durations=[3.0, 4.0, 2.0],
        means=[[0.0, 0.0]],
        sigmas=[[1.0, 1.0]],
        weights=[1.0],
        threshold=0.1,
        rho=1.0,
    )
    assert result == (1, 4.0), result
    print(result)
