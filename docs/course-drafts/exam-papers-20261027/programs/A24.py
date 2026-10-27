"""Compute row-wise softmax over active entries only."""

import numpy as np


def sparse_softmax(logits, active):
    logits = np.asarray(logits, dtype=float)
    active = np.asarray(active, dtype=bool)
    result = np.zeros_like(logits)
    for row in range(logits.shape[0]):
        mask = active[row]
        if not np.any(mask):
            continue
        values = logits[row, mask]
        exp_values = np.exp(values - np.max(values))
        result[row, mask] = exp_values / np.sum(exp_values)
    return result


if __name__ == "__main__":
    logits = np.array([[1.0, 3.0, 1000.0], [2.0, -1.0, 4.0]])
    active = np.array([[True, True, False], [False, False, False]])
    result = sparse_softmax(logits, active)
    assert np.allclose(result[0], [0.11920292, 0.88079708, 0.0])
    assert np.array_equal(result[1], [0.0, 0.0, 0.0])
    assert np.isclose(result[0].sum(), 1.0)
    print(result)
