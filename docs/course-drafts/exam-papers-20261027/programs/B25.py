"""Select the next node with the ETPP expansion cost."""

import numpy as np


def select_etpp_node(g, information_gains, distances):
    information_gains = np.asarray(information_gains, dtype=float)
    distances = np.asarray(distances, dtype=float)
    costs = g + (1.0 + information_gains) * distances
    index = int(np.argmin(costs))
    return index, float(costs[index])


def demo():
    assert select_etpp_node(4.0, [0.2, 0.5], [3.0, 2.0]) == (1, 7.0)


if __name__ == "__main__":
    demo()
