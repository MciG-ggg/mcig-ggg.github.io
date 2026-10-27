from collections.abc import Sequence
from math import isfinite
from numbers import Real
def analyze_bichannel(episodes, lambda_min, c, outcomes):
    if not isinstance(episodes, Sequence) or isinstance(episodes, (str, bytes)):
        raise ValueError("episodes must be an integer sequence")
    if any(not isinstance(episode, int) or isinstance(episode, bool) or episode < 0 for episode in episodes):
        raise ValueError("episodes must contain non-negative integers")
    if (
        not isinstance(lambda_min, Real)
        or isinstance(lambda_min, bool)
        or not isfinite(lambda_min)
        or not 0 <= lambda_min <= 1
    ):
        raise ValueError("lambda_min must be between zero and one")
    if not isinstance(c, int) or isinstance(c, bool) or c <= 0:
        raise ValueError("c must be a positive integer")
    if not isinstance(outcomes, Sequence) or isinstance(outcomes, (str, bytes)) or not outcomes:
        raise ValueError("outcomes must be a non-empty sequence")

    for row in outcomes:
        if not isinstance(row, Sequence) or isinstance(row, (str, bytes)) or len(row) != 4:
            raise ValueError("each outcome must contain four values")
        weight, goal_rate, intercept_rate, average_reward = row
        if any(
            not isinstance(value, Real) or isinstance(value, bool) or not isfinite(value)
            for value in row
        ):
            raise ValueError("outcome values must be finite numbers")
        if not 0 <= weight <= 1 or not 0 <= goal_rate <= 1 or not 0 <= intercept_rate <= 1:
            raise ValueError("weights and rates must be between zero and one")

    weights = [
        lambda_min if episode >= c else 1 - (1 - lambda_min) * (episode / c)
        for episode in episodes
    ]
    best = outcomes[0]
    for row in outcomes[1:]:
        if row[3] < best[3] or (row[3] == best[3] and row[1] <= best[1]):
            continue
        best = row
    return {
        "weights": weights,
        "best_lambda_min": best[0],
        "best_goal_rate": best[1],
        "best_intercept_rate": best[2],
        "best_average_reward": best[3],
    }


if __name__ == "__main__":
    sample_outcomes = [
        (0.5, 0.751, 0.218, 5.815),
        (0.6, 0.816, 0.153, 6.356),
        (0.7, 0.859, 0.118, 6.506),
        (0.8, 0.910, 0.067, 6.727),
        (0.9, 0.876, 0.091, 6.659),
    ]
    result = analyze_bichannel([250, 1000, 1200], 0.6, 1000, sample_outcomes)
    assert result["weights"] == [0.9, 0.6, 0.6]
    assert result["best_lambda_min"] == 0.8
    assert result["best_goal_rate"] == 0.910
    assert result["best_intercept_rate"] == 0.067
    assert result["best_average_reward"] == 6.727

    tie_result = analyze_bichannel(
        [], 0.5, 10, [(0.4, 0.7, 0.1, 2.0), (0.6, 0.8, 0.2, 2.0)]
    )
    assert tie_result["best_lambda_min"] == 0.6

    try:
        analyze_bichannel([1], 0.5, 10, [])
    except ValueError:
        pass
    else:
        raise AssertionError("empty outcomes must raise ValueError")
