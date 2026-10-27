import math
from numbers import Real


def control_metrics(times, speeds, target, band):
    try:
        times, speeds = list(times), list(speeds)
    except TypeError as error:
        raise ValueError("times and speeds must be numeric sequences") from error
    if len(times) != len(speeds) or not times:
        raise ValueError("times and speeds must have the same nonzero length")

    values = [*times, *speeds, target, band]
    if not all(isinstance(value, Real) and not isinstance(value, bool)
               and math.isfinite(value) for value in values):
        raise ValueError("all inputs must be finite real numbers")
    if any(later <= earlier for earlier, later in zip(times, times[1:])):
        raise ValueError("times must be strictly increasing")
    if target <= 0 or band < 0:
        raise ValueError("target must be positive and band must be nonnegative")

    errors = [target - speed for speed in speeds]
    rmse = math.sqrt(sum(error ** 2 for error in errors) / len(errors))
    max_abs_error = max(abs(error) for error in errors)
    overshoot_pct = 100 * max(0.0, max(speeds) - target) / target

    stable_time = None
    suffix_inside = True
    for index in range(len(errors) - 1, -1, -1):
        suffix_inside = suffix_inside and abs(errors[index]) <= band
        if suffix_inside and len(errors) - index >= 3:
            stable_time = times[index]

    return {
        "rmse": rmse,
        "max_abs_error": max_abs_error,
        "overshoot_pct": overshoot_pct,
        "stable_time": stable_time,
    }


def demo():
    result = control_metrics([0, 1, 2, 3, 4], [0, 11, 9.8, 10.2, 10.1], 10, 0.5)
    assert round(result["rmse"], 3) == 4.496
    assert result["max_abs_error"] == 10
    assert result["overshoot_pct"] == 10
    assert result["stable_time"] == 2
    assert control_metrics([0, 1], [10, 10], 10, 0.5)["stable_time"] is None
    try:
        control_metrics([0, 0], [10, 10], 10, 0.5)
    except ValueError:
        pass
    else:
        raise AssertionError("non-increasing times must raise ValueError")


if __name__ == "__main__":
    demo()
