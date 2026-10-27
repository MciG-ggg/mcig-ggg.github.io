import math


def control_metrics(times, speeds, target, band):
    errors = [target - speed for speed in speeds]
    rmse = math.sqrt(sum(error ** 2 for error in errors) / len(errors))
    max_abs_error = max(abs(error) for error in errors)
    overshoot_pct = 100 * max(0.0, max(speeds) - target) / target
    last_outside = max((i for i, error in enumerate(errors)
                        if abs(error) > band), default=-1)
    start = last_outside + 1
    stable_time = times[start] if len(times) - start >= 3 else None
    return {
        "rmse": rmse, "max_abs_error": max_abs_error,
        "overshoot_pct": overshoot_pct, "stable_time": stable_time,
    }
