def analyze_speed(rows, speed_limit, accel_limit, distance_tol):
    accelerations = []
    overspeed = [i for i, row in enumerate(rows)
                 if row[2] > speed_limit]
    excess_accel = []
    inconsistent = []
    for index in range(1, len(rows)):
        t0, s0, v0 = rows[index - 1]
        t1, s1, v1 = rows[index]
        duration = t1 - t0
        if duration <= 0:
            raise ValueError("Timestamps must increase.")
        acceleration = (v1 - v0) / duration
        accelerations.append(acceleration)
        if abs(acceleration) > accel_limit:
            excess_accel.append(index)
        if abs((s1 - s0) - (v0 + v1) * duration / 2) > distance_tol:
            inconsistent.append(index)
    return {
        "max_speed": max(row[2] for row in rows),
        "max_abs_accel": max(abs(a) for a in accelerations),
        "overspeed_indices": overspeed,
        "excess_accel_intervals": excess_accel,
        "inconsistent_intervals": inconsistent,
    }
