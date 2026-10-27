def merge_lanes(segments, y_bottom, y_top):
    groups = {"left": [], "right": []}
    for x1, y1, x2, y2 in segments:
        if x2 == x1:
            continue
        slope = (y2 - y1) / (x2 - x1)
        if abs(slope) < 0.5:
            continue
        intercept = y1 - slope * x1
        side = "left" if slope < 0 else "right"
        groups[side].append((slope, intercept))
    result = {"left": None, "right": None}
    for side, coefficients in groups.items():
        if not coefficients:
            continue
        slope = sum(pair[0] for pair in coefficients) / len(coefficients)
        intercept = sum(pair[1] for pair in coefficients) / len(coefficients)
        x_bottom = (y_bottom - intercept) / slope
        x_top = (y_top - intercept) / slope
        result[side] = (x_bottom, y_bottom, x_top, y_top)
    return result
