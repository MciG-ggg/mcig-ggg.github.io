def choose_path(paths, lower, upper):
    best = None
    for index, path in enumerate(paths):
        if not all(lo <= value <= hi
                   for value, lo, hi in zip(path, lower, upper)):
            continue
        cost = sum(value * value for value in path)
        cost += 2 * sum((right - left) ** 2
                        for left, right in zip(path, path[1:]))
        if best is None or cost < best[1]:
            best = (index, cost)
    return best
