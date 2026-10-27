from collections.abc import Sequence
from math import fsum, isfinite
from numbers import Real
def evaluate_partition(cumulative, cuts):
    if not isinstance(cumulative, Sequence) or isinstance(cumulative, (str, bytes)):
        raise ValueError("cumulative must be a numeric sequence")
    if len(cumulative) < 2 or any(
        not isinstance(value, Real) or isinstance(value, bool) or not isfinite(value)
        for value in cumulative
    ):
        raise ValueError("cumulative must contain at least two finite numbers")
    if cumulative[0] != 0 or any(right <= left for left, right in zip(cumulative, cumulative[1:])):
        raise ValueError("cumulative must start at zero and increase strictly")
    if not isinstance(cuts, Sequence) or isinstance(cuts, (str, bytes)):
        raise ValueError("cuts must be an index sequence")
    if any(not isinstance(index, int) or isinstance(index, bool) for index in cuts):
        raise ValueError("cut indices must be integers")
    if any(index <= 0 or index >= len(cumulative) - 1 for index in cuts):
        raise ValueError("cut indices must be inside the path")
    if any(right <= left for left, right in zip(cuts, cuts[1:])):
        raise ValueError("cut indices must increase strictly")

    boundaries = (0, *cuts, len(cumulative) - 1)
    lengths = [cumulative[right] - cumulative[left] for left, right in zip(boundaries, boundaries[1:])]
    mean_length = fsum(lengths) / len(lengths)
    variance = fsum((length - mean_length) ** 2 for length in lengths) / len(lengths)
    return {"lengths": lengths, "variance": variance}


if __name__ == "__main__":
    result = evaluate_partition([0, 4, 9, 15, 22, 30], [2, 4])
    assert result["lengths"] == [9, 13, 8]
    assert abs(result["variance"] - 14 / 3) < 1e-12

    try:
        evaluate_partition([0, 4, 9], [2, 1])
    except ValueError:
        pass
    else:
        raise AssertionError("unsorted cuts must raise ValueError")
