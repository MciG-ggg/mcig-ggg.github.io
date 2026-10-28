import math


assert math.isclose(math.log(2) / math.log(4), 0.5)
assert math.isclose(4 + (1 + 0.2) * 3, 7.6)
assert math.isclose(4 + (1 + 0.5) * 2, 7.0)
assert math.isclose(1 / (2 * math.pi), 0.15915494309189535)

print("A paper calculation checks passed")
