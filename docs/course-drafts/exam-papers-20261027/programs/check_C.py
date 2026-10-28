import numpy as np

from C24 import path_length
from C25 import mpc_tracking_cost


assert path_length(np.array([[0, 0], [3, 0], [3, 4]])) == 7
assert path_length(np.array([[2, 5]])) == 0
assert mpc_tracking_cost([[1, 2], [3, 4]], [[0, 2], [1, 4]]) == 5.0

print("C paper checks passed")
