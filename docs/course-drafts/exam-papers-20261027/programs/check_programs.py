from A24 import demo as check_a24
from A25 import demo as check_a25
from B24 import demo as check_b24
from B25 import demo as check_b25
from C24 import demo as check_c24
from C25 import demo as check_c25


if __name__ == "__main__":
    for check in (check_a24, check_a25, check_b24, check_b25, check_c24, check_c25):
        check()
    print("All six program checks passed")
