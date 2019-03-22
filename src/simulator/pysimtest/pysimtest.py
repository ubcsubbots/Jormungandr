#Created By: Logan Fillo
#Created On: 2019-03-14

# see README.md

name = "pysimtest"

__all__ = (["SimTestSuite",
            "forall",
            "test",])

from _pysimtest.suite import SimTestSuite
from _pysimtest.suite import forall
from _pysimtest.suite import test
from _pysimtest.main  import main

# Run program
if __name__ == "__main__":
    import sys
    main(sys.argv)
