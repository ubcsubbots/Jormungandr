#Created By: Logan Fillo
#Created On: 2019-03-14

"""
A uwsim simulation testing framework, based on the Java JUnit testing
framework as well as the python unittest testing framework

The pysimtest provides a pythonic solution for testing ai algorithms
using the uwism simulation environment. The following is a simple
example of it's use

    import pysimtest

    class MyTestSuite(pysimtest.SimTestSuite):

        @pysimtest.setup
        def setup(self):
            self.set_scene('gate')
            self.set_test_timout(60)

        @pysimtest.test(run=True)
        def test_position(self)
            self.set_vehicle_position(0,0,0)

        @pysimtest.test(run=False)
        def test_rotation(self):
            self.set_vehicle_rotation(0,0,0)

    if __name__ == '__main__':
        pysimtest.main()

There are two ways to run a test suite, either by navigating
to the directory containing pysimtest and then typing
'python -m pysimtest my_test_suite.py' into a terminal, or
by sourcing pysimtest.sh and then typing 'pysimtest my_test_suite'
"""

name = "pysimtest"

__all__ = (["SimTestSuite",
            "setup",
            "test",
            "main" ])

from _pysimtest.testsuite import SimTestSuite
from _pysimtest.testsuite import setup
from _pysimtest.testsuite import test
from _pysimtest.main import main

# Run program
if __name__ == "__main__":
    import sys
    main(sys.argv)
