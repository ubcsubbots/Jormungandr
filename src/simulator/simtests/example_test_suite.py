#Created By: You
#Created On: 0000-00-00

"""
Short description of example test suite
"""

import pysimtest

class ExampleTestSuite(pysimtest.SimTestSuite):

    @pysimtest.forall
    def forall(self):
        self.add_pole(0, -1, 7)
        self.add_path_marker(0,0,0,0,0,0)
        self.add_seafloor()
        self.set_timeout(120)

    @pysimtest.test(run=True)
    def test_example_one(self):
        self.add_pole(0, 1, 7)
        self.set_vehicle_position(5,0,5)
        self.set_vehicle_rotation(0, 0, 3.14)

    @pysimtest.test(run=True)
    def test_example_two(self):
        self.use_dynamics(True)
        self.set_wave_scale(5)
        self.set_vehicle_position(2,-5,5)
