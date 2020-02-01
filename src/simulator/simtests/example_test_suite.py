#Created By: You
#Created On: 0000-00-00

"""
Short description of example test suite
"""

import pysimtest

class ExampleTestSuite(pysimtest.SimTestSuite):

    @pysimtest.forall
    def forall(self):
        # self.add_pole(0, -1, 7)
        self.add_seafloor()
        self.set_timeout(120)

    @pysimtest.test(run=True)
    def test_example_one(self):
        self.add_pole(0, 1, 7)
        self.set_vehicle_position(5,0,8)
        self.set_vehicle_rotation(0, 0, 3.14)

    @pysimtest.test(run=True)
    def test_example_two(self):
        self.set_wave_scale(5)
        self.set_vehicle_position(2,-5,5)

    @pysimtest.test(run=True)
    def test_example_three(self):
        # self.set_wave_scale(5)
        self.add_gate(10, 0 ,6, 0, 0, 3.14/2)
        self.set_vehicle_position(4,0,5)
