#Created By: Logan Filloe
#Created On: 2020-03-05

"""
Test suite used for visual odometry tests
"""

import pysimtest

class ExampleTestSuite(pysimtest.SimTestSuite):

    @pysimtest.forall
    def forall(self):
        self.add_seafloor()
        self.set_timeout(120)

    @pysimtest.test(run=True)
    def test_approach_gate(self):
        self.add_gate(5, 0, 1,0,0,3.14/2)
        self.set_vehicle_position(0,0,0)
        self.set_vehicle_rotation(0, 0, 0)