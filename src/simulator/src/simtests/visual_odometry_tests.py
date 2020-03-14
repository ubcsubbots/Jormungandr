#Created By: Logan Filloe
#Created On: 2020-03-05

"""
Test suite used for visual odometry tests
"""

import pysimtest

class ExampleTestSuite(pysimtest.SimTestSuite):

    @pysimtest.forall
    def forall(self):
        self.use_ai(False)
        self.add_seafloor()
        self.set_timeout(500)

    @pysimtest.test(run=True)
    def test_approach_gate(self):
        self.add_gate(5, 0, 2,0,0,3.14/2)
        self.set_vehicle_position(0,0,1)
        self.set_vehicle_rotation(0, 0, 0)