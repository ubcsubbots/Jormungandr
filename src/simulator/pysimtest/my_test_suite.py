#Created By: Logan Fillo
#Created On: 2019-03-13

"""
This is an example simulation test suite
created using pysimtest
"""

import pysimtest

class GateTestSuite(pysimtest.SimTestSuite):

    @pysimtest.setup
    def setup(self):
        self.set_scene("gate")
        self.set_timeout(15)

    @pysimtest.test(run=True)
    def test_gate_one(self):
        self.set_vehicle_position(0,0,0)
        self.set_vehicle_rotation(0,0,0)
        self.assert_passes_gate(True)

    @pysimtest.test(run=True)
    def test_gate_two(self):
        self.set_vehicle_position(0,2,0)
        self.set_vehicle_rotation(0,0,0)
        self.assert_passes_gate(False)

class LineTestSuite(pysimtest.SimTestSuite):

    @pysimtest.setup
    def setup(self):
        self.set_scene("gate")
        self.set_timeout(10)

    @pysimtest.test(run=True)
    def test_line_one(self):
        self.set_vehicle_position(1,0,1)
        self.set_vehicle_rotation(0,0,0)
        self.assert_passes_gate(True)
