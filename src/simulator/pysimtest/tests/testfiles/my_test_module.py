#Created By: Logan Fillo
#Created On: 2019-03-13

"""
This is an example simulation test suite
created using pysimtest
"""

import pysimtest

class GateTestSuite(pysimtest.SimTestSuite):

    @pysimtest.forall
    def forall(self):
        # self.set_wave_scale(4)
        self.use_dynamics(True)
        self.add_seafloor()
        self.set_timeout(300)

    @pysimtest.test(run=True)
    def test_gate_one(self):
        # self.add_path_marker(0,0,0, 0, 0, 0)
        self.add_pole(-1,1,1)
        self.add_pole(1,-1,1)
        self.add_pole(1,1,-1)
        self.add_pole(1,1,1)
        self.add_pole(-1,-1,-1)
        self.add_pole(1,-1, -1)
        self.add_pole(-1,-1,1)
        self.add_pole(-1,1,-1)
        self.set_vehicle_position(5, -10, 1)
        self.set_vehicle_rotation(0,0,0)
        self.assert_passes_gate(True)

    @pysimtest.test(run=True)
    def test_gate_two(self):
        self.set_vehicle_position(1,-10,0)
        self.set_vehicle_rotation(0,0,0)
        self.assert_passes_gate(True)

    @pysimtest.test(run=True)
    def test_gate_three(self):
        self.set_vehicle_position(0,0,0)
        self.set_vehicle_rotation(0,0,0)
        self.assert_passes_gate(True)

    @pysimtest.test(run=True)
    def test_gate_four(self):
        self.set_vehicle_position(0,0,3)
        self.set_vehicle_rotation(0,0,0)
        self.assert_passes_gate(True)

    @pysimtest.test(run=True)
    def test_gate_five(self):
        self.set_vehicle_position(0,-2,5)
        self.set_vehicle_rotation(0,0,0)
        # self.assert_passes_gate(False)

class LineTestSuite(pysimtest.SimTestSuite):

    @pysimtest.forall
    def forall(self):
        self.set_wave_scale(5)
        self.set_timeout(120)

    @pysimtest.test(run=True)
    def test_line_one(self):
        self.add_pole(0,0,0)
        self.set_vehicle_position(1,-2,3)
        self.set_vehicle_rotation(4,5,6)
        # self.assert_passes_gate(True)
