#Created By: Logan Fillo
#Created On: 2019-03-20


from _pysimtest import suite
import pytest


"""
Test suite for _pysimtest.suite
"""


class ExampleSuite(suite.SimTestSuite):

    @suite.forall
    def forall(self):
        pass

    @suite.test(run=True)
    def func(self):
        self.use_dynamics(True)
        self.use_dynamics(False)
        self.add_pool()
        self.add_seafloor()
        self.add_pole(0,0,0)
        self.add_gate(0,0,0,0,0,0)
        self.add_path_marker(0,0,0,0,0,0)
        self.set_timeout(0)
        self.set_wave_scale(0)
        self.set_vehicle_position(0,0,0)
        self.set_vehicle_rotation(0,0,0)

class TestSuite:

    # just test that all functions execute
    def test_all(self):
        sim_suite = ExampleSuite()
        assert not sim_suite._in_global_state
        sim_suite.forall()
        assert not sim_suite._in_global_state
        sim_suite.func()
