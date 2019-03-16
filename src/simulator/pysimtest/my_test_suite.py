#Created By: Logan Fillo
#Created On: 2019-03-13

"""
This is an example simulation test suite
created using pysimtest
"""

import pysimtest

class MyTestSuite(pysimtest.SimTestSuite):

    @pysimtest.setup
    def setup(self):
        self.set_scene("gate")
        self.set_timeout(15)

    @pysimtest.test(run=True)
    def test_one(self):
        self.set_vehicle_position(0,0,0)
        self.set_vehicle_rotation(0,0,0)
        self.assert_passes_gate(True)

    @pysimtest.test(run=True)
    def test_two(self):
        self.set_vehicle_position(1,1,1)
        self.set_vehicle_rotation(1,1,1)
        self.assert_passes_gate(False)

    @pysimtest.test(run=True)
    def test_three(self):
        self.set_vehicle_position(1,0,1)
        self.set_vehicle_rotation(0,1,0)
        self.assert_passes_gate(True)


if __name__ == "__main__":

    # Temp, this will all be wrapped up in __main__, but
    # you should understand the basic algorithm of how it
    # executes a test suite
    my_test_suite = MyTestSuite()
    my_test_suite.setup()
    my_test_suite.test_one()
    my_test_suite.test_two()
    my_test_suite.test_three()
    tests = my_test_suite._tests
    sim_runner  = pysimtest.SimRunner()
    sim_runner.start()
    for test in tests:
        from pysimtest import constants
        sim_runner.configure_scene(test.test_args[constants.SCENE], test.sim_args)
        sim_runner.configure_timeout(test.test_args[constants.TIMEOUT])
        print(test.name)
        for arg in test.test_args:
            print arg, ": ", test.test_args[arg]
        for arg in test.sim_args:
            print arg, ": ", test.sim_args[arg]
        sim_runner.run_simulation()
    sim_runner.stop()
