#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module provides a framework for testing algorithms
using the uwsim simulation environment.
"""


import decorators
import constants
import simrun
import importlib

def main():
    mod = importlib.import_module("__main__")
    #print (mod)
    #print(SimTestSuite.__subclasses__())
    print(mod.__dict__)


def setup(func):
    """
    Decorator for setup. Decorate inital
    setup method with this

    :param func: setup function
    """
    return decorators.setup(func)

def test(run=False):
    """
    Paramaterized decorator for test. Decorate all
    test methods with this, including an argument
    specifying if you want it to run or not

    :param run: True if you want the  decorated test
                to run, False otherwise
    """
    def _decorator(func):
        return decorators.test(func, run)
    return _decorator


class SimTestSuite:
    """
    A class for creating simulation test
    suites. All test suites must extend this
    class and follow the conventions given in
    my_test_suit.py
    """
    def __init__(self):
        """
        Construct a SimTestSuite with a list of
        simulation test cases, a simulation test
        builder, and a simulation test runner
        """
        self._tests = []
        self._sim_test_builder = _SimTestBuilder()
        self._sim_test_runner  = _SimTestRunner()

    def main(self):
        self.__run_test_suite()

    def set_scene(self, scene):
        """
        Sets the uwsim scene for which this test suite
        will be performed on

        :param scene: the uwsim scene for this test suite,
                      default is 'empty', a valid argument
                      is one of: 'gate', 'line', 'empty'
        """
        self._sim_test_runner._SimTestRunner__set_scene(scene)

    def set_timeout(self, secs):
        """
        Sets the test timeout length

        :param secs: timeout length in seconds
        """
        self._sim_test_runner._SimTestRunner__set_timeout(secs)

    def set_vehicle_position(self, x, y, z):
        """
        Sets the vehicles initial position

        :param x: vehicle's x position
        :param y: vehicle's y position
        :param z: vehicle's z position
        """
        self._sim_test_builder._SimTestBuilder__set_vehicle_position(x ,y ,z)

    def set_vehicle_rotation(self, r, p, y):
        """
        Sets the vehicles initial rotation

        :param r: vehicle's r position
        :param p: vehicle's p position
        :param y: vehicle's y position
        """
        self._sim_test_builder._SimTestBuilder__set_vehicle_rotation(r ,p ,y)

    def assert_passes_gate(self, should_pass):
        """
        Determines whether or not a gate test passes

        :param should_pass: a boolean value of whether the gate
                            test is expected to pass or not
        """
        pass #TODO

    def __create_new_test(self, test_name):
        self._sim_test_builder._SimTestBuilder__new_test_case(test_name)

    def __add_test(self):
        self._tests.append(self._sim_test_builder.get_current_test())

    def __run_test_suite(self):
        self._sim_test_runner._SimTestRunner__run_test_suite(self._tests)


class _SimTestBuilder:
    """
    A private class for building
    simulation test cases
    """
    def __init__(self):
        """
        Construct a SimTestBuilder with a
        current test (initally null)
        """
        self._current_test = 0

    def get_current_test(self):
        """
        Returns the current test case
        """
        return self._current_test

    def __set_vehicle_position(self, x, y, z):
        self._current_test._SimTestCase__set_vehicle_position(x,y,z)

    def __set_vehicle_rotation(self, r, p, y):
        self._current_test._SimTestCase__set_vehicle_rotation(r, p ,y)

    def __new_test_case(self, test_name):
        self._current_test = _SimTestCase(test_name)


class _SimTestRunner:
    """
    A private class for running
    a simulation test suite
    """
    def __init__(self):
        """
        Constructs a _SimTestRunner with a test timeout
        length, a uwsim scene, and a simulation runner
        """
        self._timeout = 0
        self._scene = "default"
        self._runner = simrun.SimRunner()

    def __set_scene(self, scene):
        self._scene = scene

    def __set_timeout(self, secs):
        self._timeout = secs

    def __run_test_suite(self, tests):
        self._runner.configure_scene(self._scene)
        self._runner.configure_timeout(self._timeout)
        self._runner.configure_vehicle_ai()
        for test in tests:
            self.__run_test(test)
        self._runner.exit()

    def __run_test(self, test):
        self._runner.run_simulation(test.get_name(), test.get_args())


class _SimTestCase:
    """
    A private class representing a
    simulation test case
    """
    def __init__(self, test_name):
        """
        Constructs a _SimTestCase with a name and
        a dictionary of it's simulation arguments
        """
        self._test_name = test_name

        self._sim_args = dict()

        self._sim_args["vehicle_x_pos"] = constants.DEFAULT_VEHICLE_X_POS
        self._sim_args["vehicle_y_pos"] = constants.DEFAULT_VEHICLE_Y_POS
        self._sim_args["vehicle_z_pos"] = constants.DEFAULT_VEHICLE_Z_POS

        self._sim_args["vehicle_r_rot"] = constants.DEFAULT_VEHICLE_R_ROT
        self._sim_args["vehicle_p_rot"] = constants.DEFAULT_VEHICLE_P_ROT
        self._sim_args["vehicle_y_rot"] = constants.DEFAULT_VEHICLE_Y_ROT

    def get_name(self):
        """
        Returns self's test name
        """
        return self._test_name

    def get_args(self):
        """
        Returns a dictionary of self's
        simulation arguments
        """
        return self._sim_args

    def __set_vehicle_position(self, x, y, z):
        self._sim_args["vehicle_x_pos"] = x
        self._sim_args["vehicle_y_pos"] = y
        self._sim_args["vehicle_z_pos"] = z

    def __set_vehicle_rotation(self, r, p, y):
        self._sim_args["vehicle_r_rot"] = r
        self._sim_args["vehicle_p_rot"] = p
        self._sim_args["vehicle_y_rot"] = y
