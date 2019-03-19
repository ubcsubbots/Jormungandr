#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module contains the interface for creating
simulation test suites
"""


import decorators
import constants
import case



def forall(func):
    """
    Decorator for forall. Decorate inital forall
    method with this. All methods inside decorated
    function apply globally to a test suite.
    Calling a method in a test method which has
    already been called in the forall method will
    have no effect

    :param func: forall method
    """
    return decorators.forall(func)

def test(run=True):
    """
    Paramaterized decorator for tests. Decorate all
    test methods with this, including the run argument
    specifying if you want the test to run or not

    :param run: True if you want the  decorated test
                to run, False otherwise
    """
    def _decorator(func):
        return decorators.test(func, run)
    return _decorator


class SimTestSuite(object):
    """
    A class for creating simulation test suites.
    """
    def __init__(self):
        """
        Construct a SimTestSuite
        """
        self._in_global_state = False

        self._tests = []
        self._builder = case.SimTestBuilder()

    def forall(self):
        """
        Abstract forall method. Use this to set test
        parameters for all tests in the suite. Remember
        to decorate this with forall decorater
        """
        pass

    def add_gate(self, x_pos, y_pos, z_pos, r_rot, p_rot, y_rot):
        """
        Adds a gate to the simulation at the given x,y,z
        co-ordinates with the given r,p,y rotation

        ;param x_pos: gate's x position
        ;param y_pos: gate's y position
        ;param z_pos: gate's z position
        ;param r_rot: gate's r rotation
        ;param p_rot: gate's p rotation
        ;param y_rot: gate's y rotation
        """
        self._builder.add_gate(x_pos, y_pos, z_pos,
                               r_rot, p_rot, y_rot, self._in_global_state)

    def set_timeout(self, secs):
        """
        Sets the test timeout length of a test case,
        default is 2 minutes

        :param secs: timeout length in seconds
        """
        self._builder.set_timeout(secs, self._in_global_state)

    def set_vehicle_position(self, x, y, z):
        """
        Sets the vehicle's initial position

        :param x: vehicle's x position
        :param y: vehicle's y position
        :param z: vehicle's z position
        """
        self._builder.set_vehicle_position(x ,y ,z, self._in_global_state)

    def set_vehicle_rotation(self, r, p, y):
        """
        Sets the vehicle's initial rotation

        :param r: vehicle's r position
        :param p: vehicle's p position
        :param y: vehicle's y position
        """
        self._builder.set_vehicle_rotation(r ,p ,y, self._in_global_state)

    def assert_passes_gate(self, should_pass):
        """
        Determines whether or not a gate test passes

        :param should_pass: a boolean value of whether the gate
                            test is expected to pass or not
        """
        pass #TODO

    def _new_test(self, test_name):
        self._builder.build_test_case(test_name)

    def _add_test(self):
        self._tests.append(self._builder.get_result())
