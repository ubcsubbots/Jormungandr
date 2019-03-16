#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module contains the functionality for creating
simulation test suites
"""


import decorators
import constants
import testcase


def setup(func):
    """
    Decorator for setup. Decorate inital setup
    method with this. All methods inside decorated
    function apply globally to a test suite.
    Calling a method in a test method which has
    already been called in the setup method will
    have no effect

    :param func: setup function
    """
    return decorators.setup(func)

def test(run=False):
    """
    Paramaterized decorator for test. Decorate all
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
        self._builder = testcase.SimTestBuilder()

    def set_scene(self, scene):
        """
        Sets the uwsim scene for which a test case
        will be performed on. A scene can either be
        'custom', referring to an empty scene, or a
        preset scene. Currently the choices for preset
        scenes are: 'gate'.

        :param scene: the uwsim scene for a test case
        """
        self._builder.set_scene(scene, self._in_global_state)

    def set_timeout(self, secs):
        """
        Sets the test timeout length of a test case,
        default is 2 minutes

        :param secs: timeout length in seconds
        """
        self._builder.set_timeout(secs, self._in_global_state)

    def set_vehicle_position(self, x, y, z):
        """
        Sets the vehicles initial position

        :param x: vehicle's x position
        :param y: vehicle's y position
        :param z: vehicle's z position
        """
        self._builder.set_vehicle_position(x ,y ,z, self._in_global_state)

    def set_vehicle_rotation(self, r, p, y):
        """
        Sets the vehicles initial rotation

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
