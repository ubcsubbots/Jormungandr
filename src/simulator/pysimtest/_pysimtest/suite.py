#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module contains the interface for creating
simulation test suites
"""


import decorators
import constants as const
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

    def use_dynamics(self, is_dynamic):
        """
        If is_dynamic is True, uses dynamics
        for the test, else does not use dynamics

        :param is_dynamic: if test is dynamic or not
        """
        # NOTE: this has not been implemented yet
        self._builder.use_dynamics(is_dynamic, self._in_global_state)

    def add_pool(self):
        """
        Adds a pool to the simulation
        """
        self._builder.add_object("pool", const.POOL_MODEL,
                                 0, 0, 0, 0, 0, 0,
                                 self._in_global_state)

    def add_seafloor(self):
        """
        Adds the seafloor to the simulation
        """
        self._builder.add_object("seafloor", const.SEAFLOOR_MODEL,
                                 0, 0, 0, 0, 0, 0,
                                 self._in_global_state)
    def add_pole(self, x, y, z):
        """
        Adds a pole to the simulation at the given x,y,z
        co-ordinates

        ;param x_pos: pole's x position
        ;param y_pos: pole's y position
        ;param z_pos: pole's z position
        """
        self._builder.add_object("pole", const.POLE_MODEL,
                                  x ,y, z,
                                  0, 0, 0,
                                  self._in_global_state)

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
        self._builder.add_object("gate", const.GATE_MODEL,
                                 x_pos, y_pos, z_pos,
                                 r_rot, p_rot, y_rot,
                                 self._in_global_state)

    def add_path_marker(self, x_pos, y_pos, z_pos, r_rot, p_rot, y_rot):
        """
        Adds a path marker to the simulation at the given x,y,z
        co-ordinates with the given r,p,y rotation

        ;param x_pos: path marker's x position
        ;param y_pos: path marker's y position
        ;param z_pos: path marker's z position
        ;param r_rot: path marker's r rotation
        ;param p_rot: path marker's p rotation
        ;param y_rot: path marker's y rotation
        """
        self._builder.add_object("marker", const.MARKER_MODEL,
                                 x_pos, y_pos, z_pos,
                                 r_rot, p_rot, y_rot,
                                 self._in_global_state)

    def add_dice(self, x_pos, y_pos, z_pos, r_rot, p_rot, y_rot):
        """
        Adds a dice to the simulation at the given x,y,z
        co-ordinates with the given r,p,y rotation

        ;param x_pos: dice's x position
        ;param y_pos: dice's y position
        ;param z_pos: dice's z position
        ;param r_rot: dice's r rotation
        ;param p_rot: dice's p rotation
        ;param y_rot: dice's y rotation
        """
        self._builder.add_object("dice", const.DICE_MODEL,
                                 x_pos, y_pos, z_pos,
                                 r_rot, p_rot, y_rot,
                                 self._in_global_state)

    def set_timeout(self, secs):
        """
        Sets the test timeout length of a test case,
        default is 1 minutes

        :param secs: timeout length in seconds
        """
        self._builder.set_timeout(secs, self._in_global_state)

    def set_wave_scale(self, scale):
        """
        Sets the wave scale for the simulation, default
        is 7. Note that scale is the exponent on the actual
        scale (i.e the actual scale is 1e-7)
        """
        self._builder.set_wave_scale(scale, self._in_global_state)

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

    # Tell loader that forall is a forall method
    forall.is_forall = True
