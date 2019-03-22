#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module contains the functionality
for creating test cases
"""

import sys

import constants as const
import vehicle
import scene
import object


class SimTestCase:
    """
    A class representing a simulation test case
    """
    def __init__(self, name):
        """
        Constructs a SimTestCase
        """
        self.name       = name
        self.is_dynamic = const.DEFAULT_IS_DYNAMIC
        self.timeout    = const.DEFAULT_TIMEOUT
        self.vehicle    = None
        self.scene      = None
        self.objects    = None


class SimTestBuilder:
    """
    A helper class for building simulation tests
    """
    def __init__(self):
        """
        Construct a SimTestBuilder
        """
        self._curr_test         = None
        self._curr_vhcl         = None
        self._curr_scene        = None
        self._curr_objs         = None

        # For storing forall data
        self._glob_timeout     = None
        self._glob_is_dynamic  = None
        self._glob_vhcl_data   = dict()
        self._glob_scene_data  = dict()
        self._glob_objs        = []

    def build_test_case(self, name):
        """
        Constructs the objects needed for a test
        and sets their global data, if any

        :param name: test case name
        """
        self._curr_test     = SimTestCase(name)
        self._curr_vhcl     = vehicle.Vehicle()
        self._curr_scene    = scene.Scene()
        self._curr_objs     = []
        self._set_globals()

    def get_result(self):
        """
        Returns the current test case configured with
        the current vehicle, scene, and objects
        """
        self._curr_test.vehicle = self._curr_vhcl
        self._curr_test.scene   = self._curr_scene
        self._curr_test.objects = self._curr_objs
        return self._curr_test

    def add_object(self, name, model,
                  x_pos, y_pos, z_pos,
                  r_rot, p_rot, y_rot,
                  is_global):
        sim_object = object.SimObject(name, model,
                                      x_pos, y_pos, z_pos,
                                      r_rot, p_rot, y_rot)
        if is_global:
            self._glob_objs.append(sim_object)
        else:
            self._curr_objs.append(sim_object)

    def set_wave_scale(self, scale, is_global):
        self._set_scene_elem(const.SCENE_WAVE_SCALE,
                            1*(10**(-scale)), is_global)

    def set_timeout(self, timeout, is_global):
        if is_global:
            self._glob_timeout = timeout
        else:
            self._curr_test.timeout = timeout

    def use_dynamics(self, is_dynamic, is_global):
        if is_global:
            self._glob_is_dynamic = is_dynamic
        else:
            self._curr_test.is_dynamic = is_dynamic

    def set_vehicle_position(self, x, y, z, is_global):
        if z < 0:
            sys.exit("Error: vehicle z position cannot be less than 0 (above water)")
        self._set_vhcl_elem(const.VEHICLE_X_POS, x, is_global)
        self._set_vhcl_elem(const.VEHICLE_Y_POS, y, is_global)
        self._set_vhcl_elem(const.VEHICLE_Z_POS, z, is_global)

    def set_vehicle_rotation(self, r, p, y, is_global):
        self._set_vhcl_elem(const.VEHICLE_R_ROT, r, is_global)
        self._set_vhcl_elem(const.VEHICLE_P_ROT, p, is_global)
        self._set_vhcl_elem(const.VEHICLE_Y_ROT, y, is_global)

    def _set_globals(self):
        if self._glob_timeout is not None:
            self._curr_test.timeout = self._glob_timeout
        if self._glob_is_dynamic is not None:
            self._curr_test.is_dynamic = self._glob_is_dynamic
        if not len(self._glob_objs) == 0:
            self._curr_objs = self._glob_objs[:]
        for elem in self._glob_vhcl_data:
            self._curr_vhcl.data[elem] = self._glob_vhcl_data[elem]
        for elem in self._glob_scene_data:
            self._curr_scene.data[elem] = self._glob_scene_data[elem]

    def _set_vhcl_elem(self, elem, val, is_global):
        if is_global:
            self._glob_vhcl_data[elem] = val
        else:
            self._curr_vhcl.data[elem] = val

    def _set_scene_elem(self, elem, val, is_global):
        if is_global:
            self._glob_scene_data[elem] = val
        else:
            self._curr_scene.data[elem] = val
