#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module contains the functionality
for creating test cases
"""


import constants as const
import vehicle
import scene
import objects


class SimTestCase:
    """
    A class representing a simulation test case
    """
    def __init__(self, name):
        """
        Constructs a SimTestCase
        """
        self.name      = name
        self.timeout   = const.DEFAULT_TIMEOUT
        self.vehicle   = None
        self.scene     = None
        self.objects   = None


class SimTestBuilder:
    """
    A class for building simulation test cases
    """
    def __init__(self):
        """
        Construct a SimTestBuilder
        """
        self._curr_test         = None
        self._curr_vhcl         = None
        self._curr_scene        = None
        self._curr_objs         = None

        self._glob_test_attrs   = dict()
        self._glob_vhcl_attrs   = dict()
        self._glob_scene_attrs  = dict()
        self._glob_objs         = []

    def build_test_case(self, name):
        """
        Constructs the objects needed for
        a test case and sets their global
        attributes, if any.

        :param name: test case name
        """
        self._curr_test     = SimTestCase(name)
        self._curr_vhcl     = vehicle.Vehicle()
        self._curr_scene    = scene.Scene()
        self._curr_objs     = []
        self._set_globals()

    def get_result(self):
        """
        Returns the current test configured
        with the current vehicle, scene,
        and objects
        """
        self._curr_test.vehicle = self._curr_vhcl
        self._curr_test.scene   = self._curr_scene
        self._curr_test.objects = self._curr_objs
        return self._curr_test

    def add_gate(self, x_pos, y_pos, z_pos, r_rot, p_rot, y_rot, is_global):
        gate = objects.Gate(x_pos, y_pos, z_pos, r_rot, p_rot, y_rot)
        if is_global:
            self._glob_objs.append(gate)
        else:
            self._curr_objs.append(gate)

    def set_timeout(self, timeout, is_global):
        pass
        #TODO

    def set_vehicle_position(self, x, y, z, is_global):
        self._set_vhcl_attr(const.VEHICLE_X_POS, x, is_global)
        self._set_vhcl_attr(const.VEHICLE_Y_POS, y, is_global)
        self._set_vhcl_attr(const.VEHICLE_Z_POS, z, is_global)

    def set_vehicle_rotation(self, r, p, y, is_global):
        self._set_vhcl_attr(const.VEHICLE_R_ROT, r, is_global)
        self._set_vhcl_attr(const.VEHICLE_P_ROT, p, is_global)
        self._set_vhcl_attr(const.VEHICLE_Y_ROT, y, is_global)

    #DEPRECATED
    def _set_globals(self):
        self._curr_objs = slef._glob_objs
        
        for attr in self._glob_test_attrs:
            self._curr_test.test_attrs[attr] = self._glob_test_attrs[attr]
        for attr in self._glob_vhcl_attrs:
            self._curr_vhcl.data[attr] = self._glob_vhcl_attrs[attr]
        for attr in self._glob_scene_attrs:
            self._curr_scene.data[attr] = self._glob_scene_attrs[attr]

    #DEPRECATED
    def _set_test_attr(self, attr, val, is_global):
        if is_global:
            self._glob_test_attrs[attr] = val
        else:
            if attr not in self._glob_test_attrs:
                self._curr_test.test_attrs[attr] = val

    def _set_vhcl_attr(self, attr, val, is_global):
        if is_global:
            self._glob_vhcl_attrs[attr] = val
        else:
            if attr in self._glob_vhcl_attrs:
                glob_attr = self._glob_vhcl_attrs[attr]
                self._curr_vhcl.data[attr] = glob_attr
            else:
                self._curr_vhcl.data[attr] = val

    def _set_scene_attr(self, attr, val, is_global):
        if is_global:
            self._glob_scene_attrs[attr] = val
        else:
            if attr in self._glob_scene_attrs:
                glob_attr = self._glob_scene_attrs[attr]
                self._curr_scene.data[attr] = glob_attr
            else:
                self._curr_scene.data[attr] = val
