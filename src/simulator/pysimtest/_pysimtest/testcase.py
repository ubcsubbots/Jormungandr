#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module contains the functionality
for building test cases
"""


import constants as const
import vehicle
import scene


class SimTestCase:
    """
    A class representing a simulation test case
    """
    def __init__(self, name):
        """
        Constructs a SimTestCase
        """
        self.name      = name
        self.sim_args  = dict()
        self.test_args = dict()

        self.vehicle   = None
        self.scene     = None

        self._set_defaults()

    def set_sim_args(self, sim_args):
        self.sim_args = sim_args

    def _set_defaults(self):
        self.test_args[const.SCENE]   = const.DEFAULT_SCENE
        self.test_args[const.TIMEOUT] = const.DEFAULT_TIMEOUT


class SimTestFactory:
    """
    A class for instantiating objects needed
    to build a test case
    """
    def __init__(self):
        """
        Construct a SimTestFactory
        """
        pass

    def create_test(self, name):
        return SimTestCase(name)

    def create_vehicle(self):
        return vehicle.Vehicle()

    def create_scene(self):
        return scene.Scene()


class SimTestBuilder:
    """
    A class for building simulation test cases
    """
    def __init__(self):
        """
        Construct a SimTestBuilder
        """
        self._factory           = SimTestFactory()

        self._curr_test         = None
        self._curr_vhcl         = None
        self._curr_scene        = None

        self._glob_test_args    = dict()
        self._glob_vhcl_args    = dict()
        self._glob_scene_args   = dict()

    def build_test_case(self, name):
        self._curr_test     = self._factory.create_test(name)
        self._curr_vhcl     = self._factory.create_vehicle()
        self._curr_scene    = self._factory.create_scene()
        self._set_globals()

    def get_result(self):
        # sim_args = self._combine_vhcl_scene_attr()
        # self._curr_test.set_sim_args(sim_args)

        self._curr_test.vehicle = self._curr_vhcl
        self._curr_test.scene = self._curr_scene
        
        return self._curr_test

    def set_scene(self, scene, is_global):
        self._set_test_attr(const.SCENE, scene, is_global)

    def set_timeout(self, timeout, is_global):
        self._set_test_attr(const.TIMEOUT, timeout, is_global)

    def set_vehicle_position(self, x, y, z, is_global):
        self._set_vhcl_attr(const.VEHICLE_X_POS, x, is_global)
        self._set_vhcl_attr(const.VEHICLE_Y_POS, y, is_global)
        self._set_vhcl_attr(const.VEHICLE_Z_POS, z, is_global)

    def set_vehicle_rotation(self, r, p, y, is_global):
        self._set_vhcl_attr(const.VEHICLE_R_ROT, r, is_global)
        self._set_vhcl_attr(const.VEHICLE_P_ROT, p, is_global)
        self._set_vhcl_attr(const.VEHICLE_Y_ROT, y, is_global)

    def _set_globals(self):
        for attr in self._glob_test_args:
            self._curr_test.test_args[attr]   = self._glob_test_args[attr]
        for attr in self._glob_vhcl_args:
            self._curr_vhcl.vhcl_attr[attr]   = self._glob_vhcl_args[attr]
        for attr in self._glob_scene_args:
            self._curr_scene.scene_attr[attr] = self._glob_scene_args[attr]

    def _set_test_attr(self, attr, val, is_global):
        if is_global:
            self._glob_test_args[attr] = val
        else:
            if attr not in self._glob_test_args:
                self._curr_test.test_args[attr] = val

    def _set_vhcl_attr(self, attr, val, is_global):
        if is_global:
            self._glob_vhcl_args[attr] = val
        else:
            if attr not in self._glob_vhcl_args:
                self._curr_vhcl.vhcl_attr[attr] = val

    def _set_scene_attr(self, attr, val, is_global):
        if is_global:
            self._glob_scene_args[attr] = val
        else:
            if attr not in self._glob_scene_args:
                self._curr_scene.scene_attr[attr] = val

    def _combine_vhcl_scene_attr(self):
        sim_args = self._curr_vhcl.vhcl_attr.copy()
        sim_args.update(self._curr_scene.scene_attr)
        return sim_args
