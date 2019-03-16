#Created By: Logan Fillo
#Created On: 2019-03-14

"""
This module provides the functionality needed
to store uwsim data related to its vehicle
"""


import constants as const


class Vehicle:
    """
    A class representing a uwsim vehicle
    """
    def __init__(self):
        self.vhcl_attr = dict()
        self._set_defaults()

    def _set_defaults(self):
        self.vhcl_attr[const.VEHICLE_X_POS] = const.DEFAULT_VEHICLE_X_POS
        self.vhcl_attr[const.VEHICLE_Y_POS] = const.DEFAULT_VEHICLE_Y_POS
        self.vhcl_attr[const.VEHICLE_Z_POS] = const.DEFAULT_VEHICLE_Z_POS

        self.vhcl_attr[const.VEHICLE_R_ROT] = const.DEFAULT_VEHICLE_R_ROT
        self.vhcl_attr[const.VEHICLE_P_ROT] = const.DEFAULT_VEHICLE_P_ROT
        self.vhcl_attr[const.VEHICLE_Y_ROT] = const.DEFAULT_VEHICLE_Y_ROT
