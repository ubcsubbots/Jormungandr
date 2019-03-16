#Created By: Logan Fillo
#Created On: 2019-03-12

"""
This module contains the constants used
in the pysimtest library
"""

import os

DEVNULL = open(os.devnull, 'w')

# Dictionary keys
SCENE         = "scene"
TIMEOUT       = "timeout"

VEHICLE_X_POS = "vehicle_x_pos"
VEHICLE_Y_POS = "vehicle_y_pos"
VEHICLE_Z_POS = "vehicle_z_pos"

VEHICLE_R_ROT = "vehicle_r_rot"
VEHICLE_P_ROT = "vehicle_p_rot"
VEHICLE_Y_ROT = "vehicle_y_rot"

# Defualt values
DEFAULT_SCENE         = "custom"
DEFAULT_TIMEOUT       = 120

DEFAULT_VEHICLE_X_POS = 0
DEFAULT_VEHICLE_Y_POS = 0
DEFAULT_VEHICLE_Z_POS = 0

DEFAULT_VEHICLE_R_ROT = 0
DEFAULT_VEHICLE_P_ROT = 0
DEFAULT_VEHICLE_Y_ROT = 0
