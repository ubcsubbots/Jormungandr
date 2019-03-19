#Created By: Logan Fillo
#Created On: 2019-03-12

"""
This module contains the constants used
in the pysimtest library
"""

import os

DEVNULL = open(os.devnull, 'w')

# Dictionary keys
X_POS         = "position,x"
Y_POS         = "position,y"
Z_POS         = "position,z"
R_ROT         = "orientation,r"
P_ROT         = "orientation,p"
Y_ROT         = "orientation,y"

VEHICLE_X_POS         = "vehicle,position,x"
VEHICLE_Y_POS         = "vehicle,position,y"
VEHICLE_Z_POS         = "vehicle,position,z"
VEHICLE_R_ROT         = "vehicle,orientation,r"
VEHICLE_P_ROT         = "vehicle,orientation,p"
VEHICLE_Y_ROT         = "vehicle,orientation,y"

# Defualt values
DEFAULT_SCENE         = "custom"
DEFAULT_TIMEOUT       = 200

DEFAULT_VEHICLE_X_POS = 0
DEFAULT_VEHICLE_Y_POS = 0
DEFAULT_VEHICLE_Z_POS = 0

DEFAULT_VEHICLE_R_ROT = 0
DEFAULT_VEHICLE_P_ROT = 0
DEFAULT_VEHICLE_Y_ROT = 0
