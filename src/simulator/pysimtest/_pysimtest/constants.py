#Created By: Logan Fillo
#Created On: 2019-03-12

"""
This module contains the constants used
in the pysimtest library
"""

import os

DEVNULL = open(os.devnull, 'w')

# Dictionary keys, which represent xml paths
OBJECT_NAME           = "object,name"
OBJECT_MODEL_FILE     = "object,file"
OBJECT_X_POS          = "object,position,x"
OBJECT_Y_POS          = "object,position,y"
OBJECT_Z_POS          = "object,position,z"
OBJECT_R_ROT          = "object,orientation,r"
OBJECT_P_ROT          = "object,orientation,p"
OBJECT_Y_ROT          = "object,orientation,y"

VEHICLE_X_POS         = "vehicle,position,x"
VEHICLE_Y_POS         = "vehicle,position,y"
VEHICLE_Z_POS         = "vehicle,position,z"
VEHICLE_R_ROT         = "vehicle,orientation,r"
VEHICLE_P_ROT         = "vehicle,orientation,p"
VEHICLE_Y_ROT         = "vehicle,orientation,y"

SCENE_WAVE_SCALE      = "oceanState,waveScale"

# Object model paths
POLE_MODEL            = "objects/gate/vertCylinder.3ds"
GATE_MODEL            = "" #TODO
MARKER_MODEL          = "objects/pathmarker/Path_Marker.3ds"
POOL_MODEL            = "terrain/QualTask/pool.3ds"
SEAFLOOR_MODEL        = "terrain/seafloor/terrain_noship.ive"

# Defualt values
DEFAULT_TIMEOUT       = 60
DEFAULT_IS_DYNAMIC    = False

DEFAULT_VEHICLE_X_POS = 0
DEFAULT_VEHICLE_Y_POS = 0
DEFAULT_VEHICLE_Z_POS = 5

DEFAULT_VEHICLE_R_ROT = 0
DEFAULT_VEHICLE_P_ROT = 0
DEFAULT_VEHICLE_Y_ROT = 0

DEFAULT_WAVE_SCALE    = 1 * (10**(-7))
