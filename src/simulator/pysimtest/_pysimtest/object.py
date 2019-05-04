#Created By: Logan Fillo
#Created On: 2019-03-17

"""
This module provides the functionaltiy for
storing simulation object data
"""

import constants as const


class SimObject:
    """
    A simulation object
    """
    def __init__(self, name, model, x_pos, y_pos, z_pos,
                                    r_rot, p_rot, y_rot):
        """
        Constructs an object
        """

        self.data = dict()
        self.data[const.OBJECT_NAME]       = name
        self.data[const.OBJECT_MODEL_FILE] = model
        self.data[const.OBJECT_X_POS]      = x_pos
        self.data[const.OBJECT_Y_POS]      = y_pos
        self.data[const.OBJECT_Z_POS]      = z_pos
        self.data[const.OBJECT_R_ROT]      = r_rot
        self.data[const.OBJECT_P_ROT]      = p_rot
        self.data[const.OBJECT_Y_ROT]      = y_rot
