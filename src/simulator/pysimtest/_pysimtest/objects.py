#Created By: Logan Fillo
#Created On: 2019-03-17

"""
This module provides the classes of objects that
can be added to the uwsim simulation
"""

import constants as const


class SimObject:
    """
    Abstract object class
    """
    def __init__(self, x_pos, y_pos, z_pos,
                       r_rot, p_rot, y_rot):
        """
        Constructs an object
        """
        self.name = None

        self.data = dict()
        self.data[const.X_POS] = x_pos
        self.data[const.Y_POS] = y_pos
        self.data[const.Z_POS] = z_pos
        self.data[const.R_ROT] = r_rot
        self.data[const.P_ROT] = p_rot
        self.data[const.Y_ROT] = y_rot


class Gate(SimObject):
    """
    Gate object
    """
    def __init__(self, x_pos, y_pos, z_pos,
                       r_rot, p_rot, y_rot):
        """
        Constructs a gate
        """
        SimObject.__init__(self, x_pos, y_pos, z_pos,
                                 r_rot, p_rot, y_rot)
        self.name = "gate"


class LineMarker(SimObject):
    """
    Line marker object
    """
    def __init__(self, x_pos, y_pos, z_pos,
                       r_rot, p_rot, y_rot):
        """
        Constructs a gate
        """
        SimObject.__init__(self, x_pos, y_pos, z_pos,
                                 r_rot, p_rot, y_rot)
        self.name = "linemarker"
