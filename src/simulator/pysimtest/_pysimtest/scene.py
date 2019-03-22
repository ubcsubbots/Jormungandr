#Created By: Logan Fillo
#Created On: 2019-03-14

"""
This module provides the functionality needed
to store uwsim data related to its scene
"""


import constants as const


class Scene:
    """
    A class representing a uwsim scene
    """
    def __init__(self):
        """
        Constructs a scene
        """
        self.data = dict()
        self._set_defaults()

    def _set_defaults(self):
        self.data[const.SCENE_WAVE_SCALE] = const.DEFAULT_WAVE_SCALE
