#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module provides the funcionality for running
the uwsim simulation
"""


import constants
import process


class SimRunner:
    """
    A class for running the uwism simulation
    """
    def __init__(self):
        """
        Constructs a SimRunner
        """
        self._process = process.SimProcess()

    def setup(self):
        """
        Sets up the simulation
        """
        self._process.start()

    def exit(self):
        """
        Exits the simulation
        """
        self._process.stop()

    def run_simulation(self, timeout, scene, sim_args):
        """
        Runs the simulation

        :param timeout: the simulation timeout length
        :param scene: the simulation scene
        :param sim_args: the simulation arguments
        """
        self._process.configure_scene(scene, sim_args)
        self._process.configure_timeout(timeout)
        self._process.run_simulation()
