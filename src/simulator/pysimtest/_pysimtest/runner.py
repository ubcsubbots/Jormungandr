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

    def configure_scene(self, scene, sim_args):
        """
        Configures the uwsim scene in preparation
        for running the simulation

        :param scene: the simulation scene
        :param sim_args: the simulation arguments
        """
        self._process.configure_scene(scene, sim_args)

    def configure_timeout(self, timeout):
        """
        Configures simulation timout mechanism

        :param timeout: the simulation timeout length
        """
        self._process.configure_timeout(timeout)

    def run_simulation(self):
        """
        Runs simulation
        """
        self._process.run_simulation()

    def start(self):
        """
        Starts the simulation
        """
        self._process.start()

    def stop(self):
        """
        Stops the simulation
        """
        self._process.stop()
