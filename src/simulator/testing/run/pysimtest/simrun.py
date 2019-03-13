#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module provides the funcionality needed
to run the uwsim simulation
"""

import constants
import subprocess
import os
import signal
import time

class SimRunner:
    """
    A class for running the processes which
    the uwism simulation
    """
    def __init__(self):
        """
        Constructs a SimRunner with a  simulation
        process manager
        """
        self._manager = _SimProcessManager()

    def configure_scene(self, scene):
        """
        Configures the uwsim scene in preparation
        for running the simulation
        """
        self._manager._SimProcessManager__configure_scene(scene)

    def configure_timeout(self, timeout):
        """
        Configures simulation timout
        """
        self._manager._SimProcessManager__configure_timeout(timeout)

    def configure_vehicle_ai(self):
        """
        Configure's vehicle ai
        """
        self._manager._SimProcessManager__configure_vehicle_ai()

    def run_simulation(self, name, sim_args):
        """
        Runs the simulation based on
        the given args

        :param name: name associated with this running
        :param sim_args: the arguments for the simulation
        """
        self._manager._SimProcessManager__init_sim(sim_args)
        self._manager._SimProcessManager__execute_sim()

    def exit(self):
        """
        Exits simulation
        """
        self._manager._SimProcessManager__clean_up()

class _SimProcessManager:
    """
    A class for managing the processes
    needed to run the simulation
    """
    def __init__(self):
        """
        Construct a _SimProcessManager with the directory
        that the pysimtest library is found in, the scene
        path for the simulation, and the uwsim and vehicle
        ai launch group process ids
        """
        self._lib_path = os.path.dirname(
                         os.path.dirname(
                         os.path.abspath(__file__)))
        self._scene_path  = ""
        self._sim_timeout = 60

        self._uwsim_gpid  = 0
        self._launch_gpid = 0


    def __configure_scene(self, scene):
        self._scene_path = scene + "/" + scene

    def __configure_timeout(self, timeout):
        self._sim_timeout = timeout

    def __configure_vehicle_ai(self):
        """
        Launch ros simulator ai in
        subprocess
        """
        cmd = "roslaunch simulator simulator_ai_launch.launch"
        proc = subprocess.Popen(cmd, shell=True, stdout=constants.DEVNULL,
                                preexec_fn=os.setsid)
        self._launch_gpid = proc.pid

    def __init_sim(self, sim_args):
        """
        Generates the scene.xml file based on
        given simulation args. Note that this
        command is executed in this process (to
        prevent race conditions with uwsim)

        :param sim_args: simulation arguments
        """
        args = self.__format_sim_args(sim_args)
        xacro_path = (self._lib_path + "/scenes/" +
                      self._scene_path + ".xacro")
        xml_path   = (self._lib_path + "/scenes/" +
                      self._scene_path + ".xml")
        cmd        = ("rosrun xacro xacro --inorder " +
                      xacro_path + " > " + xml_path + args)
        subprocess.call(cmd, shell=True, stdout=constants.DEVNULL)

    def __execute_sim(self):
        """
        Executes the uwsim simulation process
        in a subprocess. The simulation is killed
        when either the test passes or the timeout
        length passes
        """
        cmd = ("rosrun uwsim uwsim_binary --dataPath " +
                self._lib_path + "/data --configfile " +
                self._lib_path + "/scenes/" + self._scene_path + ".xml")
        proc = subprocess.Popen(cmd, shell=True, stdout=constants.DEVNULL,
                                preexec_fn=os.setsid)
        self._uwsim_gpid = proc.pid
        time.sleep(self._sim_timeout)
        os.killpg(os.getpgid(self._uwsim_gpid), signal.SIGTERM)

    def __clean_up(self):
        """
        Kills any processes used by the
        simulation that might still be running
        """
        os.killpg(os.getpgid(self._uwsim_gpid), signal.SIGTERM)
        os.killpg(os.getpgid(self._launch_gpid), signal.SIGTERM)


    def __format_sim_args(self, sim_args):
        arg_string = ""
        for arg in sim_args:
            param = arg
            value = str(sim_args[arg])
            arg_string += " " + param + ":=" + value + " "
        return arg_string
