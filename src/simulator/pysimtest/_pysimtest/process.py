#Created By: Logan Fillo
#Created On: 2019-03-12

"""
This module contains the functionality
for maintaing the processes needed to run
the uwsim simulation
"""


import constants
import subprocess
import os
import signal
import time


class SimProcess:
    """
    A class for managing the processes
    needed to run the simulation
    """
    def __init__(self):
        """
        Construct a SimProcessManager
        """
        self._lib_path = os.path.dirname(
                         os.path.dirname(
                         os.path.abspath(__file__)))

        self._timeout = constants.DEFAULT_TIMEOUT

        # Group process id's
        self._roscore_gpid = None
        self._launch_gpid  = None
        self._uwsim_gpid   = None

    def start(self):
        """
        Starts any initial processes needed
        for to setup the uwsim simulation
        """
        self._run_roscore()
        self._launch_vehicle_ai()

    def stop(self):
        """
        Kills any processes used by the
        simulation that might be running
        """
        if self._uwsim_gpid is not None:
            os.killpg(os.getpgid(self._uwsim_gpid), signal.SIGTERM)
        if self._launch_gpid is not None:
            os.killpg(os.getpgid(self._launch_gpid), signal.SIGTERM)
        if self._roscore_gpid is not None:
            os.killpg(os.getpgid(self._roscore_gpid), signal.SIGTERM)

    def configure_scene(self, scene, sim_args):
        """
        Configures the out.xml file based on the
        .xacro scene file and input arguments.

        :param scene: the name of the .xacro file
        :param sim_args: the .xacro arguments
        """
        xacro_path = (self._lib_path +
                      "/uwsim/scenes/xacro/" +
                      scene + ".xacro")
        xml_path   = (self._lib_path +
                      "/uwsim/scenes/out.xml")
        args       =  self._format_sim_args(sim_args)

        # This is not working, we have to use shell=True :(, for some reason the args
        # are not passed to the xacro and there is a non-determinastic error that sometimes
        # arises: 'xacro: error: expected exactly one input file as argument'

        # cmd        = ["rosrun", "xacro", " xacro", "--inorder", xacro_path, ">", xml_path, args]
        # print(cmd)
        # proc = subprocess.Popen(cmd, stdout=constants.DEVNULL)

        cmd = "rosrun xacro xacro --inorder " + xacro_path + " > " + xml_path + args
        proc = subprocess.Popen(cmd, shell=True)

        proc.communicate()

    def configure_timeout(self, timeout):
        """
        Sets the timeout length of the simulation

        :param timeout: the simulation timeout length

        NOTE: Eventually this should configure the results
              mechanism which would close the simulation if
              a test passed while it is running, otherwise it
              would close out after the timeout length
        """
        self._timeout = timeout

    def run_simulation(self):
        """
        Executes the uwsim simulation process
        in a subprocess. The simulation is killed
        when either the test passes or the timeout
        length passes
        """
        cmd = ["rosrun", "uwsim", "uwsim_binary", "--dataPath",
                self._lib_path + "/uwsim/data", "--configfile",
                self._lib_path + "/uwsim/scenes/out.xml"]

        proc = subprocess.Popen(cmd,
                                stdout=constants.DEVNULL,
                                stderr=constants.DEVNULL,
                                preexec_fn=os.setsid)

        self._uwsim_gpid = proc.pid
        time.sleep(self._timeout) #TODO: add pass functionality
        os.killpg(os.getpgid(self._uwsim_gpid), signal.SIGTERM)

    def _run_roscore(self):
        """
        Runs roscore if it isn't already running
        """
        has_launched = False
        dummy_cmd    = ["rostopic", "list"]
        # This attempts to run a dummy command that needs roscore to work
        # Once the command works (i.e exit code 0), we know roscore is running
        while subprocess.call(dummy_cmd, stdout=constants.DEVNULL):
            if not has_launched:
                cmd    = ["roscore"]
                proc   = subprocess.Popen(cmd,
                                          stdout=constants.DEVNULL,
                                          preexec_fn=os.setsid)
                self._roscore_gpid = proc.pid
                has_launched       = True

    def _launch_vehicle_ai(self):
        """
        Launches ros simulator ai in subprocess
        """
        cmd = ["roslaunch", "simulator", "simulator_ai_launch.launch"]

        proc = subprocess.Popen(cmd,
                                stdout=constants.DEVNULL,
                                preexec_fn=os.setsid)
        self._launch_gpid = proc.pid

    def _format_sim_args(self, sim_args):
        args = ""
        for arg in sim_args:
            param = arg
            value = str(sim_args[arg])
            args += ( " " + param + ":=" + value + " ")
        return args
