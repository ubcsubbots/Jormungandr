#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module provides the funcitonality for
running the uwsim simulation
"""

import xml.etree.ElementTree as xmltree
import subprocess
import os
import signal
import time

import constants


class SimRunner:
    """
    A class for running the uwism simulation
    """
    def __init__(self):
        """
        Constructs a SimRunner
        """
        self._lib_path = os.path.dirname(
                         os.path.dirname(
                         os.path.abspath(__file__)))
        # Group process id's
        self._launch_pgid   = None
        self._dynamics_pgid = None
        self._uwsim_pgid    = None

    def setup(self):
        """
        Starts any initial processes needed
        to setup the simulation environment
        """
        self._launch_vehicle_ai()

    def exit(self):
        """
        Kills any processes created by the
        simulation if they are still running
        """
        self.stop_simulation()
        if self._launch_pgid is not None:
            os.killpg(os.getpgid(self._launch_pgid), signal.SIGTERM)
            self._launch_pgid = None

    def configure_simulation(self,test):
        """
        Configures the out.xml file based on the given
        vehicle, scene and objects

        This

        :param test: the test to configure the simulation with
        """
        if (test.is_dynamic):
            self._launch_dynamics()

        scene_nodes     = self._configure_scene(test.scene)
        vehicle_nodes   = self._configure_vehicle(test.vehicle)
        object_nodes    = self._configure_objects(test.objects)
        interface_nodes = self._configure_interfaces()

        tree  = xmltree.parse(self._lib_path +
                              "/uwsim/xml/in.xml")
        root  = tree.getroot()
        nodes = (scene_nodes + vehicle_nodes +
                 object_nodes + interface_nodes)
        for node in nodes:
            # Insert node at end of tree to maintain UWsim doctype order
            root.insert(len(list(root)),node)
        header = ("<?xml version=\"1.0\" ?>\n" +
                  "<!DOCTYPE UWSimScene SYSTEM \"UWSimScene.dtd\" >\n")
        with open(self._lib_path + "/uwsim/xml/out.xml", 'wb') as file:
            file.write(header)
            tree.write(file)

    def run_simulation(self, timeout):
        """
        Runs the uwsim simulation process
        in a subprocess. The simulation is stopped
        when the timeout length passes

        :param timeout: the simulation timeout length
        """
        cmd = ["rosrun", "uwsim", "uwsim_binary", "--dataPath",
                self._lib_path + "/uwsim/data", "--configfile",
                self._lib_path + "/uwsim/xml/out.xml"]
        proc = subprocess.Popen(cmd,
                                stdout=constants.DEVNULL,
                                stderr=constants.DEVNULL,
                                preexec_fn=os.setsid)
        self._uwsim_pgid = proc.pid
        time.sleep(timeout)
        self.stop_simulation()

    def stop_simulation(self):
        """
        Stops the current execution of the simulation
        if it is still running, alongside the dynamics
        if they are running as well
        """
        if self._dynamics_pgid is not None:
            os.killpg(os.getpgid(self._dynamics_pgid), signal.SIGTERM)
            self._dynamics_pgid = None
        if self._uwsim_pgid is not None:
            os.killpg(os.getpgid(self._uwsim_pgid), signal.SIGTERM)
            self._uwsim_pgid = None

    def _launch_vehicle_ai(self):
        """
        Launches ros simulator ai in subprocess.
        """
        cmd = ["roslaunch", "simulator", "simulator_ai.launch"]
        proc = subprocess.Popen(cmd,
                                stdout=constants.DEVNULL,
                                stderr=subprocess.PIPE,
                                preexec_fn=os.setsid)
        self._launch_pgid = proc.pid

    def _launch_dynamics(self):
        """
        Launches dynamics in subprocess
        """
        print("DYNAMICS LAUNCHED! (not really though)")
        pass #TODO

    def _configure_vehicle(self, vehicle):
        """
        Configures the nodes in the vehicle.xml tree
        and returns them

        :param vehicle: the vehicle
        """
        vehicle_tree = xmltree.parse(self._lib_path +
                                      "/uwsim/xml/vehicle.xml")
        vehicle_root = vehicle_tree.getroot()
        self._configure_data(vehicle_root, vehicle.data)
        nodes = list(vehicle_root)
        return nodes

    def _configure_scene(self, scene):
        """
        Configures the nodes in the scene.xml tree
        and returns them

        :param scene: the scene
        """
        scene_tree = xmltree.parse(self._lib_path +
                                    "/uwsim/xml/scene.xml")
        scene_root = scene_tree.getroot()
        self._configure_data(scene_root, scene.data)
        nodes = list(scene_root)
        return nodes

    def _configure_objects(self, objects):
        """
        Configures every object's xml tree in the given list
        of objects and returns them

        :param objects: the list of objects
        """
        nodes = []
        for object in objects:
            object_tree = xmltree.parse(self._lib_path +
                                        "/uwsim/xml/object.xml")
            object_root = object_tree.getroot()
            self._configure_data(object_root, object.data)
            nodes.extend(list(object_root))
        return nodes

    def _configure_interfaces(self):
        """
        Configures the nodes in the interfaces.xml tree
        and returns them
        """
        interface_tree = xmltree.parse(self._lib_path +
                                      "/uwsim/xml/interfaces.xml")
        interface_root = interface_tree.getroot()
        #TODO: add options to change interfaces
        nodes = list(interface_root)
        return nodes

    def _configure_data(self, root, data):
        """
        A helper function which configures an xml tree starting
        at the given root with the given data

        :param tree: the xml tree
        :param data: dictionary of data
        """
        # TODO: make this work for paths with possible collisions
        for elem in data:
            path = list(elem.split(","))
            val  = data[elem]
            self._traverse_and_insert(root, path, val)

    def _traverse_and_insert(self, root, path, val):
        """
        A helper function which traverses an xml tree, starting
        from the root and following the path until it ends. Then,
        inserts the val into whatever node the path ended on

        :param root: start of traversal
        :param path: the list of nodes the traversal visits
        :param val: the value to be inserted at end of path
        """
        target = root
        for node in path:
            target = target.find(node)
        target.text = str(val)
