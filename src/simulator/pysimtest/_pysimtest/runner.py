#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module provides the funcitonality for
running the uwsim simulation
"""

import xml.etree.ElementTree as elemtree
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
        self._launch_pgid  = None
        self._uwsim_pgid   = None

    def setup(self):
        """
        Starts any initial processes needed
        to setup the uwsim simulation
        """
        self._launch_vehicle_ai()

    def exit(self):
        """
        Kills any processes created by the
        simulation if they are still running
        """
        if self._uwsim_pgid is not None:
            os.killpg(os.getpgid(self._uwsim_pgid), signal.SIGTERM)
            self._uwsim_pgid = None
        if self._launch_pgid is not None:
            os.killpg(os.getpgid(self._launch_pgid), signal.SIGTERM)
            self._launch_pgid = None

    def configure_simulation(self,test):
        """
        Configures the out.xml file based on the given
        vehicle, scene and objects

        :param test: the test to configure the simulation with
        """
        scene_nodes     = self._configure_scene(test.scene)
        vehicle_nodes   = self._configure_vehicle(test.vehicle)
        object_nodes    = self._configure_objects(test.objects)
        interface_nodes = self._configure_interfaces()

        in_tree         = elemtree.parse(self._lib_path +
                                         "/uwsim/scenes/xml/in.xml")
        in_root         = in_tree.getroot()
        nodes = scene_nodes + vehicle_nodes + object_nodes + interface_nodes
        for node in nodes:
            # Insert node at end of in root to maintain UWsim doctype order
            in_root.insert(len(list(in_root)),node)
        header = ("<?xml version=\"1.0\" ?>\n" +
                  "<!DOCTYPE UWSimScene SYSTEM \"UWSimScene.dtd\" >\n")
        with open(self._lib_path + "/uwsim/scenes/xml/out.xml", 'wb') as file:
            file.write(header)
            in_tree.write(file)

    def run_simulation(self, timeout):
        """
        Executes the uwsim simulation process
        in a subprocess. The simulation is stopped
        when the timeout length passes

        :param timeout: the simulation timeout length
        """
        cmd = ["rosrun", "uwsim", "uwsim_binary", "--dataPath",
                self._lib_path + "/uwsim/data", "--configfile",
                self._lib_path + "/uwsim/scenes/xml/out.xml"]
        proc = subprocess.Popen(cmd,
                                stdout=constants.DEVNULL,
                                stderr=constants.DEVNULL,
                                preexec_fn=os.setsid)
        self._uwsim_pgid = proc.pid
        time.sleep(timeout)
        self.stop_simulation()

    def stop_simulation(self):
        """
        Stops the current execution of the
        simulation if it is still running
        """
        if self._uwsim_pgid is not None:
            os.killpg(os.getpgid(self._uwsim_pgid), signal.SIGTERM)
            self._uwsim_pgid = None

    def _launch_vehicle_ai(self):
        """
        Launches ros simulator ai in subprocess.
        """
        cmd = ["roslaunch", "simulator", "simulator_ai_launch.launch"]
        proc = subprocess.Popen(cmd,
                                stdout=constants.DEVNULL,
                                stderr=subprocess.PIPE,
                                preexec_fn=os.setsid)
        self._launch_pgid = proc.pid

    def _configure_vehicle(self, vehicle):
        """
        Configures all xml nodes found in vehicle.xml
        and returns them in a list
        :param vehicle: the vehicle
        """
        vehicle_tree = elemtree.parse(self._lib_path +
                                      "/uwsim/scenes/xml/vehicle.xml")
        vehicle_root = vehicle_tree.getroot()
        for elem in vehicle.data:
            target  = tuple(elem.split(","))
            t_root  = vehicle_root.find(target[0])
            t_node  = t_root.find(target[1])
            t_val   = t_node.find(target[2])
            t_val.text = str(vehicle.data[elem])
        nodes = list(vehicle_root)
        return nodes

    def _configure_scene(self, scene):
        """
        Configures all xml nodes found in scene.xml
        and returns them in a list

        :param scene: the scene
        """
        scene_tree = elemtree.parse(self._lib_path +
                                    "/uwsim/scenes/xml/scene.xml")
        scene_root = scene_tree.getroot()
        for elem in scene.data:
            target  = tuple(elem.split(","))
            t_root  = scene_root.find(target[0])
            t_node  = t_root.find(target[1])
            t_val   = t_node.find(target[2])
            t_val.text = str(scene.data[elem])
        nodes = list(scene_root)
        return nodes

    def _configure_objects(self, objects):
        """
        For all objects, configures their xml nodes and
        adds them to the main objects xml root. Then returns
        all nodes in the main objects xml root

        :param objects: the list of objects
        """
        objects_tree = elemtree.parse(self._lib_path +
                                      "/uwsim/scenes/xml/objects.xml")
        objects_root = objects_tree.getroot()
        for object in objects:
            name = object.name
            object_tree = elemtree.parse(self._lib_path +
                                        "/uwsim/scenes/xml/" + name + ".xml")
            object_root = object_tree.getroot()
            # TODO: Do stuff here to alter the object tree
            for child in list(object_tree.getroot()):
                objects_root.append(child)
        nodes = list(objects_root)
        return nodes

    def _configure_interfaces(self):
        """
        Configures interface nodes and returns
        them in a list
        """
        interface_tree = elemtree.parse(self._lib_path +
                                      "/uwsim/scenes/xml/interfaces.xml")
        interface_root = interface_tree.getroot()
        #TODO: add option to add dynamic sim interfaces
        nodes = list(interface_root)
        return nodes
