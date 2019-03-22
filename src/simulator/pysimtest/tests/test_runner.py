#Created By: Logan Fillo
#Created On: 2019-03-20


import xml.etree.ElementTree as xmltree
import os

from _pysimtest import constants as const
from _pysimtest import runner
from _pysimtest import object
import pytest


"""
Test suite for _pysimtest.runner
"""

lib_path = os.path.dirname(
           os.path.dirname(
           os.path.abspath(__file__)))

class TestRunner:

    def test_setup(self):
        sim_runner  = runner.SimRunner()
        sim_runner.setup()
        #TODO: assert that launch vehicle ai is running
        sim_runner.exit()

    def test_all_processes_and_exit(self):
        sim_runner  = runner.SimRunner()
        sim_runner._launch_vehicle_ai()
        sim_runner._launch_dynamics()
        sim_runner.run_simulation(10)
        sim_runner.exit()
        #TODO: assert that all processes are stopped

    def test_stop_simulation(self):
        sim_runner = runner.SimRunner()
        sim_runner._launch_dynamics()
        sim_runner.run_simulation(10)
        sim_runner.stop_simulation()
        #TODO: assert that dynamics and sim were stopped but not ai

    def test_configure_data(self):
        tree = xmltree.parse(lib_path +
                            "/uwsim/scenes/xml/vehicle.xml")
        root = tree.getroot()
        data  = dict()
        data[const.VEHICLE_X_POS] = 100
        data[const.VEHICLE_Y_POS] = -100
        data[const.VEHICLE_Z_POS] = 100
        sim_runner = runner.SimRunner()
        nodes = sim_runner._configure_data(root, data)
        for elem in data:
            path = list(elem.split(","))
            val = data[elem]
            target = root
            for node in path:
                target = target.find(node)
            assert target.text == str(val)

    def test_configure_objects(self):
        sim_runner = runner.SimRunner()
        x_pos, y_pos, z_pos, r_rot, p_rot, y_rot = 0,0,0,0,0,0
        sim_object1 = object.SimObject("", const.POLE_MODEL,
                                      x_pos, y_pos, z_pos,
                                      r_rot, p_rot, y_rot)
        sim_object2 = object.SimObject("", const.POOL_MODEL,
                                      x_pos, y_pos, z_pos,
                                      r_rot, p_rot, y_rot)
        obj_nodes = sim_runner._configure_objects([sim_object1, sim_object2])
        assert len(obj_nodes) == 2
