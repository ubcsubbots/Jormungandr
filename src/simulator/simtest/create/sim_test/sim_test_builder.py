#Created By: Owen Guo
#Created On: March 09, 2019

"""
This module provides a framework for building
simulation test suites. See example_test_suite.py
for an example of it's use
"""

import csv
import os

SOURCE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

TARGET = SOURCE_PATH + '/args.csv'

DEFAULT_VEHICLE_X_POS = 1
DEFAULT_VEHICLE_Y_POS = 1
DEFAULT_VEHICLE_Z_POS = 1

DEFAULT_VEHICLE_R_ROT = 1
DEFAULT_VEHICLE_P_ROT = 1
DEFAULT_VEHICLE_Y_ROT = 1

def initTestSuite():
    """
    Initializes arguments file for this
    test suite

    :return: returns nothing
    """
    with open(TARGET, 'w') as write_file:
        pass


class SimTestBuilder:
    """
    A class used to build a simulation test
    """
    def __init__(self, test_desc):
        """
        Constructs a simulation test builder

        :param test_desc: A short descriptor of the test
        """
        self.test_desc = test_desc

        self.vehicle_x_pos = DEFAULT_VEHICLE_X_POS
        self.vehicle_y_pos = DEFAULT_VEHICLE_Y_POS
        self.vehicle_z_pos = DEFAULT_VEHICLE_Z_POS

        self.vehicle_r_rot = DEFAULT_VEHICLE_R_ROT
        self.vehicle_p_rot = DEFAULT_VEHICLE_P_ROT
        self.vehicle_y_rot = DEFAULT_VEHICLE_Y_ROT

    def setVehiclePos(self, x, y, z):
        """
        Sets the intial x, y and z position
        of the simulation robot

        :param x: robot's x position
        :param y: robot's y position
        :param z: robot's z position
        :return: returns nothing
        """
        self.vehicle_x_pos = x
        self.vehicle_y_pos = y
        self.vehicle_z_pos = z

    def setVehicleRot(self, r, p, z):
        """
        Sets the intial r, p and y rotation
        of the simulation robot

        :param r: robot's r rotation
        :param p: robot's p rotation
        :param y: robot's y rotation
        :return: returns nothing
        """
        self.vehicle_r_rot = r
        self.vehicle_p_rot = p
        self.vehicle_y_rot = z

    def createTest(self):
        """
        creates a test case based on self's
        current members

        :return: returns nothing
        """
        row = [self.test_desc, self.vehicle_x_pos, self.vehicle_y_pos, self.vehicle_z_pos,
                self.vehicle_r_rot, self.vehicle_p_rot, self.vehicle_y_rot]

        with open(TARGET, 'a') as write_file:
            writer = csv.writer(write_file)
            writer.writerow(row)
