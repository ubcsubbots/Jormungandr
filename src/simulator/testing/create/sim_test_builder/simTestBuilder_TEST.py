#Created By: Owen Guo
#Created On: 2019-03-09

import csv
import itertools as it
import os
import subprocess

#SOURCE_PATH = os.path.dirname(os.path.abspath(__file__))
SOURCE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
target = SOURCE_PATH + '/args.csv'

DEFAULT_VEHICLE_X_POS = 1
DEFAULT_VEHICLE_Y_POS = 1
DEFAULT_VEHICLE_Z_POS = 1

DEFAULT_VEHICLE_R_ROT = 1
DEFAULT_VEHICLE_P_ROT = 1
DEFAULT_VEHICLE_Y_ROT = 1

def initTestSuite():
    with open(target, 'w') as write_file:
        pass


class SimTestBuilder:
    def __init__(self, test_name):
        self.test_name = test_name

        self.vehicle_x_pos = DEFAULT_VEHICLE_X_POS
        self.vehicle_y_pos = DEFAULT_VEHICLE_Y_POS
        self.vehicle_z_pos = DEFAULT_VEHICLE_Z_POS

        self.vehicle_r_rot = DEFAULT_VEHICLE_R_ROT
        self.vehicle_p_rot = DEFAULT_VEHICLE_P_ROT
        self.vehicle_y_rot = DEFAULT_VEHICLE_Y_ROT

    def setVehiclePos(self, x, y, z):
        self.vehicle_x_pos = x
        self.vehicle_y_pos = y
        self.vehicle_z_pos = z

    def setVehicleRot(self, r, p, z):
        self.vehicle_r_rot = r
        self.vehicle_p_rot = p
        self.vehicle_y_rot = z

    def executeTest(self):
        args = [self.test_name, self.vehicle_x_pos, self.vehicle_y_pos, self.vehicle_z_pos,
                self.vehicle_r_rot, self.vehicle_p_rot, self.vehicle_y_rot]

        print(" ========================================")
        print(" TEST: $a1 ARGS: $a2 $a3 $a4 $a5 $a6 $a7 ")

        print(" INITIALIZING TEST...")
        test_out = subprocess.Popen(["./init_test_case"] + args,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT)

        print("RUNNING SIMULATION...")
        sim_out = subprocess.Popen(["./run_simulator", "gate/gate.xml",
                                    ">", "/dev/null", "2>&1", "&"],
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT)
        print("RUNNING TEST")
