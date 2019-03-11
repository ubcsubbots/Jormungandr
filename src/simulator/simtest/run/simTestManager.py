#Created By: Owen Guo
#Created On: 2019-03-09

import subprocess

DEFAULT_VEHICLE_X_POS = 1
DEFAULT_VEHICLE_Y_POS = 1
DEFAULT_VEHICLE_Z_POS = 1

DEFAULT_VEHICLE_R_ROT = 1
DEFAULT_VEHICLE_P_ROT = 1
DEFAULT_VEHICLE_Y_ROT = 1


class SimTestManager:
    def __init__(self):
        self.tests = []

    def addTest(self, test):
        self.tests.append(test)

    def executeAllTests(self):
        print("\n\nThere are currently " + str(len(self.tests)) + " tests loaded.")
        print("Preparing to execute tests:\n")

        try:
            subprocess.call("roslaunch simulator simulator_ai_launch.launch > /dev/null 2>&1 &", shell=True)

            for test in self.tests:
                test.executeTest()
        except KeyboardInterrupt:
            subprocess.call("pkill -x uwsim_binary", shell=True)
            subprocess.call("pkill -x roslaunch", shell=True)

        subprocess.call("pkill -x uwsim_binary", shell=True)
        subprocess.call("pkill -x roslaunch", shell=True)

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
        args = [str(self.test_name), str(self.vehicle_x_pos), str(self.vehicle_y_pos),
                str(self.vehicle_z_pos), str(self.vehicle_r_rot), str(self.vehicle_p_rot), str(self.vehicle_y_rot)]

        print("========================================")
        print("Test Name: " + str(self.test_name))
        print(self.__getStats())

        print(" INITIALIZING TEST...")

        subprocess.call(['./init_test_case.sh'] + args)

        print(" RUNNING SIMULATION...")
        subprocess.call("./run_simulator.sh \"gate/gate.xml\" > /dev/null 2>&1 &", shell=True)
        print(" RUNNING TEST")

        subprocess.call("g++ -std=gnu++11 test.cpp -o test", shell=True)
        run = subprocess.call("./test", shell=True)

    def __getStats(self):
        acc = ""
        acc += "Vehicle Position (x,y,z): (" + str(self.vehicle_x_pos) + ", " + str(self.vehicle_y_pos) + ", " + str(self.vehicle_z_pos) + ")\n"
        acc += "Vehicle Rotation (r,p,y): (" + str(self.vehicle_p_rot) + ", " + str(self.vehicle_p_rot) + ", " + str(self.vehicle_y_rot) + ")\n"
        return acc
