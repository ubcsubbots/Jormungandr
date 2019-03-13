# SOURCE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
#
#
#
#
# def executeAllTests(self):
#     print("\n\nThere are currently " + str(len(self.tests)) + " tests loaded.")
#     print("Preparing to execute tests:\n")
#
#     try:
#         subprocess.call("roslaunch simulator simulator_ai_launch.launch > /dev/null 2>&1 &", shell=True)
#
#         for test in self.tests:
#             test.executeTest()
#     except KeyboardInterrupt:
#         subprocess.call("pkill -x uwsim_binary", shell=True)
#         subprocess.call("pkill -x roslaunch", shell=True)
#
#     subprocess.call("pkill -x uwsim_binary", shell=True)
#     subprocess.call("pkill -x roslaunch", shell=True)
#
# def executeTest(self):
#     args = [str(self.test_name), str(self.vehicle_x_pos), str(self.vehicle_y_pos),
#             str(self.vehicle_z_pos), str(self.vehicle_r_rot), str(self.vehicle_p_rot), str(self.vehicle_y_rot)]
#
#     print("========================================")
#     print("Test Name: " + str(self.test_name))
#     print(self.__getStats())
#
#     print(" INITIALIZING TEST...")
#
#     subprocess.call(['./init_test_case.sh'] + args)
#
#     print(" RUNNING SIMULATION...")
#     subprocess.call("./run_simulator.sh \"gate/gate.xml\" > /dev/null 2>&1 &", shell=True)
#     print(" RUNNING TEST")
#
#     subprocess.call("g++ -std=gnu++11 test.cpp -o test", shell=True)
#     run = subprocess.call(["./test"])
#
# def __getStats(self):
#     acc = ""
#     acc += "Vehicle Position (x,y,z): (" + str(self.vehicle_x_pos) + ", " + str(self.vehicle_y_pos) + ", " + str(self.vehicle_z_pos) + ")\n"
#     acc += "Vehicle Rotation (r,p,y): (" + str(self.vehicle_p_rot) + ", " + str(self.vehicle_p_rot) + ", " + str(self.vehicle_y_rot) + ")\n"
#     return acc
