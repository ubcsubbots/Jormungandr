#Created By: Logan Fillo
#Created On: March 09, 2019

"""
An example of a test suite created
using the sim_test_builder module
"""

from sim_test import sim_test_builder

def test_position_at_origin():
    builder = sim_test_builder.SimTestBuilder(test_desc="Position at Origin") # Add short description of test
    builder.set_vehicle_pos(0,0,0) # Set paramaters as desired
    builder.create_test() # Add this at end of every test

def test_no_rotation():
    builder = sim_test_builder.SimTestBuilder(test_desc="No Rotation")
    builder.set_vehicle_rot(0,0,0)
    builder.create_test()

def test_rotation_and_position():
    builder = sim_test_builder.SimTestBuilder(test_desc="Position at Origin with" \
                                                       +" No Rotation")
    builder.set_vehicle_rot(0,0,0)
    builder.set_vehicle_pos(0,0,0)
    builder.create_test()

def build_test_suite(): # This method is needed to build the test suite
    sim_test_builder.init_test_suite() # Always include this line at the start
    # Add more/less test cases as needed
    test_position_at_origin()
    test_no_rotation()
    test_rotation_and_position()

# Always include this at the end
if __name__ == '__main__':
    build_test_suite()
