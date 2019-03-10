# This is an example of a simulation test suite
from sim_test import sim_test_builder

def test_position():
    builder = sim_test_builder.SimTestBuilder(test_desc="Position At Origin")
    builder.setVehiclePos(0,0,0)
    builder.createTest()

def test_rotation():
    builder = sim_test_builder.SimTestBuilder(test_desc="Default Rotation")
    builder.setVehicleRot(0,0,0)
    builder.createTest()

def test_rotation_and_position():
    builder = sim_test_builder.SimTestBuilder(test_desc="Position At Origin And" \
                                                     +" Default Rotation")
    builder.setVehicleRot(0,0,0)
    builder.setVehiclePos(0,0,0)
    builder.createTest()

def build_test_suite():
    sim_test_builder.initTestSuite() 
    test_position()
    test_rotation()
    test_rotation_and_position()
    # Add more test cases here as needed

# Build test suite
if __name__ == '__main__':
    build_test_suite()
