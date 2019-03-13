from sim_test_builder import simTestBuilder


def test_1():
    builder = simTestBuilder.SimTestBuilder(test_name="Rotation")
    builder.setVehiclePos(2,3,4)
    builder.createTest()

def test_2():
    builder = simTestBuilder.SimTestBuilder(test_name="Postition")
    builder.setVehiclePos(9,9,9)
    builder.createTest()

def test_3():
    builder = simTestBuilder.SimTestBuilder(test_name="Position and Rotation")
    builder.setVehicleRot(0,0,0)
    builder.createTest()

def main():
    simTestBuilder.initTestSuite()
    test_1()
    test_2()
    test_3()


if __name__ == '__main__':
    main()
