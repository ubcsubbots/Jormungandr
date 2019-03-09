import simTestBuilder as SimTestBuilder


def test_1():
    sim_test_builder = SimTestBuilder.SimTestBuilder(test_name="test")
    sim_test_builder.setVehiclePos(2,3,4)
    sim_test_builder.createTest()

def test_2():
    sim_test_builder = SimTestBuilder.SimTestBuilder(test_name=2)
    sim_test_builder.setVehiclePos(9,9,9)
    sim_test_builder.createTest()

def test_3():
    sim_test_builder = SimTestBuilder.SimTestBuilder(test_name=3)
    sim_test_builder.setVehicleRot(0,0,0)
    sim_test_builder.createTest()

def main():
    SimTestBuilder.initTestSuite()
    test_1()
    test_2()
    test_3()


if __name__ == '__main__':
    main()
