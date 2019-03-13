import simTestManager as STM
from functools import wraps



def main():
    #hey hey its test 1
    test1 = STM.SimTestBuilder(1)
    test1.setVehiclePos(2,3,4)
    test1.setVehicleRot(0,0,0)

    test2 = STM.SimTestBuilder(2)
    test2.setVehiclePos(2,3,4)
    test2.setVehicleRot(9,9,9)

    test3 = STM.SimTestBuilder(3)
    test3.setVehiclePos(2,3,4)
    test3.setVehicleRot(4,5,6)

    test4 = STM.SimTestBuilder(4)
    test4.setVehiclePos(2,3,4)
    test4.setVehicleRot(4,5,6)

    manager = STM.SimTestManager()
    manager.addTest(test1)
    manager.addTest(test2)
    manager.addTest(test3)
    manager.addTest(test4)

    manager.executeAllTests()


if __name__ == '__main__':
    main()
