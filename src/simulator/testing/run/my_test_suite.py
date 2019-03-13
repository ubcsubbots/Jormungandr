from pysimtest import simtest

class MyTestSuite(simtest.SimTestSuite):

    @simtest.setup
    def setup(self):
        self.set_scene("gate")
        self.set_timeout(15)

    @simtest.test(run=True)
    def test_one(self):
        self.set_vehicle_position(0,0,0)
        self.set_vehicle_rotation(0,0,0)
        self.assert_passes_gate(True)

    @simtest.test(run=False)
    def test_two(self):
        self.set_vehicle_position(1,1,1)
        self.set_vehicle_rotation(1,1,1)
        self.assert_passes_gate(False)

    @simtest.test(run=False)
    def test_three(self):
        self.set_vehicle_position(1,0,1)
        self.set_vehicle_rotation(0,1,0)
        self.assert_passes_gate(True)


if __name__ == "__main__":
    simtest.main()













    my_test_suite = MyTestSuite()
    my_test_suite.setup()
    my_test_suite.test_one()
    my_test_suite.test_two()
    my_test_suite.test_three()
    my_test_suite.main()
