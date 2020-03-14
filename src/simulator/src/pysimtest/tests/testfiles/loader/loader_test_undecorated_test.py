import pysimtest

class Suite(pysimtest.SimTestSuite):

    # Undecorated
    # @pysimtest.test(run=True)
    def test_undecorated(self):
        self.set_vehicle_position(0,0,0)

    # Decorated
    @pysimtest.test(run=True)
    def test_decorated(self):
        self.set_vehicle_position(0,0,0)
