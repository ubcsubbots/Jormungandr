import pysimtest

class Suite(pysimtest.SimTestSuite):

    @pysimtest.forall
    def forall(self):
        self.set_timeout(3)
        self.set_vehicle_position(0,0,0)

    @pysimtest.test(run=True)
    def test_one(self):
        pass

    @pysimtest.test(run=False)
    def test_two(self):
        pass
