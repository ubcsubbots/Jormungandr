import pysimtest

class Suite(pysimtest.SimTestSuite):

    # Undecorated
    # @pysimtest.forall
    def forall(self):
        self.set_vehicle_position(0,0,0)
