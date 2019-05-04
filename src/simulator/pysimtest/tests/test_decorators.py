#Created By: Logan Fillo
#Created On: 2019-03-18


import pysimtest
import pytest


"""
Test suite for _pysimtest.decorators
"""

@pysimtest.forall
def wrapped_forall(self):
    pass

def unwrapped_forall(self):
    pass

@pysimtest.test(run=True)
def wrapped_test(self):
    pass

def unwrapped_test(self):
    pass

@pysimtest.test(run=False)
def no_run_test(self):
    pass

class TestDecorators:

    def test_forall_function_attr(self):
        assert hasattr(wrapped_forall, "is_forall")

    def test_forall_no_function_attr(self):
        assert not hasattr(unwrapped_forall, "is_forall")

    def test_test_function_attr(self):
        assert hasattr(wrapped_test, "is_test")

    def test_test_no_function_attr(self):
        assert not hasattr(unwrapped_test, "test")

    def test_test_run_attr(self):
        assert hasattr(wrapped_test, "run")

    def test_test_no_run_attr(self):
        assert not hasattr(no_run_test, "run")

    def test_counter_increases(self):
        # no_run_test was defined after wrapped_test, so counter should increase
        before = wrapped_test.num
        after  = no_run_test.num
        assert before < after
