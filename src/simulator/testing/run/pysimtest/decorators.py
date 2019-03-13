#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module contains the implementation of
the function decorators for simtest
"""


import functools


def setup(func):
    @functools.wraps(func)
    def _decorator(self):
        func(self)
    return _decorator

def test(func, run):
    @functools.wraps(func)
    def _decorator(self):
        if run:
            test_name = func.__name__
            self._SimTestSuite__create_new_test(test_name)
            func(self)
            self._SimTestSuite__add_test()
    return _decorator
