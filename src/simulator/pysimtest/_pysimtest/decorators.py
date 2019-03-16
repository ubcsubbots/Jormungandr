#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module contains the implementation of
the function decorators for testsuite
"""


import functools


def setup(func):
    """
    Decorator for setup. Puts the test suite
    being set up into global state for the duration
    of this method, so that any methods called in
    func will set global args for the test suite
    """
    @functools.wraps(func)
    def _decorator(self):
        self._in_global_state = True
        func(self)
        self._in_global_state = False
    return _decorator

def test(func, run):
    """
    Parameterized decorater for test. If the decorated
    method is argumented to run, creates a new test case before
    func is called so that func sets the new test case's args,
    then adds the argumented test case to the test suite
    """
    @functools.wraps(func)
    def _decorator(self):
        if run:
            test_name = func.__name__
            func.func_dict["is_test"] = True
            self._new_test(test_name)
            func(self)
            self._add_test()
    return _decorator
