#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module contains the implementation of
the function decorators for testsuite
"""


import functools


def forall(func):
    """
    Decorator for forall.

    Puts the test suite being set up into global state
    for the duration of this method, so that any methods
    called in func will set global data for the test suite
    """
    # Add a function attribute if it is decorated
    func.is_forall = True
    @functools.wraps(func)
    def _decorator(self):
        self._in_global_state = True
        func(self)
        self._in_global_state = False
    return _decorator

def test(func, run):
    """
    Parameterized decorater for test.

    Maintains test declaration order through num function
    attribute, adds run function attribute if decorater is
    set to run, then, in the wrapper, if it is set to run, builds
    a new test object so that wrapped function sets parameters of
    said object, then adds the test object to the test suite
    """
    # Add a function attribute if it is decorated
    func.is_test  = True
    # Maintain test declaration order
    test.counter += 1
    func.num = test.counter
    # Add function attribute if it set to run
    if run:
        func.run = True
    @functools.wraps(func)
    def _decorator(self):
        if run:
            name = func.__name__
            self._new_test(name)
            func(self)
            self._add_test()
    return _decorator

# Add internal counter attribute
test.counter = 0
