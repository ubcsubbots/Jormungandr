#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module contains the implementation of
the function decorators for testsuite
"""


import functools


def forall(func):
    """
    Decorator for forall. Puts the test suite
    being set up into global state for the duration
    of this method, so that any methods called in
    func will set global attr for the test suite
    """
    # Add a function attribute specifying it is decorated
    func.is_forall = True
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
    func is called so that func sets the new test case's attr,
    then adds the test case to the test suite
    """
    # Add a function attribute specifying it is decorated
    func.is_test  = True
    test.counter += 1
    # Maintain test declaration order
    func.num = test.counter
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
