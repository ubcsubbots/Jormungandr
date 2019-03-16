#Created By: Logan Fillo
#Created On: 2019-03-12

"""
This module contains the pysimtest main program
"""


import sys
import os
import importlib
import inspect

import loader
import runner


def check_valid_arg(args):
    """
    check if args are valid

    :param args: sys argv
    """
    if not (len(args) == 2):
        print("Error: missing test suite name")
        exit(1)
    if not args[1].endswith(".py"):
        print("Error: needs a python (.py) file")
        exit(1)

def find_target(target):
    """
    checks to make sure there is only one
    existing target match, then return it

    :param target: target module name
    """
    src    = os.path.dirname(
             os.path.dirname(
             os.path.dirname(
             os.path.dirname(
             os.path.abspath(__file__)))))
    matches = []
    for root, dirs, files in os.walk(src):
        if target in files:
            matches.append(os.path.join(root, target))
    if len(matches) > 1 or len(matches) == 0 :
        print("Error: problem with test suite name")
        exit(1)
    return matches[0]

def get_module(path):
    """
    Gets the module object from path

    :param path: module path
    """
    name, suf, mode, mod_type = inspect.getmoduleinfo(path)
    module = importlib.import_module(name)
    return module

def get_suites(module):
    """
    Returns list of instances of
    all suites in the module

    :param module: target module object
    """
    import testsuite
    suites = []
    for name, obj in inspect.getmembers(module):
        if inspect.isclass(obj):
            if issubclass(obj, testsuite.SimTestSuite):
                suites.append(obj)

    return [suite() for suite in suites]

def main(args):
    """
    runs pysimtest

    :param args: sys argv
    """
    check_valid_arg(args)
    path   = find_target(args[1])

    module = get_module(path)
    suites = get_suites(module)


    for suite in suites:
        for attr in dir(suite):
            obj = getattr(suite, attr)
            if callable(obj) and attr == "setup":
                setup = obj
                setup()
        for attr in dir(suite):
            obj = getattr(suite, attr)
            if callable(obj) and attr.startswith("test"):
                test = obj
                test()
