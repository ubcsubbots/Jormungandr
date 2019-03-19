#Created By: Logan Fillo
#Created On: 2019-03-12

"""
This module contains the pysimtest main program
"""


import sys
import os

import loader


def check_valid_arg(args):
    """
    check if args are valid

    :param args: sys argv
    """
    if not len(args) == 2:
        sys.exit("Error: invalid arguments, needs a python module")
    if not args[1].endswith(".py"):
        sys.exit("Error: needs a python (.py) file")

def find_target(target):
    """
    Checks to make sure there is only one
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
    if len(matches) > 1:
        sys.exit("Error: multiple modules found")
    if len(matches) == 0 :
        sys.exit("Error: cannot find module")
    return matches[0]

def main(args):
    """
    Runs pysimtest

    :param args: sys argv
    """
    check_valid_arg(args)
    path       = find_target(args[1])
    sim_loader = loader.SimTestLoader()
    sim_loader.load_and_execute(path)
