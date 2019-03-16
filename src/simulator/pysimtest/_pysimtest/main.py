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
    if not (len(args) == 2):
        print("Error: missing test suite name")
        exit(1)
    if not args[1].endswith(".py"):
        print("Error: needs a python (.py) file")
        exit(1)

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
    if len(matches) > 1 or len(matches) == 0 :
        print("Error: problem with test suite name")
        exit(1)
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
