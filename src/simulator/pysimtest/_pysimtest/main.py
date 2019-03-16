#Created By: Logan Fillo
#Created On: 2019-03-12

"""
This module contains the pysimtest main program
"""


import importlib
import testsuite


def main():
    mod  = importlib.import_module("__main__")
    subs = testsuite.SimTestSuite.__subclasses__()
    print("running test suite...")
