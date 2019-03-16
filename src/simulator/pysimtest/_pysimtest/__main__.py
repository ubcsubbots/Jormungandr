#Created By: Logan Fillo
#Created On: 2019-03-12

"""
This module contains the pysimtest main entry point
"""


import main
import atexit


@atexit.register
def cleanup():
    print("cleaning up...")

from pysimtest.main import main
main()
