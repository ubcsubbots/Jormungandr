#Created By: Logan Fillo
#Created On: 2019-03-18


import subprocess

from _pysimtest import main

import pytest



"""
Test suite for _pysimtest.main
"""

class TestMain:

    def test_main_no_args(self):
        argv = ["module_name"]
        with pytest.raises(SystemExit):
            main.main(argv)

    def test_main_too_many_args(self):
        argv = ["module_name", "main_test_module1.py", "main_test_module2.py"]
        with pytest.raises(SystemExit):
            main.main(argv)

    def test_main_no_module(self):
        argv = ["modulename", "main_test_module_not_found.py"]
        with pytest.raises(SystemExit):
            main.main(argv)

    def test_main_too_many_module(self):
        argv = ["module_name", "main_test_collision.py"]
        with pytest.raises(SystemExit):
            main.main(argv)

    def test_main_no_py(self):
        argv = ["module_name", "main_test_collision"]
        with pytest.raises(SystemExit):
            main.main(argv)

    def test_main_pass(self):
        argv = ["module_name", "main_test_pass.py"]
        try:
            main.main(argv)
        except SystemExit:
            pytest.fail()
