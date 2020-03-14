#Created By: Logan Fillo
#Created On: 2019-03-18


import signal
import threading
import time
import os

from _pysimtest import loader
import pytest


"""
Test suite for _pysimtest.loader
"""

dir_path = os.path.dirname(os.path.realpath(__file__))

class TestLoader:

    def test_load_and_execute_undecorated_test(self):
        path = (dir_path + "/testfiles/loader/loader_test_undecorated_test.py")
        try:
            sim_loader = loader.SimTestLoader()
            sim_loader.load_and_execute(path)
            pytest.fail()
        except SystemExit:
            pass

    def test_load_and_execute_undecorated_forall(self):
        path = (dir_path + "/testfiles/loader/loader_test_undecorated_forall.py")
        try:
            sim_loader = loader.SimTestLoader()
            sim_loader.load_and_execute(path)
            pytest.fail()
        except SystemExit:
            pass

    def test_load_suites_all_running(self):
        path = (dir_path + "/testfiles/loader/loader_test_all_run.py")
        try:
            sim_loader = loader.SimTestLoader()
            suites = sim_loader._load_suites(path)
            for suite in suites:
                assert len(suite._tests) == 2
        except SystemExit:
            pytest.fail()
            pass

    def test_load_suites_half_running(self):
        path = (dir_path +"/testfiles/loader/loader_test_half_run.py")
        try:
            sim_loader = loader.SimTestLoader()
            suites = sim_loader._load_suites(path)
            for suite in suites:
                assert len(suite._tests) == 1
        except SystemExit:
            pytest.fail()
            pass

    def test_load_suites_two_suites(self):
        path = (dir_path +"/testfiles/loader/loader_test_two_suites.py")
        try:
            sim_loader = loader.SimTestLoader()
            suites = sim_loader._load_suites(path)
            assert len(suites) == 2
            running_tests = 0
            for suite in suites:
                running_tests += len(suite._tests)
            assert running_tests == 3
        except SystemExit:
            pytest.fail()
            pass

    def test_execute_suites(self):
        path = (dir_path +"/testfiles/loader/loader_test_all_run.py")
        sim_loader = loader.SimTestLoader()
        suites   = sim_loader._load_suites(path)
        start    = time.time()
        sim_loader._execute_suites(suites)
        end      = time.time()
        run_time = end-start
        print(run_time)
        assert run_time > 10 and run_time < 20
