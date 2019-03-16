#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module provides functionality for
loading tests from a system path and
executing them
"""


import importlib
import inspect

import constants as const
import runner


class SimTestLoader:
    def __init__(self):
        """
        Constructs a SimTestLoader
        """
        self._runner = runner.SimRunner()

    def load_and_execute(self, path):
        """
        Loads all tests from a path and
        executes them

        :param path: the module/package path
        """
        loaded_suites = self._load_suites(path)
        self._execute_suites(loaded_suites)


    def _load_suites(self, path):
        """
        Loads all test suites found in the
        given path

        :param path: path to suites
        :returns : the loaded suites
        """
        suites = self._get_suites(
                 self._get_module(path))
        for suite in suites:
            self._load_tests(suite)
        return suites

    def _load_tests(self, suite):
        """
        Loads setup method then loads
        all tests in the given suite

        :param suite: the test suite object
        """
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

    def _execute_suites(self, suites):
        """
        Execute the test suites

        :param suites: the test suite objects
        """
        self._runner.setup()
        for suite in suites:
            tests = suite._tests
            self._execute_tests(tests)
        self._runner.exit()

    def _execute_tests(self, tests):
        """
        Execute the test cases

        :param tests: the test cases
        """
        for test in tests:
            self._execute_test(test)

    def _execute_test(self, test):
        """
        Executes the test

        :param test: the test case
        """
        print("---------------------------")
        print("Name:    " + test.name)
        print("Timeout:  " + str(test.test_args[const.TIMEOUT]))
        print("Scene:   " + test.test_args[const.SCENE])
        self._runner.run_simulation(test.test_args[const.TIMEOUT],
                                    test.test_args[const.SCENE],
                                    test.sim_args)

    def _get_module(self, path):
        """
        Gets the module object from path

        :param path: module path
        """
        name, suf, mode, mod_type = inspect.getmoduleinfo(path)
        module = importlib.import_module(name)
        return module

    def _get_suites(self, module):
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
