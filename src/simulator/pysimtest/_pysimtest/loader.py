#Created By: Logan Fillo
#Created On: 2019-03-11

"""
This module provides functionality for
loading tests from a system path and
executing them
"""


import imp
import inspect
import sys

import constants as const
import runner


class SimTestLoader:
    """
    A class for loading and executing simulation test suites
    """
    def __init__(self):
        """
        Constructs a SimTestLoader
        """
        self._runner = runner.SimRunner()

    def load_and_execute(self, path):
        """
        Loads all test suites from given
        path and executes their tests

        :param path: the file path
        """
        loaded_suites = self._load_suites(path)
        self._execute_suites(loaded_suites)


    def _load_suites(self, path):
        """
        Loads all test suites found in the
        given path and returns them

        :param path: path to suites
        """
        suites = self._get_suites(
                 self._get_module(path))
        for suite in suites:
            self._load_tests(suite)
        return suites

    def _load_tests(self, suite):
        """
        Calls valid forall method then calls all the
        valid tests in the given suite. Calling
        a properly decorated test loads it into
        the suite

        :param suite: the test suite object
        """
        self._call_forall(suite)
        self._call_tests(suite)

    def _call_forall(self, suite):
        """
        Searches for a forall method in the
        given suite, if it finds it, and it
        has been properly decorated, calls it,
        else exits program

        :param suite: the test suite object
        """
        for attr in dir(suite):
            obj = getattr(suite, attr)
            if callable(obj) and attr == "forall":
                if hasattr(obj.im_func, "is_forall"):
                    forall = obj
                    forall()
                    break
                else:
                    sys.exit("Error: undecorated forall")

    def _call_tests(self, suite):
        """
        Searches for all test methods set to run in the
        given suite and checks if they are properly
        decorated. If they are, sorts them by test
        num and calls them, else exits program

        :param suite: the test suite object
        """
        tests = []
        for attr in dir(suite):
            obj = getattr(suite, attr)
            if callable(obj) and attr.startswith("test"):
                if hasattr(obj.im_func, "is_test"):
                    if hasattr(obj.im_func, "run"):
                        tests.append(obj)
                else:
                    sys.exit("Error: undecorated test")
        tests.sort(key=lambda test : test.im_func.num)
        for test in tests:
            test()

<<<<<<< HEAD
=======
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
                                    test)
>>>>>>> e5eedcd037316604eda51835537c9fe2326a1490

    def _get_module(self, path):
        """
        Gets the module object from path

        :param path: module path
        """
        name   = inspect.getmodulename(path)
        module = imp.load_source(name, path)
        return module

    def _get_suites(self, module):
        """
        Returns objects of all test suites in the module

        :param module: target module object
        """
        import suite
        suites = []
        for name, obj in inspect.getmembers(module):
            if inspect.isclass(obj):
                if issubclass(obj, suite.SimTestSuite):
                    suites.append(obj)
        # Returns instances of the suites
        return [suite() for suite in suites]

    def _execute_suites(self, suites):
        """
        Uses the simulation runner to execute
        a test suite

        :param suites: the test suite objects
        """
        self._runner.setup()
        for suite in suites:
            print("---------------------------")
            print(type(suite).__name__)
            tests = suite._tests
            self._execute_tests(tests)
        self._runner.exit()

    def _execute_tests(self, tests):
        """
        Executes all test cases. Stops the simulation if it
        receives a KeyboardInterrupt at anytime during the
        execution of a test case

        :param tests: the test cases
        """
        for test in tests:
            try:
                self._runner.configure_simulation(test)
                print("---------------------------")
                print("Name:     " + test.name)
                self._runner.run_simulation(test.timeout)
            except KeyboardInterrupt:
                print("\nInterrupt: test stopped")
                self._runner.stop_simulation()
