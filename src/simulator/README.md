## Creating Simulation Test Suites

To create a simulation test suite, navigate to `Jormungandr/src/simulator/simtest/create` and create a test suite file
following the naming convention of `my_test_suite.py`. Then, follow the structure and syntax presented in `example_test_suite.py`to create your test suite. See `Jormungandr/src/simulator/simtest/create/sim_test/sim_test_builder.py`
to view the source code of the test building module.

## Running Simulation Test Suites

  1. Navigate to `Jormungandr/src/simulator/simtest/run`
  2. Run `./run_test_suite.sh my_test_suite.py` where `my_test_suite.py` is your test suite file

## Vewing Test Suite Results

The results of your test suite will be printed to the terminal, but a `.txt` version identified by your
test suite and the day it was ran will be available in `Jormungandr/src/simulator/simtest/results`. Note that the
file is overwritten if it already exists, so be aware of this when running a test suite multiple times in a day.


# Pysimtest
Pysimtest is a uwsim simulation testing framework, based on the Java JUnit as well as the python unittest testing frameworks.

Pysimtest provides a pythonic solution for simple modular testing of ai algorithms using the uwism simulation environment. It offers a high-level interface to easily change the simulation by modifying scene parameters, setting the vehicle's initial pose, and adding multiple objects at varying positions. Creating and running a simulation test suite using Pysimtest is easy and efficient.

## Table of Contents
- [Example] (#example)
- [Creating Test Suites] (#creating-test-suites)
- [Running Test Suites] (#running-test-suites)
- [Code API] (#code-api)
  - [Decorators] (#decorators)
  - [Functions] (#functions)


## Example
```
#Created By: You
#Created On: 0000-00-00

"""
Short description of example test suite
"""

import pysimtest

class ExampleTestSuite(pysimtest.SimTestSuite):

    @pysimtest.forall
    def forall(self):
        self.add_gate(0,-2, 0)
        self.set_timeout(180)

    @pysimtest.test(run=True)
    def test_example_one(self):
        self.set_vehicle_position(-3,-2,0)
        self.set_vehicle_rotation(3.14, 0, 0)
```

## Creating Test Suites
To create a valid simulation test suite using pysimtest, you must adhere to certain conventions. First, make a python module `my_test_module.py` and store it in the`simtests` package under `Jormungandr/src/simulator`. Next, import `pysimtest` and make a class `MyTestSuite` which extends `pysimtest.SimTestSuite`.  If you decide that you want to set parameters which will apply to all test cases in your suite, you can do so by overriding the abstract `forall` method. Remember to decorate the `forall` method with the `@pysimtest.forall` decorator for it to properly work.  Any parameter set in the `forall` method will be automatically applied to each test case.  To create a valid test, define a function `test_certain_condition` and wrap it with the `@pysimtest.test(run=True)` decorator. Note that a valid test must start with the string 'test' and be wrapped with the decorator.  Set the parameters for the test case by calling any of the functions defined below, remembering to call them using `self`. You can use the test decorator's `run` parameter to make the test run or not. You may define multiple test suites in the same module (i.e having `MyOtherTestSuite` also in `my_test_module.py`) and run them all using a single command.

## Running Test Suites
There are two ways to run a test suite. 
- 1.  Navigate to `Jormungandr/src/simulator/pysimtest` and execute the following on the command line: `python pysimtest.py my_test_module.py`
- 2. Source the shell script `pysimtest.sh` found in `Jormungandr/src/simulator/pysimtest`, or add it to your `.bashrc`, and enter the command `pysimtest my_test_module.py`. This option conveniently works from any directory.

Once you run your test suite, the terminal will notify you which test in which suite is currently running. To skip a test, you can enter `Ctr`+`C` into the terminal. The best option for troubleshooting is to close your current terminal and open a new one.
## Code API

### Decorators
- `@pysimtest.forall`: If overriding the `forall` method, wrap it in this decorator
- `@pysimtest.test(run=True)`: Wrap all test methods with this decorator. Use the `run` parameter to specifiy if you want the test to run or not

### Functions
- `SimTestSuite.add_pole(x, y, z)`: Spawns a pole in the simulation at the position x,y,z
- `SimTestSuite.add_gate(x_pos, y_pos, z_pos, r_rot, p_rot, y_rot)`: Spawns a gate in the simulation at the position x,y,z with orientation r,p,y
- `SimTestSuite.set_timeout(secs)`: Sets the timeout length for a test case.  After the timeout length passes, the next test, if any, will run
- `SimTestSuite.set_vehicle_position(x, y, z)`: Sets the vehicle's initial position to be x,y,z
- `SimTestSuite.set_vehicle_rotation(r, p, y)`: Sets the vehicle's inital orientation to by r,p,y

