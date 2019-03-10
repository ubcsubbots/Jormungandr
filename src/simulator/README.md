## Creating Simulation Test Suites

To create a simulation test suite, navigate to `Jormungandr/src/simulator/simtest/create` and create a test suite file
following the naming convention of `my_test_suite.py`. Then, follow the structure and syntax presented in `example_test_suite.py`to create your test suite. See `Jormungandr/src/simulator/simtest/create/sim_test/sim_test_builder.py`
to view the source code of the test building module.

## Running Simulation Test Suites

  1. Navigate to `Jormungandr/src/simulator/simtest/run`
  2. Run `./run_test_suite.sh my_test_suite.py` where `my_test_suite.py` is your test suite file

## Vewing Test Suite Results

The results of your test suite will be printed to the terminal, but a `.txt.` version identified by your
test suite and the day it was ran will be available in `Jormungandr/src/simulator/simtest/results`. Note that the
file is overwritten if it already exists, so be aware of this when running a test suite multiple times in a day.
