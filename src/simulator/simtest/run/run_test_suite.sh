#!/bin/bash

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"

# Check that we got exactly one argument
if [ "$#" -ne 1 ]; then
    echo "[Error] This program takes 1 argument (i.e my_test_suite.py)."
    exit 1
fi

# Test suite .py file
TEST_SUITE=$1

# Exit if test suite file does not exist
if [ ! -f "$(dirname "$CURR_DIR")/create/$1" ]; then
    echo "[Error] The input file does not exist"
    exit 1
fi

# Execute python test suite file which generates args.csv
python "$(dirname "$CURR_DIR")/create/$1"

# The csv file in which each line contains a set of args for a test case
ARGS_CSV="$(dirname "$CURR_DIR")/create/args.csv"

# Initialize results file, overwrite if necessary
TEST_SUITE_NAME=$(echo "$TEST_SUITE" | cut -f 1 -d '.')
DATE="$(date +%Y-%m-%d)"
RESULTS_FILE="results-$TEST_SUITE_NAME-$DATE.txt"
RESULTS_PATH="$(dirname "$CURR_DIR")/results/$RESULTS_FILE"
cat /dev/null > "${RESULTS_PATH}"

# Launch ros nodes (in seperate process)
roslaunch simulator simulator_ai_launch.launch > /dev/null 2>&1 &

PASSED=0
TOTAL=0

# For each line in args.csv, initialize a test case with args and run the simulation
while IFS="," read a1 a2 a3 a4 a5 a6 a7 a8
do

  echo "=============================================="
  echo "TEST: $a1"
  echo "----------------------------------------------"
  echo "ARGS: x_pos = $a2    y_pos = $a3    z_pos = $a4"
  echo "      roll  = $a5    pitch = $a6    yaw   = $a7"
  echo "----------------------------------------------"
  echo "==============================================" >> $RESULTS_PATH
  echo "TEST: $a1" >> $RESULTS_PATH
  echo "----------------------------------------------" >> $RESULTS_PATH
  echo "ARGS: x_pos = $a2    y_pos = $a3    z_pos = $a4">> $RESULTS_PATH
  echo "      roll  = $a5    pitch = $a6    yaw   = $a7">> $RESULTS_PATH
  echo "----------------------------------------------" >> $RESULTS_PATH

  # Initialize test case
  TOTAL=$((TOTAL+1))
  ./init_test_case.sh $TOTAL $a2 $a3 $a4 $a5 $a6 $a7

  # Run this iteration's version of the simulator (in seprate process)
  ./run_simulator.sh "gate/gate.xml" > /dev/null 2>&1 &

  # Compile and run temporary C++ program which "determines" test result
  g++ -std=gnu++11 test.cpp -o test
  RESULT="$(./test)"

  # Determine if test passed or failed
  echo "RESULT: [$RESULT]"
  echo "RESULT: [$RESULT]" >> $RESULTS_PATH
  if [[ $RESULT = "PASSED" ]]; then
    PASSED=$((PASSED+1))
  fi

  # Kill uwsim
  pkill -x uwsim_binary

done < $ARGS_CSV

# Check to make sure at least one test ran
if [ "$TOTAL" = 0 ]; then
    echo "[Error] No tests ran succesfully"
    exit 1
fi


PERCENT_PASSED=$((PASSED*100/TOTAL))
echo "=============================================="
echo "TESTS PASSED: $PERCENT_PASSED%"
echo "==============================================">> $RESULTS_PATH
echo "TESTS PASSED: $PERCENT_PASSED%" >> $RESULTS_PATH

# Cleanup
pkill -x uwsim_binary
pkill -x roslaunch
