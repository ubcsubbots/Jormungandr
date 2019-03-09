#!/bin/bash

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"

SIM_TEST=$1

# Execute python script which generates args.csv
python "$CURR_DIR/$1"

# The csv file in which each line contains a set of args for a test case
ARGS_CSV="$CURR_DIR/args.csv"

# Launch ros nodes (in seperate process)
roslaunch simulator simulator_ai_launch.launch > /dev/null 2>&1 &

PASSED=0
TOTAL=0

# For each line in args.csv, initialize a test case with args and run the simulation
while IFS="," read a1 a2 a3 a4 a5 a6 a7 a8
do

  echo " ========================================"
  echo " TEST: $a1 ARGS: $a2 $a3 $a4 $a5 $a6 $a7 "

  # Initialize test case
  echo " INITIALIZING TEST..."
  ./init_test_case.sh $a1 $a2 $a3 $a4 $a5 $a6 $a7

  # Run this iteration's version of the simulator (in seprate process)
  echo " RUNNING SIMULATION..."
  ./run_simulator.sh "gate/gate.xml" > /dev/null 2>&1 &

  echo "========================================">> $CURR_DIR/results.txt
  echo "TEST: $a1 ARGS: $a2 $a3 $a4 $a5 $a6 $a7 ">> $CURR_DIR/results.txt

  # Compile and run C++ script which determines test result
  echo " RUNNING TEST..."
  g++ -std=gnu++11 test.cpp -o test

  RESULT="$(./test)"
  echo $RESULT >> $CURR_DIR/results.txt
  if [[ $RESULT = "PASSED" ]]; then
    PASSED=$((PASSED+1))
  fi

  echo " TEST COMPLETE!"

  TOTAL=$a1

done < $ARGS_CSV

# CLeanup
pkill -x uwsim_binary
pkill -x roslaunch

# Send percentage passed to results
PERCENT_PASSED=$((PASSED*100/TOTAL))
echo "========================================">> $CURR_DIR/results.txt
echo "TESTS PASSED: $PERCENT_PASSED%" >> $CURR_DIR/results.txt
