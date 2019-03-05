#!/bin/bash

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"

# Execute python script which generates args.csv
python "$CURR_DIR/arg_generator.py"

# The csv file in which each line contains a set of args for a test case
ARGS_CSV="$CURR_DIR/args.csv"

# For each line in args.csv, initialize a test case with args and run the simulation
while IFS="," read a1 a2 a3 a4 a5 a6 a7 a8
do
  echo
  echo " ========================================"
  echo " TEST: $a1 ARGS: $a2 $a3 $a4 $a5 $a6 $a7 "
  echo " ========================================"
  ./init_test_case.sh $a1 $a2 $a3 $a4 $a5 $a6 $a7
  echo " RUNNING SIMULATION..."
  ./run_simulator.sh "gate/gate.xml" > /dev/null 2>&1 &
  # Launch node which publishes messages to simulation robot in different process
  echo " RUNNING TEST..."
  # Run C++ script which checks to see if the robot passes through gate
  # Set a shell variable to be the output of C++ script, which is written to once test ends

  # This is here to emulate the time it takes for the test to run. Because we are running the C++
  # script in this process, the pkill commands will not execute until the C++ script finishes execution
  sleep 20s

  # Kill the script that runs the simulator, and the UWsim script
  pkill -x run_simulator.s
  pkill -x uwsim_binary 
  # Kill the process which is running the node
  echo " TEST COMPLETE!"

  # Write output of C++ script to a results.txt file

done < $ARGS_CSV
