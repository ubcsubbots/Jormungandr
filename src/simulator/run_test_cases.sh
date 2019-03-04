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
  echo ========================================
  echo " TEST:$a1 ARGS:$a2,$a3,$a4,$a5,$a6,$a7"
  echo ========================================
  ./init_test_case.sh $a1 $a2 $a3 $a4 $a5 $a6 $a7
  echo " RUNNING SIMULATION..."
  #./run_simulator.sh "gate/gate1.xml &"
  # Keep track of process which is running the simulator
  # Launch node which publishes messages to simulation robot in different process
  # Keep track of process which is running the node
  echo " RUNNING TEST..."
  # Run C++ script which checks to see if the robot passes through gate in this process
  # Set variable to be the output of C++ script, which is written to once test ends
  # Kill the process which is running the simulation and the process which is running the node
  echo " TEST COMPLETE!"
  # Write output of C++ script to a results.txt file

done < $ARGS_CSV
