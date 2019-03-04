#!/bin/bash

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"

# Convert command line args to ros args
TEST_NUM_ARG="testnum:=$1"
XPOS_ARG="xpos:=$2"
YPOS_ARG="ypos:=$3"
ZPOS_ARG="zpos:=$4"
ROLL_ARG="roll:=$5"
PITCH_ARG="pitch:=$6"
YAW_ARG="yaw:=$7"
#GATE_X_ARG="gatex:=$8"
#GATE_Y_ARG="gatex:=$9"
#GATE_Z_ARG="gatex:=$10"

TEST_NUM="${TEST_NUM_ARG:9:${#TEST_NUM_ARG}}"

# Convert .xacro to .xml with given args
rosrun xacro xacro --inorder "$CURR_DIR/scenes/gate/gate.xacro" > "$CURR_DIR/scenes/gate/gate.xml" $TEST_NUM_ARG\
-- $XPOS_ARG $YPOS_ARG $ZPOS_ARG \
-- $ROLL_ARG $PITCH_ARG $YAW_ARG \
#-- $GATE_X_ARG $GATE_Y_ARG $GATE_Z_ARG \

echo " TEST INITIALIZED..."
