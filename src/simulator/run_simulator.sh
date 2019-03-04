#!/bin/bash

# The current directory
CURR_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"

# Check that we got exactly one argument
if [ "$#" -ne 1 ]; then
    echo "This program takes 1 argument (the name of a scene in the scenes folder)."
    exit 1
fi
SCENE_FILENAME=$1

echo $SCENE_FILENAME

echo $CURR_DIR

rosrun uwsim uwsim_binary --dataPath $CURR_DIR/data --configfile "$CURR_DIR/scenes/${SCENE_FILENAME}"
