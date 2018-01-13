#!/bin/bash

# Check that we got exactly one argument
if [ "$#" -ne 1 ]; then
    echo "This program takes 1 argument (the name of a scene in the scenes folder)."
    exit 1
fi
SCENE_FILENAME=$1

rosrun uwsim uwsim scenes/ $SCENE_FILENAME --dataPath data
