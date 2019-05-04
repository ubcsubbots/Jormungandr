#!/bin/bash

##########################
# pysimtest shell command
###########################

pysimtest () {

  # Make sure there is one arg
  if [ "$#" -ne 1 ]; then
      echo "Error: missing test suite name"
      return
  fi

  # Find pysimtest.py
  PYSIMTEST="$(find ~ -name "pysimtest.py" -type f)"

  # Run program with arg
  python $PYSIMTEST $1

}
