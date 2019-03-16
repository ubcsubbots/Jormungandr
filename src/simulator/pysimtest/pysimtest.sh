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

  python pysimtest.py $1

}
