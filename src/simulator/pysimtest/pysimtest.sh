#!/bin/bash

pysimtest () {

  # Check that we got exactly one argument
  if [ "$#" -ne 1 ]; then
      echo "Error, missing test suite name"
      return
  fi

  python "$1.py"


  # Check to see if file exists
  # FILE_EXISTS="$(find $PWD -type f | grep -cm1 "$1.py" )"

  # if [ "$FILE_EXISTS" -ne "1" ] ; then
    # echo "File could not be found"
    # return
  # fi

  # Find file
  # FILE="$(find $PWD -type f | grep "$1.py" )"

  # cd pysimtest

  # Run package
  # python -m pysimtest $FILE

}
