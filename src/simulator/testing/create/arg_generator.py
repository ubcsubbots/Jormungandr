#Created By: Logan Fillo
#Created On: 2019-03-03

import csv
import os
import itertools as it

"""
This is a python script which generates a .csv file where each line
contains an element of the cartesian product of the args needed to
init a simulation test case. The entire document contains all elements
of the cartesian product of the args

"""

# Path of .py source file
SOURCE_PATH = os.path.dirname(os.path.abspath(__file__))

def main():

    # Possible args
    xpos_args = [0]
    ypos_args = [3.5]
    zpos_args = [1.5]
    r_args = [0, 3.14/2]
    p_args = [0, 3.14]
    y_args = [0, 3.14/2]

    # Create cartesian product of all args, note that the order of args matters
    cprod = it.product(xpos_args, ypos_args, zpos_args, r_args, p_args, y_args)

    # Create lines prefixed with the test num
    lines = []
    test_num = 1
    for elem in cprod:
        line = (test_num, elem)
        line = (line[0],) + line[1]
        lines.append(line)
        test_num+=1

    # Write lines to csv file
    target = SOURCE_PATH + '/args.csv'
    with open(target, 'w') as write_file:
        writer = csv.writer(write_file)
        writer.writerows(lines)

if __name__ == '__main__':
    main()
