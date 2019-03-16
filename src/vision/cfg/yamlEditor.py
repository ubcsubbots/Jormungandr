#!/usr/bin/env python
'''
	Created By: Bryson Marazzi
	Created On: March 14th, 2019
	Description: Reads the contents of a given yaml file which contains the serialization of the parameters object from the
				 dynamic reconfiguration file export. Then creates a new output file for a launch file to read.
'''

import yaml
import io,sys


'''
	Method to parse python objects from a given yaml file, and output a cleaner version of the parameters to outFileName. 
	Expects: outFileName and inFileName to have extension .yaml or .yml. 
'''
def edit(inFileName,outFileName):
	#get the bit strea from the yaml file
	stream = file(inFileName, 'r')    # 'document.yaml' contains a single YAML document.

	#get the python object from the yaml file
	object = yaml.load(stream)

	#obtain the keys that are represented in the yaml file
	keys = object.keys()

	#make a dictinary to write to yaml file, ignoring the "groups"
	parameters = dict()
	for k in keys:
		if(k != "groups"):
			parameters[k] = object.get(k)

	#write to a new yaml file
	with io.open(outFileName, 'w', encoding='utf8') as outfile:
	    yaml.dump(parameters, outfile, default_flow_style=False, allow_unicode=True)


def main():
    #Obtain the parameters passed from the 
    if len(sys.argv) > 2:
	    inFileName = sys.argv[1]
	    outFileName = sys.argv[2]
	    #print("\n\nOUTFILENAME\n\n" + outFileName)
	    edit(inFileName,outFileName)
	    print("Success, output yaml file as " + outFileName)

    else:
	print("Error, please pass two parameters, inFile, outFile.")


if __name__ == '__main__':
    main()

