#!/usr/bin/env python


import yaml

#get the bit strea from the yaml file
stream = file('testYAM.yaml', 'r')    # 'document.yaml' contains a single YAML document.

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
print("parameters: ",parameters)
''''
print("V-low: ",object.v_low)
print("V_high: ",object.v_high)
print("h-low: ",object.h_low)

print("h-high: ",object.h_high)

print("s-high: ",object.s_high)
print("s-low: ",object.s_low)

print("items: ", object.dictitems)

released = {
		"iphone" : 2007,
		"iphone 3G" : 2008,
		"iphone 3GS" : 2009,
		"iphone 4" : 2010,
		"iphone 4S" : 2011,
		"iphone 5" : 2012
	}
print(released)
'''
#parameter names to find


