#!/usr/bin/env python

import os 

def get_files(root_path,ext):
	all_files = []
	for root, dirs, files in os.walk(root_path):
		for filename in files:
			if filename.lower().endswith(ext):
				all_files.append(os.path.join(root, filename))
	return all_files

currentdir = os.getcwd()
tests_folder = currentdir+"/tests/"
offline_test_configs = get_files(tests_folder,".yaml")
print(offline_test_configs)
						
						
# python generator_test.py --config online_config.yaml

try: 
	for testconfig in offline_test_configs:
		command = "python generator_test.py --config {0}".format(testconfig)
		os.system(command)
		print("Ran {0}".format(command))
		input("press enter to continue")
except Exception as e:
	print(e)
	print("Error running command")
	pass	

			