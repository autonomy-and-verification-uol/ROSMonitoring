#!/usr/bin/env python

# MIT License
#
# Copyright (c) [2019] [Angelo Ferrando]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import sys
import yaml
import argparse
import shutil 
import pathlib

from generator_ros2 import *

# test using python generator_test.py --config online_config.yaml
# or offline_config.yaml

def copy_dir(source, destination):
    destination_path: pathlib.Path

    if isinstance(source, str):
        source_path = pathlib.Path(source)
    elif isinstance(source, pathlib.Path):
        source_path = source

    if isinstance(destination, str):
        destination_path = pathlib.Path(destination)
    elif isinstance(destination, pathlib.Path):
        destination_path = destination

    destination_path.mkdir(parents=True, exist_ok=True)
    if source_path.is_dir():
        destination_path = destination_path.joinpath(source_path.name)
        destination_path.mkdir(parents=True, exist_ok=True)

    for item in os.listdir(source_path):
        s: pathlib.Path = source_path / item
        d: pathlib.Path = destination_path / item
        if s.is_dir():
            copy_dir(s, d)
        else:
            shutil.copy2(str(s), str(d))

def generate_monitor_package(monitor_id, topics_with_types_and_action, log, url, port, oracle_action, silent, warning):
	monloc = 'code/monitor/monitor/'
	packageloc = 'code/monitor/'
	mongen = MonitorGenerator()
	lines = mongen.create_mon_file_lines(topics_with_types_and_action, monitor_id, silent, oracle_action, url, port, log)
	tp_lists = mongen.get_topic_msg_types(topics_with_types_and_action)
	mongen.codegenutils.write_lines(lines, monitor_id, monloc)
	mongen.create_package_xml(tp_lists, packageloc)


def main(argv):
	
	parser = argparse.ArgumentParser(
		description='this is a Python program to generate for monitoring ROS topics',
		formatter_class=argparse.RawTextHelpFormatter)
	parser.add_argument('--config_file',
		help='YAML configuration file',
		default='./config.yaml',
		metavar='STRING')
	args = parser.parse_args()  # maybe in the future we will need more arguments, for now it's just one

	with open(args.config_file, 'r') as stream:  # open the config file for the generator
		try:
			config = yaml.safe_load(stream)  # load the config file
			nodes = {}
			package_name = 'monitor'
			monpathfound = False
			
			if 'path' in config:
				monpath = config['path']
				if not monpath.endswith('src/'):
					if not monpath.endswith('src'):
						print('Monitor path needs to end with src/ - current value: {0}'.format(monpath))
						return
					else:
						monpath = monpath+"/"
						print('Adding a backslash to monitor path')
				monpathfound = True
				monloc = monpath + package_name + '/' + package_name + '/'
				packageloc = monpath + package_name + '/'
							
								
						
					
			if 'nodes' in config:  # check if there are nodes to be instrumented
				for node in config['nodes']:
					if 'name' not in node['node'] or 'package' not in node['node'] or 'path' not in node['node']:
						print('For each node you have to give: name, package and path to its launch file.')
						return
					if node['node']['name'] in nodes:
						print('There cannot be two nodes with the same name.')
						return
					nodes[node['node']['name']] = (node['node']['package'], node['node']['path'], [])
			if 'monitors' in config:  # check if there are monitors to create
				
				ids = []
				for monitor in config['monitors']:

					if 'id' not in monitor['monitor'] or 'topics' not in monitor['monitor'] or 'log' not in monitor['monitor']:
						print('Each monitor in the configuration file must contain the id, when, log and the list of the topics fields.')
						return
					if 'oracle' in monitor['monitor']  and ('port' not in monitor['monitor']['oracle'] or 'url' not in monitor['monitor']['oracle'] or 'action' not in monitor['monitor']['oracle']):
						print('Url, port and action must be specified inside the oracle.')
						return
					for topic_with_types_and_action in monitor['monitor']['topics']:
						if 'name' not in topic_with_types_and_action or 'type' not in topic_with_types_and_action or 'action' not in topic_with_types_and_action:
							print('Each topic needs a name, a type and an action.')
							return
						if topic_with_types_and_action['action'] not in ['log', 'filter']:
							print('The actions available for the topics are: log, filter.')
							return
						if (('publishers' not in topic_with_types_and_action and 'subscribers' not in topic_with_types_and_action) or 'oracle' not in monitor['monitor']) and topic_with_types_and_action['action'] == 'filter':
							print('In order to filter the wrong messages, the monitor has to be ONLINE and in the middle of the communication. To have this, the \'oracle\' field and the list of publishers (or subscribers) must be given.')
							return
						if 'publishers' in topic_with_types_and_action and 'subscribers' in topic_with_types_and_action:
							print('The publishers and subscribers fileds are mutually exclusive. The instrumentation process affects only one side per time.')
							return
						if 'publishers' in topic_with_types_and_action:
							for publisher in topic_with_types_and_action['publishers']:
								if publisher not in nodes:
									print('The publisher ' + publisher + ' has not been defined in the list of nodes.')
									return
								nodes[publisher][2].append(topic_with_types_and_action['name'])
						if 'subscribers' in topic_with_types_and_action:
							for subscriber in topic_with_types_and_action['subscribers']:
								if subscriber not in nodes:
									print('The subscriber ' + subscriber + ' has not been defined in the list of nodes.')
									return
								nodes[subscriber][2].append(topic_with_types_and_action['name'])
					if 'oracle' in monitor['monitor']:
						url = monitor['monitor']['oracle']['url']
						port = monitor['monitor']['oracle']['port']
						oracle_action = monitor['monitor']['oracle']['action']
						if not isinstance(url, str) or not isinstance(port, int):
							print('The url must be a string and port must be an integer.')
							return
						if oracle_action not in ['nothing', 'modify']:
							print('The actions available for the oracle are: nothing, modify.')
							return
					else:
						url = None
						port = None
						oracle_action = None
					ids.append(monitor['monitor']['id'])
					if 'silent' in monitor['monitor'] and monitor['monitor']['silent']:
						silent = True
					else:
						silent = False
					if 'warning' in monitor['monitor'] and monitor['monitor']['warning']:
						warning = monitor['monitor']['warning']
					else:
						warning = 0
					
					generate_monitor_package(monitor['monitor']['id'], monitor['monitor']['topics'], monitor['monitor']['log'], url, port, oracle_action, silent, warning)
				lfg = LaunchFileGen()
				launchpath = 'code/monitor/launch/'
				lfg.write_monitor_launch(ids, 'monitor', launchpath)
				lfg.instrument_node_launch_files(nodes)
				# now that we are all done lets go copy the entire folder 
				copy_dir('code/',packageloc)
				
		except yaml.YAMLError as exc:
			print(exc)


if __name__ == '__main__':
	main(sys.argv)
