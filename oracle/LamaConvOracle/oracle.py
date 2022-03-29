#!/usr/bin/env python3

# MIT License
#
# Copyright (c) [2020] [Angelo Ferrando]
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
#
# Modified [2022] [Fatma Faruq] for using LamaConv
import os
import subprocess
import sys
import json
sys.path.append('../TLOracle/')
from websocket_server import WebsocketServer
from threading import *
import argparse
import importlib
from enum import Enum

# type of properties available in LamaConv
class TLTENSE(Enum):
	FUTURE = 0
	PAST = 1 # because we need to modify the arguments a bit to generate a pt monitor


class cd:
    """Context manager for changing the current working directory"""
    def __init__(self, newPath):
        self.newPath = os.path.expanduser(newPath)

    def __enter__(self):
        self.savedPath = os.getcwd()
        os.chdir(self.newPath)

    def __exit__(self, etype, value, traceback):
        os.chdir(self.savedPath)


class LamaConvOracle(object):

	def __init__(self):
		self.property = None
		self.tl_oracle = None
		self.model = None
		self.result = None
		self.monitor_file_name='monitor.py'
		self.lamaconv_location=os.getcwd()
		self.tense = TLTENSE.FUTURE
		self.lamaconv_fn = 'rltlconv.jar'

	# Called for every client connecting (after handshake)
	def new_client(self,client, server):
		print("New ROS monitor connected and was given id %d" % client['id'])
		# server.send_message_to_all("Hey all, a new client has joined us")


	# Called for every client disconnecting
	def client_left(self,client, server):
		print("ROS monitor (%d) disconnected" % client['id'])


	# Called when a client sends a message
	def message_received(self,client, server, message):
		message_dict = json.loads(message)
		if self.check_event(message) is True:
			message_dict['verdict'] = 'true'
		elif self.check_event(message) is False:
			message_dict['verdict'] = 'false'
			message_dict['spec'] = property.PROPERTY
		else:
			message_dict['verdict'] = 'unkonwn'
		server.send_message(client, json.dumps(message_dict))

	def monitor_result_callback(self,monitor_res):
		self.result = monitor_res

	def create_monitor_file(self,python_monitor_code):
		mon_file_name = self.monitor_file_name
		monitor_file_prologue = "#Python monitor generated using LamaConv\n#This file stores the monitor that is used\n"
		print("Saving LamaConv monitor as python file")
		with open(mon_file_name,"w") as f:
			f.write(monitor_file_prologue+python_monitor_code)
			print("LamaConv monitor python file saved")
		module_name = mon_file_name.replace(".py","")
		monitor = importlib.import_module(module_name,"Monitor")
		print("Loaded LamaConv monitor")
		return monitor.Monitor(self.monitor_result_callback)



	def generate_lamaconv_monitor(self,ltl_property):
		lamaconv_dir = self.lamaconv_location
		lamaconv_fn = self.lamaconv_fn
		python_monitor_code = None
		print("Switching to LamaConv directory")
		with cd(lamaconv_dir):
			print("Switched to LamaConv directory")
			if not os.path.isfile(lamaconv_fn):
				print("Lamaconv jar {0} does not exist in folder {1}. Please check the values for the arguments lamaconvdir and lamaconvfn. LamaConv can be downloaded from https://www.isp.uni-luebeck.de/lamaconv".format(self.lamaconv_fn,os.getcwd()))
				raise Exception("Could not find LamaConv jar")
			print("Generating LamaConv monitor")
			subprocessargs = ['java','-jar',lamaconv_fn,ltl_property,'--props']
			if self.tense is TLTENSE.FUTURE:
				# say it's a formula
				subprocessargs.append('--formula')
				# say we want a moore monitor
				subprocessargs.append('--moore')
				# we want to minimise the automaton/monitor
				subprocessargs.append('--min')

			elif self.tense is TLTENSE.PAST:
				# say we want a past time monitor
				# lamaconv generates an inductive monitor
				subprocessargs.append('--ptmonitor')

			else:
				print('Invalid tense argument, see help')
			# we want python code
			subprocessargs.append('--pythoncode')
			p = subprocess.run(subprocessargs, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
			python_monitor_code = p.stdout
			if 'RuntimeException' in p.stderr or p.returncode!= 0 :
				print('Error: {0}'.format(p.stderr))
				print('Error in arguments to lamaconv. Please check the value of the tense argument (currently tense is {0})\n Command run was:\n {1}'.format(self.tense,subprocessargs))
				return None
			#print(python_monitor_code)
		python_mon = self.create_monitor_file(python_monitor_code)
		return python_mon


	# Function checking the event against the specification
	# generates a lamaconv monitor if one does not exist
	# checks message against monitor
	# lamaconv is three valued so we're looking for violated, satisfied, neither violated nor satisfied

	def check_event(self,event):
		event_dict = json.loads(event)
		if not self.tl_oracle:
			# create the ltl string

			ltl_prop = 'LTL='+self.property.PROPERTY
			# use lamaconv to generate a monitor
			self.tl_oracle = self.generate_lamaconv_monitor(ltl_prop)
			if self.tl_oracle is None:
				print('Error generating oracle from lamaconv')
				raise Exception('Terminating program due to error')
		# so now we get the predicates
		abs_msg = self.property.abstract_message(event_dict)
		# we pass these to the monitor which is a moore fsm generated by lamaConv
		self.tl_oracle.transit(abs_msg)
		# the transit function updates the monitor result callback
		# so then we an just return the result
		return self.result

	def main(self,argv):
		parser = argparse.ArgumentParser(
			description='this is an Oracle Python implementation based on LamaConv for monitoring future-time LTL properties',
			formatter_class=argparse.RawTextHelpFormatter)
		parser.add_argument('--property',
			help='Python file describing the property, the predicates and how to translate JSON messages to high-level predicate representations',
			default = 'property',
			type = str)
		parser.add_argument('--online',
			action='store_true')
		parser.add_argument('--offline',
			action='store_true')
		parser.add_argument('--port',
			help='Port where the Websocket Oracle has to listen on',
			default = 8080,
			type = int)
		parser.add_argument('--trace',
			help='File to analyse containing a trace of events generated by a previous execution of the system',
			type = str)
		parser.add_argument('--lamaconvloc',
			help='Path to directory where the LamaConv jar file is stored, default is current working directory. Please download LamaConv from https://www.isp.uni-luebeck.de/lamaconv',
			default=os.getcwd(),
			type=str)
		parser.add_argument('--lamaconvfn',
			help='Name of the lamaconv jar file. Default rltlconv.jar',
			default='rltlconv.jar',
			type=str)
		parser.add_argument('--tense',
			help='TL tense - past-time LTL or future-time LTL. Write past for past-time and future for future-time. Support for mixed LTL is not available yet. Default value is future-time.',
			default='future',
			type=str)
		args = parser.parse_args()

		self.property = importlib.import_module(args.property)
		self.lamaconv_location = args.lamaconvloc
		self.lamaconv_fn = args.lamaconvfn
		if args.tense == 'future':
			self.tense = TLTENSE.FUTURE
		elif (args.tense == 'past'):
			self.tense = TLTENSE.PAST
		else:
			print('Invalid tense {0}. Please specify the tense. For future, --tense future. For past, --tense past')
			return

		if args.online:
			# init Websocket
			server = WebsocketServer(args.port)
			server.set_fn_new_client(self.new_client)
			server.set_fn_client_left(self.client_left)
			server.set_fn_message_received(self.message_received)
			server.run_forever()
		elif args.offline:
			if args.trace is None:
				print('For offline verification you have to specify the file containing the trace to analyse (example, --trace <path_to_file>)')
				return
			with open(args.trace, 'r') as log_file:
				trace = log_file.readlines()
				for event in trace:
					property_value = 'unknown'
					isconsistent = ' '
					if self.check_event(event) is True:
						property_value = 'true'
					elif self.check_event(event) is False:
						property_value = 'false'
						isconsistent = ' not '
					print('Property {0}: {1}'.format(property_value, self.property.PROPERTY))
					print('The event: \n {0} \n is{1}consistent with the specification.'.format(event, isconsistent))
				#return
		else:
			print('You have to specify if the oracle has to perform Online (--online) or Offline (--offline) verification')

if __name__ == '__main__':
	LamaConvOracle().main(sys.argv)
