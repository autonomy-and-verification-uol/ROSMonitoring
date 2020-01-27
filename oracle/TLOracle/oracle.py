import reelay
import sys
import json
from websocket_server import WebsocketServer
from threading import *
import argparse
import importlib
from enum import Enum

# type of properties available in reelay
class TypeOfProperty(Enum):
	PTL = 0
	MTL = 1
	STL = 2

# ws_lock = Lock()

# Called for every client connecting (after handshake)
def new_client(client, server):
	print("New ROS monitor connected and was given id %d" % client['id'])
	# server.send_message_to_all("Hey all, a new client has joined us")


# Called for every client disconnecting
def client_left(client, server):
	print("ROS monitor (%d) disconnected" % client['id'])


# Called when a client sends a message
def message_received(client, server, message):
    global time
    # print("ROS monitor (%d) said: %s" % (client['id'], message))
    message_dict = json.loads(message)
    if not time:
        time = message_dict['time']
        message_dict['time'] = 0
    else:
        message_dict['time'] = message_dict['time'] - time
    if tlOracle.update(property.abstract_message(message_dict)):
        server.send_message(client, message)
    else:
        message_dict['error'] = True
        message_dict['spec'] = property.PROPERTY
        server.send_message(client, json.dumps(message_dict))


def main(argv):
	global property
	global tlOracle
	global time

	parser = argparse.ArgumentParser(
        description='this is an Oracle Python implementation based on Reelay for monitoring PTL, MTL and STL properties',
        formatter_class=argparse.RawTextHelpFormatter)
	parser.add_argument('--property',
		help='Python file describing the property, the predicates and how to translate JSON messages to high-level predicate representations',
		default = 'property',
		type = str)
	parser.add_argument('--port',
		help='Port where the Websocket Oracle has to listen on',
		default = 8080,
		type = int)
	args = parser.parse_args()

	property = importlib.import_module(args.property)
	time = None

	if property.TYPE.value == TypeOfProperty.PTL.value:
		tlOracle = reelay.past_ltl.monitor(
	        pattern = property.PROPERTY)
	elif property.TYPE.value == TypeOfProperty.MTL.value:
		tlOracle = reelay.past_mtl.monitor(
	        pattern = property.PROPERTY,
	        time_model = 'discrete')
	elif property.TYPE.value ==TypeOfProperty.STL.value:
		tlOracle = reelay.past_stl.monitor(
	        pattern = property.PROPERTY,
	        time_model = 'dense')
	else:
		print('Property type not supported by reelay')
		return

	# init Websocket
	server = WebsocketServer(args.port)
	server.set_fn_new_client(new_client)
	server.set_fn_client_left(client_left)
	server.set_fn_message_received(message_received)
	server.run_forever()

if __name__ == '__main__':
	main(sys.argv)
