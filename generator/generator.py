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
import yaml
import xml.etree.ElementTree as ET


class MonitorGenerator():

	def __init__(self, monitor_id, topics_with_types_and_action, services_with_types_and_action, log, url, port, oracle_action, silent, warning, ids, nodes):
		self.input_path = './auxiliary_files/'
		self.monitor_file = '../monitor/src/' + monitor_id + '.py'
		self.launch_file = '../monitor/run.launch'
		self.monitor_id = monitor_id
		self.topics_with_types_and_action = topics_with_types_and_action
		#added:
		self.services_with_types_and_action = services_with_types_and_action
		self.log = log
		self.url = url
		self.port = port
		self.oracle_action = oracle_action
		self.silent = silent
		self.warning = warning
		self.monitor_ids = ids
		self.nodes = nodes
		
		self.std_imports = self.get_standard_imports()
		self.topic_msg_type_imports = self.get_imports_for_topic_msg_types()
		#added:
		self.service_msg_type_imports = self.get_imports_for_service_msg_types()
		#added:
		self.service_callbacks = self.get_callback_per_service()
		self.pub_with_callbacks = self.get_publisher_per_topic()
		self.pub_dict = self.get_dict_for_publishers()
		self.msg_dict = self.get_dict_for_msg_types()
		self.monitor_def = self.get_mon_node()
		self.other_callbacks = self.get_other_callbacks()
		
		
	def get_standard_imports(self):
		# get the imports the monitor is gonna need
		f = open(os.path.join(self.input_path, 'standard_imports.txt'), 'r')
		std_imports = f.read()
		f.close()
		if self.oracle_action == 'nothing':
		    std_imports += '''\ndict_msgs = {}'''
		return std_imports
			
	def get_imports_for_topic_msg_types(self):	
        		# get the imports for the msg types used by the monitor (extracted by the previous instrumentation)
		msg_type_imports = ''
		msg_import_set = set()
		for topic_with_types_and_action in self.topics_with_types_and_action:
			i = topic_with_types_and_action['type'].rfind('.')
			package = topic_with_types_and_action['type'][:i]
			msg_type = topic_with_types_and_action['type'][i+1:]
			msg_import_set.update([(package, msg_type)])
		for (package, msg_type) in msg_import_set:
            		msg_type_imports += '''\nfrom {p} import {t}'''.format(p = package, t = msg_type)
		return msg_type_imports


	def get_imports_for_service_msg_types(self):	
        		# get the imports for the msg types used by the monitor (extracted by the previous instrumentation)
		msg_type_imports = ''
		msg_import_set = set()
		for service_with_types_and_action in self.services_with_types_and_action:
			srv_type = service_with_types_and_action['type']
			i = srv_type.rfind('.')
			package = srv_type[:i]
			msg_type = srv_type[i+1:]
			msg_import_set.update([(package, msg_type)])
		for (package, msg_type) in msg_import_set:
            		msg_type_imports += '''\nfrom {p} import {t}'''.format(p = package, t = msg_type)
		return msg_type_imports

	def get_callback_per_service(self):
		callbacks = '\n'
		for service_with_types_and_action in self.services_with_types_and_action:
			
			srv_name = service_with_types_and_action['name']
			srv_type = service_with_types_and_action['type']
			callbacks += '''\ndef callback_{srv}(request, response):\n\tglobal ws, ws_lock'''.format(srv = srv_name+'_mon'.replace('/','_'))
			callbacks += '\n\tdict = dict()'
			if not self.silent:
				callbacks += '''\n\trospy.loginfo('monitor has observed: ' + str(request))'''
				
				callbacks += '''\n\tdict['request'] = message_converter.convert_ros_message_to_dictionary(request)\n\tdict['service'] = '{srv}'\n\tdict['time'] = rospy.get_time()\n\tws_lock.acquire()'''.format(srv = srv_name)
			if self.oracle_action == 'nothing':
				callbacks += '''\n\twhile dict['time'] in dict_msgs:\n\t\tdict['time'] += 0.01'''
			if self.url != None and self.port != None:
				callbacks += '''\n\tws.send(json.dumps(dict))'''
				if self.oracle_action == 'nothing':
					callbacks += '''\n\tdict_msgs[dict['time']] = request'''
				callbacks += '\n\tmsg = self.ws.recv()\n'
				callbacks += '''\n\tws_lock.release()\n'''
				if not self.silent:
					callbacks += '''\n\trospy.loginfo('event propagated to oracle')\n'''
				callbacks += '''\n\treturn self.on_message_request(msg)\n'''

			else:
				callbacks += '''\n\tlogging(dict)'''
				callbacks += '''\n\tws_lock.release()'''
				if not self.silent:
					callbacks += '''\n\trospy.loginfo('event has been successfully logged')\n'''

		return callbacks
		            	
	def get_publisher_per_topic(self):
    		# write the creation of the publisher for each topic (and the callback function for the instrumented one)
		pub_with_callbacks = '\n'
		for topic_with_types_and_action in self.topics_with_types_and_action:
			tp_name = topic_with_types_and_action['name']
			tp_side = tp_name
			if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action:
				tp_type = topic_with_types_and_action['type']
				if 'subscribers' in topic_with_types_and_action:
					tp_side += '_mon'

			pub_with_callbacks += '''\ndef callback{tp}(data):\n\tglobal ws, ws_lock'''.format(tp = tp_name.replace('/','_'))

			if not self.silent:
            			pub_with_callbacks += '''\n\trospy.loginfo('monitor has observed: ' + str(data))'''
			pub_with_callbacks += '''\n\tdict = message_converter.convert_ros_message_to_dictionary(data)\n\tdict['topic'] = '{tp}'\n\tdict['time'] = rospy.get_time()\n\tws_lock.acquire()'''.format(tp = tp_name)
			if self.oracle_action == 'nothing':
                			pub_with_callbacks += '''\n\twhile dict['time'] in dict_msgs:\n\t\tdict['time'] += 0.01'''
            		
			if self.url != None and self.port != None:
                			pub_with_callbacks += '''\n\tws.send(json.dumps(dict))'''
                			if self.oracle_action == 'nothing':
                				pub_with_callbacks += '''\n\tdict_msgs[dict['time']] = data'''
                				                		
######
                			pub_with_callbacks += '\n\tmsg = self.ws.recv()'
                			pub_with_callbacks += '''\n\tws_lock.release()\n'''
                			pub_with_callbacks += '''\n\treturn self.on_message(msg)\n'''

			else:
            			pub_with_callbacks += '''\n\tlogging(dict)'''
            			pub_with_callbacks += '''\n\tws_lock.release()\n'''
            			if not self.silent:
                				pub_with_callbacks += '''\n\trospy.loginfo('event propagated to oracle')\n'''
            			if not self.silent:
            				pub_with_callbacks += '''\n\trospy.loginfo('event has been successfully logged')'''
            			if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action:
            				pub_with_callbacks += '''\n\tpub_dict['{tp}'].publish(data)\n'''.format(tp = tp_name)
            		
            		
		return pub_with_callbacks
       
	def get_dict_for_publishers(self):
        		# write the dictionary for dynamically keeping track of the publishers
        		pub_dict = '''\npub_dict = {'''
        		first_time = True
        		for topic_with_types_and_action in self.topics_with_types_and_action:
        		#for (topic, (type, _), _, _, _, _, _) in topics_with_types:
        			if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action:
        				if(first_time):
        					first_time = False
        				else:
        					pub_dict += ', '
        				tp_name = topic_with_types_and_action['name']
        				pub_dict += ''' '{tp1}' : pub{tp2}'''.format(tp1 = tp_name, tp2 = tp_name.replace('/','_'))
        		pub_dict += '''}'''
        
        		return pub_dict

	def get_dict_for_msg_types(self):
        		# write the dictionary for dynamically keeping track of the message types (we need it for the message converter)
        		msg_dict = '''\nmsg_dict = {'''
        		first_time = True
        		for topic_with_types_and_action in self.topics_with_types_and_action:
        		#for (topic, (type, imp), _, _, _, _, _) in topics_with_types:
        			if(first_time):
        				first_time = False
        			else:
        				msg_dict += ', '
        			tp_name = topic_with_types_and_action['name']
        			tp_type = topic_with_types_and_action['type']
        			i = tp_type.rfind('.')
        			msg_dict += ''' '{tp}' : "{ty}"'''.format(tp = tp_name, ty = tp_type[0:i].replace('.msg', '') + '/' + tp_type[i+1:])
        		msg_dict += '''}'''
        		return msg_dict

	def get_mon_node(self):
        		# write the definition of the monitor function which will be used to initialize the rosnode, and create the Subscribers for all the instrumented topics.
        		# In short, the monitor observes the instrumented topics generated by the real nodes, and then it publishes (propagates) them
        		# to the usual Subscribers
        		f = open(os.path.join(self.input_path, 'monitor_function.txt'), 'r')
        		monitor_def = '\n'+f.read().replace('$ID$', self.monitor_id)
        		f.close()
        
        		for topic_with_types_and_action in self.topics_with_types_and_action:
        			tp_name = topic_with_types_and_action['name']
        			tp_type = topic_with_types_and_action['type']
        			tp_side = tp_name
        			if 'publishers' in topic_with_types_and_action:
        				tp_side += '_mon'

        			monitor_def += '''\n\trospy.Subscriber('{tps}', {ty}, callback{tp})'''.format(tp = tp_name.replace('/','_'), tps = tp_side, ty = tp_type[tp_type.rfind('.')+1:])
        		if not self.silent:
        			monitor_def += '''\n\trospy.loginfo('monitor started and ready')'''
        		return monitor_def    

	def get_other_callbacks(self):
        		
        		# write the auxiliary callbacks functions called by the websocket used by the monitor
        		# when a topic is observed by the monitor, if we are doing online RV, it propagates the topic to the
        		# oracle. The oracle checks the event and returns the outcome to the monitor
        		# the monitor then propagates the event to the other nodes (unless we decided to filter the errors,
        		# in that case the monitor does not propagate the event)
        		if self.url != None and self.port != None:
        			other_callbacks = '''\n\ndef on_message(message):\n\tglobal error, log, actions\n\tjson_dict = json.loads(message)'''
        			other_callbacks+= '''\n\tif json_dict['verdict'] == 'true' or json_dict['verdict'] == 'currently_true' or json_dict['verdict'] == 'unknown':'''
        			other_callbacks+= '''\n\t\tif json_dict['verdict'] == 'true' and not pub_dict:\n\t\t\trospy.loginfo('The monitor concluded the satisfaction of the property under analysis, and can be safely removed.')'''
        			other_callbacks+= '''\n\t\t\tws.close()\n\t\t\texit(0)'''
        			other_callbacks+= '''\n\t\telse:\n\t\t\tlogging(json_dict)\n\t\t\ttopic = json_dict['topic']'''
        			if not self.silent:
        				other_callbacks += '''\n\t\t\trospy.loginfo('The event ' + message + ' is consistent and republished')'''
        			if self.oracle_action == 'nothing':
        				other_callbacks += '''\n\t\t\tif topic in pub_dict:\n\t\t\t\tpub_dict[topic].publish(dict_msgs[json_dict['time']])\n\t\t\tdel dict_msgs[json_dict['time']]'''
        			else:
        				other_callbacks += '''\n\t\t\tdel json_dict['topic']\n\t\t\tdel json_dict['time']\n\t\t\tROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)'''
        				other_callbacks += '''\n\t\t\t if topic in pub_dict:\n\t\t\t\tpub_dict[topic].publish(ROS_message)'''
        			other_callbacks += '''\n\telse:\n\t\tlogging(json_dict)\n\t\t#if (json_dict['verdict'] == 'false' and actions[json_dict['topic']][1] >= 1) or (json_dict['verdict'] == 'currently_false' and actions[json_dict['topic']][1] == 1):'''

        			if not self.silent:
        				other_callbacks += '''\n\t\trospy.loginfo('The event ' + message + ' is inconsistent..')'''
        			other_callbacks += '''\n\t\terror = MonitorError()\n\t\terror.topic = json_dict['topic']\n\t\terror.time = json_dict['time']\n\t\terror.property = json_dict['spec']'''
        			if self.oracle_action == 'nothing':
        				other_callbacks += '''\n\t\terror.content = str(dict_msgs[json_dict['time']])'''
        			else:
        				other_callbacks += '''\n\t\tjson_dict_copy = json_dict.copy()\n\t\tdel json_dict_copy['topic']\n\t\tdel json_dict_copy['time']'''
        				other_callbacks+='''\n\t\tdel json_dict_copy['spec']\n\t\tdel json_dict_copy['error']\n\t\terror.content = json.dumps(json_dict_copy)'''
        			other_callbacks += '''\n\t\tpub_error.publish(error)\n\t\tif json_dict['verdict'] == 'false' and not pub_dict:'''
        			other_callbacks += '''\n\t\t\trospy.loginfo('The monitor concluded the violation of the property under analysis, and can be safely removed.')\n\t\t\tws.close()'''
        			other_callbacks += '''\n\t\t\texit(0)'''
        			other_callbacks +='''\n\t\tif actions[json_dict['topic']][0] != 'filter':'''
        			other_callbacks +='''\n\t\t\t#if json_dict['verdict'] == 'currently_false':\n\t\t\t#rospy.loginfo('The event ' + message + ' is consistent ')'''
        			other_callbacks +='''\n\t\t\ttopic = json_dict['topic']'''
        			if self.oracle_action == 'nothing':
        				other_callbacks += '''\n\t\t\tif topic in pub_dict:\n\t\t\t\tpub_dict[topic].publish(dict_msgs[json_dict['time']])'''
        				other_callbacks += '''\n\t\t\tdel dict_msgs[json_dict['time']]'''
        			else:
        				other_callbacks += '''\n\t\t\tdel json_dict['topic']\n\t\t\tdel json_dict['time']'''
        				other_callbacks +='''\n\t\t\tdel json_dict['error']\n\t\t\tdel json_dict['spec']'''
        				other_callbacks +='''\n\t\t\tROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)'''
        				other_callbacks +='''\n\t\t\tif topic in pub_dict:\n\t\t\t\tpub_dict[topic].publish(ROS_message)'''
        			other_callbacks += '''\n\t\terror = True'''
        			other_callbacks += '''\n\tpub_verdict.publish(json_dict['verdict'])'''
        			#other_callbacks += '''\n\ndef on_message_request(msg):'''
        			
        	
        		else:
        			other_callbacks = ''
        		other_callbacks += '''\n\ndef logging(json_dict):\n\ttry:\n\t\twith open(log, 'a+') as log_file:\n\t\t\tlog_file.write(json.dumps(json_dict) + '\\n')'''
        		if not self.silent:
        			other_callbacks += '''\n\t\trospy.loginfo('event logged')'''
        		other_callbacks += '''\n\texcept:\n\t\trospy.loginfo('Unable to log the event.')'''
        		other_callbacks +='''\n\ndef main(argv):\n\tglobal log, actions, ws\n\tlog = '{l}' '''.format(l = self.log)
        		other_callbacks += '''\n\tactions = {'''
        		first_time = True
        		for topic_with_types_and_action in self.topics_with_types_and_action:
        		# if 'warning' in topic_with_types_and_action:
        		#     warning = topic_with_types_and_action['warning']
        		# else:
        		#     warning = 0
        			if(first_time):
        				first_time = False
        			else:
        				other_callbacks += ', '
        			other_callbacks += '''\n\t\t'{tp}' : ('{act}', {w})'''.format(tp = topic_with_types_and_action['name'], act = topic_with_types_and_action['action'], w = self.warning)
        		if self.url != None and self.port != None:
        			other_callbacks += '''\n\t}\n\tmonitor()\n\twebsocket.enableTrace(True)'''
        			other_callbacks += '''\n\tws = websocket.WebSocket()\n\tws.connect('ws://{u}:{p}')\n\trospy.loginfo('Websocket is open')\n\trospy.spin()'''.format(u = self.url, p = self.port)
        			
        			#other_callbacks += '''\n\tws = websocket.WebSocketApp(\n\t\t'ws://{u}:{p}',\n\t\ton_message = on_message,
  #          \n\t\ton_error = on_error,\n\t\ton_close = on_close,\n\t\ton_open = on_open)\n\tws.run_forever()'''.format(u = self.url, p = self.port)
        		else:
        			other_callbacks += '''\n\t}\n\tmonitor()\n\trospy.spin()'''
        		other_callbacks += '''\n\nif __name__ == '__main__':\n\tmain(sys.argv)'''
        		return other_callbacks

### [MGS]: add services_with_types_and_action to args
	def write_monitor(self):
        	# function which creates the python ROS monitor
        	with open(self.monitor_file, 'w') as monitor: # the monitor code will be in monitor.py
        		monitor.write(self.std_imports + self.topic_msg_type_imports + self.service_msg_type_imports + self.service_callbacks + self.pub_with_callbacks + self.pub_dict + self.msg_dict + self.monitor_def + self.other_callbacks)

	def write_launch_file(self):
		with open(self.launch_file, 'w') as launch_file:
        		content = '''<launch>\n'''
        		for id in self.monitor_ids:
        			content += '''<node pkg="monitor" type="{monitor_id}.py" name="{monitor_id}" output="screen"/>\n'''.format(monitor_id = id)
        		content += '''</launch>\n'''
        		launch_file.write(content)

	def instrument_launch_files(self):
        	# instrument the launch files through adding/removing remap params
        		if not self.nodes: return
      
        		launch_files = {}
        		for name in self.nodes:
        			(package, path, topics) = self.nodes[name]
        			if path not in launch_files:
        				launch_files[path] = []
        			launch_files[path].append((name, package, topics))
        		for path in launch_files:
        			file_name = path.replace('.launch', '_instrumented.launch')
        			tree = ET.parse(path)
        			launch = tree.getroot()
        			for node in launch.findall('node'):
        				for (name, package, topics) in launch_files[path]:
        					if node.get('name') == name and node.get('pkg') == package:
        						for topic in topics:
        							remap = ET.SubElement(node, 'remap')
        							remap.set('from', topic)
        							remap.set('to', topic + '_mon')
        						break
        			tree.write(file_name)
