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

	def __init__(self, monitor_id, topics_with_types_and_action, services_with_types_and_action, ordered_topics, log, url, port, oracle_action, silent, warning, ids, nodes):
		self.input_path = './auxiliary_files/'
		self.monitor_file = '../monitor/src/' + monitor_id + '.py'
		self.launch_file = '../monitor/run.launch'
		self.monitor_id = monitor_id
		self.topics_with_types_and_action = topics_with_types_and_action
		#added:
		self.services_with_types_and_action = services_with_types_and_action
		self.ordered_topics = ordered_topics
		self.log = log
		self.url = url
		self.port = port
		self.oracle_action = oracle_action
		self.silent = silent
		self.warning = warning
		self.monitor_ids = ids
		self.nodes = nodes
		
	def get_standard_imports(self):
		# get the imports the monitor will need
		f = open(os.path.join(self.input_path, 'standard_imports.txt'), 'r')
		std_imports = f.read()
		f.close()
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
		
	def get_initialisations(self):
		inits = '\n\nws_lock = Lock()'
		if self.oracle_action == 'nothing':
		    inits += '''\ndict_msgs = {}'''
		inits += '''\npub_dict = {}'''
		inits += '''\nsrv_type_dict = dict()'''
		return inits
		
	def get_reordering_inits_and_functions(self):
		reordering_inits_and_functions = '\n\ntopics_to_reorder = ["'+'", "'.join(self.ordered_topics)+'"]' 
		f = open(os.path.join(self.input_path, 'reordering_inits_and_functions.txt'), 'r')
		reordering_inits_and_functions += f.read()
		f.close()
		return reordering_inits_and_functions
		
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
            		msg_type_imports += '''\nfrom {p} import {t}, {t}Response'''.format(p = package, t = msg_type)
		return msg_type_imports

	def get_callback_per_service(self):
		callbacks = '\n'
		for service_with_types_and_action in self.services_with_types_and_action:
			
			srv_name = service_with_types_and_action['name']
			srv_type = service_with_types_and_action['type']
			callbacks += '''\ndef callback_{srv}(request):\n\tglobal ws, ws_lock'''.format(srv = srv_name+'_mon'.replace('/','_'))
			callbacks += '\n\td = dict()'
			if not self.silent:
				callbacks += '''\n\trospy.loginfo('monitor has observed: ' + str(request))'''
				
			callbacks += '''\n\td['request'] = message_converter.convert_ros_message_to_dictionary(request)\n\td['service'] = '{srv}'\n\td['time'] = rospy.get_time()\n\tws_lock.acquire()'''.format(srv = srv_name.replace('/','_'))
			if self.oracle_action == 'nothing':
				callbacks += '''\n\twhile d['time'] in dict_msgs:\n\t\td['time'] += 0.01'''
			if self.url != None and self.port != None:
				callbacks += '''\n\tws.send(json.dumps(d))'''
				if self.oracle_action == 'nothing':
					callbacks += '''\n\tdict_msgs[d['time']] = request'''
				callbacks += '\n\tmsg = ws.recv()\n'
				callbacks += '''\n\tws_lock.release()\n'''
				if not self.silent:
					callbacks += '''\n\trospy.loginfo('event propagated to oracle')\n'''
				
				callbacks += '''\n\ttry:\n\t\treturn on_message_service_request(msg)\n\texcept error:\n\t\tres = {t}Response()\n\t\tres.error = True\n\t\treturn res\n'''.format(t = srv_type[srv_type.rfind('.')+1:])

			else:
				callbacks += '''\n\tlogging(d)'''
				callbacks += '''\n\tws_lock.release()'''
				#if not self.silent:
				#	callbacks += '''\n\trospy.loginfo('event has been successfully logged')\n'''

		return callbacks
	
	def get_customised_send_earliest_msg_to_oracle_function(self):
		pub_with_callbacks = '''\ndef logEarliestMsg():\n\tglobal ws, ws_lock, data_dict, msgs_dict\n\tmin_time = min(list(msgs_dict.keys()))\n\td = msgs_dict[min_time]\n'''

		pub_with_callbacks += '''\n\td['time'] = rospy.get_time()\n\tws_lock.acquire()'''
		if self.oracle_action == 'nothing':
                		pub_with_callbacks += '''\n\twhile d['time'] in dict_msgs:\n\t\td['time'] += 0.01'''
            		
		if self.url != None and self.port != None:
                		pub_with_callbacks += '''\n\tws.send(json.dumps(d))'''
                		if self.oracle_action == 'nothing':
                			pub_with_callbacks += '''\n\tdict_msgs[d['time']] = data_dict[min_time]'''

                		pub_with_callbacks += '\n\tmsg = ws.recv()'
                		pub_with_callbacks += '''\n\tws_lock.release()\n'''
                		
                		pub_with_callbacks += '''\n\tfor topic in topics_to_reorder:\n\t\tif min_time in buffers[topic]:\n\t\t\tbuffers[topic].remove(min_time)\n\t\t\tbreak\n\tmsgs_dict.pop(min_time)\n\tdata_dict.pop(min_time)\n'''
                		
                		pub_with_callbacks += '''\n\treturn on_message_topic(msg)\n'''

		else:
            		pub_with_callbacks += '''\n\tlogging(d)'''
            		pub_with_callbacks += '''\n\tws_lock.release()\n'''
            		
            		pub_with_callbacks += '''\n\tfor topic in topics_to_reorder:\n\t\tif min_time in buffers[topic]:\n\t\t\tbuffers[topic].remove(min_time)\n\t\t\tbreak\n\tif min_time in msgs_dict.keys():\n\t\tmsgs_dict.pop(min_time)\n\tif min_time in data_dict.keys():\n\t\tdata_dict.pop(min_time)\n'''
            		
            		#if not self.silent:
                		#	pub_with_callbacks += '''\n\trospy.loginfo('event has been successfully logged')'''
            		#if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action:
            		#	pub_with_callbacks += '''\n\tpub_dict['{tp}'].publish(data)\n'''.format(tp = tp_name)
            		
		return pub_with_callbacks
	           	
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

			pub_with_callbacks += '''\ndef callback_{tp}(data):\n\tglobal ws, ws_lock'''.format(tp = tp_name.replace('/','_'))
			pub_with_callbacks += '''\n\td['topic'] = '{tp}' '''.format(tp = tp_name)
			if not self.silent:
            			pub_with_callbacks += '''\n\trospy.loginfo('monitor has observed the following message on topic '+d['topic']+': ' + str(data))'''
			pub_with_callbacks += '''\n\td = message_converter.convert_ros_message_to_dictionary(data)'''
			
			if tp_name in self.ordered_topics:
				pub_with_callbacks += '''\n\taddToBuffer(d['topic'], d, data)\n'''
			else:
				pub_with_callbacks += '''\n\td['time'] = rospy.get_time()\n\tws_lock.acquire()'''
				if self.oracle_action == 'nothing':
                				pub_with_callbacks += '''\n\twhile d['time'] in dict_msgs:\n\t\td['time'] += 0.01'''
            		
				if self.url != None and self.port != None:
                				pub_with_callbacks += '''\n\tws.send(json.dumps(d))'''
                				if self.oracle_action == 'nothing':
                					pub_with_callbacks += '''\n\tdict_msgs[d['time']] = data'''

                				pub_with_callbacks += '\n\tmsg = ws.recv()'
                				pub_with_callbacks += '''\n\tws_lock.release()\n'''
                				pub_with_callbacks += '''\n\treturn on_message_topic(msg)\n'''

				else:
            				pub_with_callbacks += '''\n\tlogging(d)'''
            				pub_with_callbacks += '''\n\tws_lock.release()\n'''
            				#if not self.silent:       #logging function takes care of this  
                				#	pub_with_callbacks += '''\n\trospy.loginfo('event has been successfully logged')\n'''
            				#if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action:
            				#	pub_with_callbacks += '''\n\tpub_dict['{tp}'].publish(data)\n'''.format(tp = tp_name)
            		
            		
		return pub_with_callbacks
      

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
        		monitor_def = '\n'+f.read().replace('$ID$', self.monitor_id)+'\n'
        		f.close()
        		for topic_with_types_and_action in self.topics_with_types_and_action:
        			if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action:
        				tp_name = topic_with_types_and_action['name'] 
        				tp_side = tp_name
        				if 'subscribers' in topic_with_types_and_action:
        					tp_side = tp_name + '_mon'
        			
        				tp_type = topic_with_types_and_action['type']
        				
        				monitor_def += '''\n\tpub{tp} = rospy.Publisher(name = '{tps}', data_class = {ty}, latch = True, queue_size = 1000)'''.format(tp = tp_name.replace('/','_'), tps = tp_side, ty = tp_type[tp_type.rfind('.')+1:])
        				monitor_def += '''\n\tpub_dict['{tp}'] = pub{tp}\n'''.format(tp = tp_name)
                        
        		for topic_with_types_and_action in self.topics_with_types_and_action:
        			tp_name = topic_with_types_and_action['name']
        			tp_type = topic_with_types_and_action['type']
        			tp_side = tp_name
        			if 'publishers' in topic_with_types_and_action:
        				tp_side += '_mon'

        			monitor_def += '''\n\trospy.Subscriber('{tps}', {ty}, callback_{tp})'''.format(tp = tp_name.replace('/','_'), tps = tp_side, ty = tp_type[tp_type.rfind('.')+1:])
        		
        		for service_with_types_and_action in self.services_with_types_and_action:
        			srv_name = service_with_types_and_action['name']
        			srv_type = service_with_types_and_action['type']
        			srv_side = srv_name + '_mon'
        			monitor_def += '''\n\tsrv_type_dict['{srv}'] = {typ}'''.format(srv = srv_name, typ = srv_type[srv_type.rfind('.')+1:])

        			monitor_def += '''\n\trospy.Service('{srvs}', {ty}, callback_{srv})'''.format(srv = srv_side.replace('/','_'), srvs = srv_side, ty = srv_type[srv_type.rfind('.')+1:])
        			        		
        		
        		if not self.silent:
        			monitor_def += '''\n\trospy.loginfo('monitor started and ready')'''
        		return monitor_def    

	def get_on_message_topic(self):
        		f = open(os.path.join(self.input_path, 'on_message_topic.txt'), 'r')
        		on_msg_topic = '\n\n'+f.read()
        		f.close()
        		on_msg_topic+= '''\tif verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':'''
        		on_msg_topic+= '''\n\t\tif verdict == 'true' and not pub_dict:\n\t\t\trospy.loginfo('The monitor concluded the satisfaction of the property under analysis, and can be safely removed.')'''
        		on_msg_topic+= '''\n\t\t\tws.close()\n\t\t\texit(0)'''
        		on_msg_topic+= '''\n\t\telse:\n\t\t\tlogging(json_dict)'''
        		
        		if not self.silent:
        			on_msg_topic += '''\n\t\t\trospy.loginfo('The event ' + message + ' is consistent and republished')'''
        		if self.oracle_action == 'nothing':
        			on_msg_topic += '''\n\t\t\tif topic in pub_dict:\n\t\t\t\tpub_dict[topic].publish(msg)\n\t\t\tdel dict_msgs[json_dict['time']]'''
        		else:
        			on_msg_topic += '''\n\t\t\tdel json_dict['topic']\n\t\t\tdel json_dict['time']\n\t\t\tROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)'''
        			on_msg_topic += '''\n\t\t\tif topic in pub_dict:\n\t\t\t\tpub_dict[topic].publish(ROS_message)'''
        		on_msg_topic += '''\n\telse:\n\t\tlogging(json_dict)'''

        		if not self.silent:
        			on_msg_topic += '''\n\t\trospy.loginfo('The event ' + message + ' is inconsistent.')'''
        				
        		on_msg_topic += "\n\t\tpublish_error('topic', topic, json_dict)"	
        			
        		on_msg_topic += '''\n\t\tif verdict == 'false' and not pub_dict:'''
        		on_msg_topic += '''\n\t\t\trospy.loginfo('The monitor concluded the violation of the property under analysis, and can be safely removed.')\n\t\t\tws.close()'''
        		on_msg_topic += '''\n\t\t\texit(0)'''
        		on_msg_topic +='''\n\t\tif actions[topic][0] != 'filter':'''
        			
        		if self.oracle_action == 'nothing':
        			on_msg_topic += '''\n\t\t\tif topic in pub_dict:\n\t\t\t\tpub_dict[topic].publish(msg)'''
        			on_msg_topic += '''\n\t\t\tdel dict_msgs[json_dict['time']]'''
        		else:
        			on_msg_topic += '''\n\t\t\tdel json_dict['topic']\n\t\t\tdel json_dict['time']'''
        			on_msg_topic +='''\n\t\t\tdel json_dict['error']\n\t\t\tdel json_dict['spec']'''
        			on_msg_topic +='''\n\t\t\tROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)'''
        			on_msg_topic +='''\n\t\t\tif topic in pub_dict:\n\t\t\t\tpub_dict[topic].publish(ROS_message)'''
        		on_msg_topic += '''\n\tpublish_verdict(verdict)'''
       	


        		return on_msg_topic

	def get_on_message_service_request(self):
        		f = open(os.path.join(self.input_path, 'on_message_service_request.txt'), 'r')
        		on_msg_request = '\n\n'+f.read()
        		f.close()
        		on_msg_request+= '''\tif verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':'''
        		on_msg_request+= '''\n\t\tlogging(json_dict)'''
        		
        		if not self.silent:
        			on_msg_request += '''\n\t\trospy.loginfo('The event ' + message + ' is consistent and the service', str(service), 'is called.')'''
        		on_msg_request +='''\n\t\tdel json_dict['verdict']'''
        		
        		on_msg_request +='''\n\t\tcall_service(service, srv_type_dict[service], json_dict)'''
        		on_msg_request +='''\n\t\tdel json_dict['request']'''
        		on_msg_request +='''\n\t\tmsg = get_oracle_verdict(json_dict)\n\t\treturn on_message_service_response(msg)'''
        		on_msg_request +='''\n\telse:\n\t\tlogging(json_dict)'''

        		if not self.silent:
        			on_msg_request += '''\n\t\trospy.loginfo('The request ' + message + ' is inconsistent.')'''
        				
        		on_msg_request += "\n\t\tpublish_error('service', service, json_dict)"	
        		on_msg_request += '''\n\t\tpublish_verdict(verdict)'''	
        		
        		on_msg_request +='''\n\t\tif actions[service][0] != 'filter':'''
        		on_msg_request +='''\n\t\t\tif 'verdict' in json_dict: del json_dict['verdict']'''
        		on_msg_request +='''\n\t\t\tcall_service(service, srv_type_dict[service], json_dict)'''
        		on_msg_request +='''\n\t\t\tdel json_dict['request']'''
        		on_msg_request +='''\n\t\t\tmsg = get_oracle_verdict(json_dict)\n\t\t\treturn on_message_service_response(msg)'''
        		on_msg_request +='''\n\t\telse:\n\t\t\traise Exception('The request violates the monitor specification, so it has been filtered out.')'''	
        			       		
        		return on_msg_request

	def get_on_message_service_response(self):
        		f = open(os.path.join(self.input_path, 'on_message_service_response.txt'), 'r')
        		on_msg_response = '\n\n'+f.read()
        		f.close()
        		on_msg_response+= '''\tif verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':'''
        		on_msg_response+= '''\n\t\tlogging(json_dict)'''
        		
        		if not self.silent:
        			on_msg_response += '''\n\t\trospy.loginfo('The response ' + message + ' is consistent, the result is returned.')'''
        		on_msg_response +='''\n\t\treturn dict_msgs[json_dict['time']]'''
        		
        		on_msg_response +='''\n\telse:\n\t\tlogging(json_dict)'''

        		if not self.silent:
        			on_msg_response += '''\n\t\trospy.loginfo('The response ' + message + ' is inconsistent.')'''
        				
        		on_msg_response += "\n\t\tpublish_error('service', service, json_dict)"	
        		on_msg_response += '''\n\t\tpublish_verdict(verdict)'''	
        		
        		on_msg_response +='''\n\t\tif actions[service][0] != 'filter':'''
        		on_msg_response +='''\n\t\t\treturn dict_msgs[json_dict['time']]'''
        		on_msg_response +='''\n\t\telse:\n\t\t\traise Exception('The request violates the monitor specification, so it has been filtered out.')'''	
        			       		
        		return on_msg_response
        		        		
	def get_main_function(self):
        		main_fct ='''\n\ndef main(argv):\n\tglobal log, actions, ws\n\tlog = '{l}' '''.format(l = self.log)
        		main_fct += '''\n\tactions = {'''
        		first_time = True
        		for topic_with_types_and_action in self.topics_with_types_and_action:

        			if(first_time):
        				first_time = False
        			else:
        				main_fct += ', '
        			main_fct += '''\n\t\t'{tp}' : ('{act}', {w})'''.format(tp = topic_with_types_and_action['name'], act = topic_with_types_and_action['action'], w = self.warning)
        			
        		for service_with_types_and_action in self.services_with_types_and_action:
        			main_fct += ', '
        			main_fct += '''\n\t\t'{srv}' : ('{act}', {w})'''.format(srv = service_with_types_and_action['name'], act = service_with_types_and_action['action'], w = self.warning)
        			
        		if self.url != None and self.port != None:
        			main_fct += '''\n\t}\n\twebsocket.enableTrace(True)'''
        			main_fct += '''\n\tws = websocket.WebSocket()\n\tws.connect('ws://{u}:{p}')\n\trospy.loginfo('Websocket is open')\n\tmonitor()\n\trospy.spin()'''.format(u = self.url, p = self.port)
        			
        		else:
        			main_fct += '''\n\t}\n\tmonitor()\n\trospy.spin()'''
        		main_fct += '''\n\nif __name__ == '__main__':\n\tmain(sys.argv)'''
        		return main_fct
        		
	def get_logging_function(self):
        		fct = '''\ndef logging(json_dict):\n\ttry:\n\t\twith open(log, 'a+') as log_file:\n\t\t\tlog_file.write(json.dumps(json_dict) + '\\n')'''
        		if not self.silent:
        			fct += '''\n\t\trospy.loginfo('event logged')'''
        		fct += '''\n\texcept:\n\t\trospy.loginfo('Unable to log the event.')'''
        		return fct
        			
	def get_functions(self):
		f = open(os.path.join(self.input_path, 'functions.txt'), 'r')
		functions = '\n\n'+f.read()
		f.close()
		
		return functions

### [MGS]: add services_with_types_and_action to args
	def write_monitor(self):
        	# function which creates the python ROS monitor
        	with open(self.monitor_file, 'w') as monitor: # the monitor code will be in monitor.py
        		#if self.url != None and self.port != None:
        			# + self.get_dict_for_service_types()
        		monitor.write(self.get_standard_imports() + self.get_imports_for_topic_msg_types() + self.get_imports_for_service_msg_types() + self.get_initialisations() +  self.get_dict_for_msg_types() + self.get_reordering_inits_and_functions() + self.get_customised_send_earliest_msg_to_oracle_function() + self.get_publisher_per_topic() + self.get_callback_per_service() + self.get_mon_node() + self.get_on_message_topic() + self.get_on_message_service_request() + self.get_on_message_service_response() + self.get_functions() + self.get_logging_function() + self.get_main_function())

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
