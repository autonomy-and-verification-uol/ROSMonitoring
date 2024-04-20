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

	def __init__(self, monitor_id, topics_with_types_and_action, services_with_types_and_action, ordered_topics, ordered_services, log, url, port, oracle_action, silent, warning, ids, nodes):
		self.input_path = './auxiliary_files/'
		self.monitor_file = '../monitor/src/' + monitor_id + '.py'
		self.launch_file = '../monitor/run_{id}.launch'.format(id = monitor_id)
		self.monitor_id = monitor_id
		self.topics_with_types_and_action = self.get_list_with_names_with_slash(topics_with_types_and_action)#topics_with_types_and_action
		
		#added:
		self.services_with_types_and_action = self.get_list_with_names_with_slash(services_with_types_and_action) #services_with_types_and_action
					
		self.ordered_topics = [self.getNameWithSlash(t) for t in ordered_topics]
		self.ordered_services = [self.getNameWithSlash(t) for t in ordered_services]
		self.ordered_interfaces = self.ordered_topics + self.ordered_services
		self.log = log
		self.url = url
		self.port = port
		self.oracle_action = oracle_action
		self.silent = silent
		self.warning = warning
		self.monitor_ids = ids
		self.nodes = nodes
		self.topics_to_republish = [topic_with_types_and_action['name'] for topic_with_types_and_action in self.topics_with_types_and_action if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action]
		self.offline = self.url == None and self.port == None
	
	def get_service_type(self, srv_name):
		for service_with_types_and_action in self.services_with_types_and_action:
			if service_with_types_and_action['name'] == srv_name:
				return service_with_types_and_action['type']
				
	def get_list_with_names_with_slash(self, l):
		new_l = []
		for l_dict in l:
			new_dict = dict()
			for k, v in l_dict.items():
				if k == 'name':
					new_dict[k] = self.getNameWithSlash(v)
				else:
					new_dict[k] = v
			new_l.append(new_dict)
		return new_l
					
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
		inits = '\n\nws_lock = RLock()'
		if self.oracle_action == 'nothing':
		    inits += '''\ndict_msgs = {}'''
		inits += '''\npub_dict = {}'''
		inits += '''\nsrv_type_dict = dict()'''
		tps_to_republish = ''
		if self.topics_to_republish:
			tps_to_republish = '"'+'", "'.join(self.topics_to_republish)+'"'
		inits += '\ntopics_to_republish = ['+tps_to_republish+']'
		
		return inits
		
	def get_reordering_inits_and_functions(self):
		topics_to_reorder = ''
		if self.ordered_topics:
			 topics_to_reorder = '"'+'", "'.join(self.ordered_topics)+'"'
		services_to_reorder = ''
		if self.ordered_services:
			 services_to_reorder = '"'+'", "'.join(self.ordered_services)+'"'
		reordering_inits_and_functions = '\n\ntopics_to_reorder = ['+topics_to_reorder+']' 
		reordering_inits_and_functions += '\n\nservices_to_reorder = ['+services_to_reorder+']'
		reordering_inits_and_functions += '\n\ninterfaces_to_reorder = topics_to_reorder + services_to_reorder'
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
		
	def get_callservice_fct(self):
		if self.offline:
			s = '''\ndef call_service(service, msgType, d, request):\n\tsrv = rospy.ServiceProxy(service, msgType)\n\tresponse = srv(request)'''
			if not self.silent:
				s += '''\n\trospy.loginfo('The service '+str(service)+' has been called.')'''
			s += '''\n\treturn response\n'''
			return s
		else:
			s = '''\ndef call_service(service, msgType, json_dict):\n\tglobal dict_msgs\n\trequest = dict_msgs[json_dict['time']]\n\tsrv = rospy.ServiceProxy(service, msgType)\n\tresponse = srv(request)'''
			if not self.silent:
				s += '''\n\trospy.loginfo('The service '+str(service)+' has been called.')'''
			s += '''\n\tjson_dict['response'] = message_converter.convert_ros_message_to_dictionary(response)\n\tdict_msgs[json_dict['time']] = response'''
			if self.ordered_services:
				s += '''\n\tif service in services_to_reorder:\n\t\tsrv_res[service][getTime(json_dict)] = (response, False)'''
			s += '''\n\treturn json_dict
'''
			return s
	
	def get_callback_per_service(self):
		callbacks = '\n'
		for service_with_types_and_action in self.services_with_types_and_action:
			
			srv_name = service_with_types_and_action['name']				
			srv_type = service_with_types_and_action['type']
			callbacks += '''\ndef callback_{srv}(request):'''.format(srv = (srv_name+'_mon').replace('/','_'))
			
			if self.offline:
				callbacks += '''\n\tglobal ws, ws_lock, dict_msgs'''
			else:
				callbacks += '''\n\tglobal ws, ws_lock, dict_msgs, srv_res'''
			
			callbacks += '''\n\td = dict()'''
			if not self.silent:
				callbacks += '''\n\trospy.loginfo('monitor has observed: ' + str(request))'''
				
			callbacks += '''\n\td['request'] = message_converter.convert_ros_message_to_dictionary(request)'''
			
			callbacks += "\n\td['service'] = '{srv}'\n\td['stamp'] = d['request']['stamp']\n\td['time'] = rospy.get_time()".format(srv = srv_name)
			
			
			
			if not self.offline: #online
				if not srv_name in self.ordered_services: #online, service not to be reordered
					callbacks += '''\n\tws_lock.acquire()'''
					if self.oracle_action == 'nothing':
						callbacks += '''\n\twhile d['time'] in dict_msgs:\n\t\td['time'] += 0.01'''
				
					callbacks += '''\n\tws.send(json.dumps(d))'''
					if self.oracle_action == 'nothing':
						callbacks += '''\n\tdict_msgs[d['time']] = request'''
					callbacks += '\n\tmsg = ws.recv()'
					callbacks += '''\n\tws_lock.release()'''
					if not self.silent:
						callbacks += '''\n\trospy.loginfo('event propagated to oracle')\n'''
				
					callbacks += '''\n\ttry:\n\t\treturn on_message_service_request(msg)\n\texcept Exception as e:\n\t\trospy.loginfo('Exception: '+str(e))\n\t\tres = {srv}Response()\n\t\tres.error = True\n\t\treturn res\n\n'''.format(srv = srv_type[srv_type.rfind('.')+1:])
				else: #online, service is to be reordered
					callbacks += '''\n\tws_lock.acquire()'''
					callbacks += '''\n\tif not d['service'] in srv_res:\n\t\tsrv_res[d['service']] = dict()'''
					#if self.oracle_action == 'nothing':
					
					callbacks += '''\n\twhile getTime(d) in srv_res[d['service']]:\n\t\td['time'] += 0.01'''
					
					callbacks += '''\n\tsrv_res[d['service']][getTime(d)] = ('None', False)\n\tws_lock.release() '''

					callbacks += '''\n\taddToBuffer(d['service'], d, request)\n\twhile srv_res[d['service']][getTime(d)][1] == False:\n\t\tpass'''
					callbacks += '''\n\tws_lock.acquire()\n\tt = getTime(d)\n\tresponse = srv_res[d['service']][t][0]\n\tsrv_res[d['service']].pop(t)\n\tws_lock.release()\n\treturn response'''

			else:
				if srv_name in self.ordered_services:
					callbacks += '''\n\taddToBuffer(d['service'], d, str(request))'''
				else:
					callbacks += '''\n\tlogging(d)'''
				callbacks += '''\n\tservice = d['service']\n\trospy.wait_for_service(service)'''
				callbacks += '''\n\ttry:\n\t\tresponse = call_service(service, srv_type_dict[service], d, request)\n\t\tresDict = message_converter.convert_ros_message_to_dictionary(response)'''
				callbacks += '''\n\t\td = dict()\n\t\td['response'] = resDict\n\t\td['service'] = '{srv}'\n\t\td['stamp'] = d['response']['stamp']'''.format(srv = srv_name)
				if srv_name in self.ordered_services:
					callbacks += '''\n\t\taddToBuffer(d['service'], d, str(response))'''
				else:
					callbacks += '''\n\t\tlogging(d)'''
				callbacks += '''\n\t\treturn response\n\texcept Exception as e:\n\t\trospy.loginfo('Exception: '+str(e))\n\t\tres = {srv}Response()\n\t\tres.error = True\n\t\treturn res\n\n'''.format(srv = srv_type[srv_type.rfind('.')+1:])
				#callbacks += '''\n\ttry:\n\t\treturn on_message_service_request(msg)\n\texcept:\n\t\tres = {t}Response()\n\t\tres.error = True\n\t\treturn res\n'''.format(t = srv_type[srv_type.rfind('.')+1:])
				#if not self.silent:  #logging function takes care of this 
				#	callbacks += '''\n\trospy.loginfo('event has been successfully logged')\n'''

		return callbacks+'\n'
	
	def get_customised_send_earliest_msg_to_oracle_function(self):
		pub_with_callbacks = '''\ndef logEarliestMsg():\n\tglobal ws_lock, data_dict, msgs_dict, buffers\n\tmin_time = min(list(msgs_dict.keys()))\n\td = msgs_dict[min_time]\n'''

		pub_with_callbacks += '''\n\td['time'] = rospy.get_time()'''
		if self.oracle_action == 'nothing':
                		pub_with_callbacks += '''\n\twhile d['time'] in dict_msgs:\n\t\td['time'] += 0.01'''
            		
		if not self.offline:
                		pub_with_callbacks += '''\n\tws.send(json.dumps(d))'''
                		if self.oracle_action == 'nothing':
                			pub_with_callbacks += '''\n\tdict_msgs[d['time']] = data_dict[min_time]'''

                		pub_with_callbacks += '\n\tmsg = ws.recv()'
                		
                		if not self.silent:
                			pub_with_callbacks += '''\n\trospy.loginfo('event propagated to oracle')'''
                		
                		pub_with_callbacks += '''\n\tfor interface in interfaces_to_reorder:\n\t\tif min_time in buffers[interface]:\n\t\t\tbuffers[interface].remove(min_time)\n\t\t\tbreak\n\tmsgs_dict.pop(min_time)\n\tdata_dict.pop(min_time)\n'''
                		
                		pub_with_callbacks += '''\n\treturn d, msg'''
              		
                		pub_with_callbacks += '''\n\n\ndef on_message_function_call(d, msg):'''
                		
                		if self.ordered_topics:
                			pub_with_callbacks += '''\n\tif 'topic' in d.keys():\n\t\ton_message_topic(msg)\n'''
                		
                		if self.ordered_services:
                			pub_with_callbacks += '''\n\tif 'service' in d.keys():'''
                		
                			
                			pub_with_callbacks += '''\n\t\tif 'request' in d.keys():\n\t\t\ttry:\n\t\t\t\ton_message_service_request(msg)\n\t\t\texcept Exception as e:\n\t\t\t\trospy.loginfo('Exception: '+str(e))'''
                			for service in self.ordered_services:
                				srv_type = self.get_service_type(service)
                				pub_with_callbacks += '''\n\t\t\t\tif d['service'] == '{srv}':\n\t\t\t\t\tres = {srv_type}Response()'''.format(srv = service, srv_type = srv_type[srv_type.rfind('.')+1:])
                				
                			pub_with_callbacks += '''\n\t\t\t\tres.error = True\n\t\t\t\treturn res'''
                			
                			pub_with_callbacks += '''\n\t\tif 'response' in d.keys():\n\t\t\ton_message_service_response(msg)\n'''

		else:
                		pub_with_callbacks += '''\n\tlogging(d)'''
            		
                		pub_with_callbacks += '''\n\tfor interface in interfaces_to_reorder:\n\t\tif min_time in buffers[interface]:\n\t\t\tbuffers[interface].remove(min_time)\n\t\t\tbreak\n\tmsgs_dict.pop(min_time)\n\tdata_dict.pop(min_time)\n'''
                		
                		pub_with_callbacks += '''\n\tws_lock.release()\n'''
                		
            		#if self.topics_to_republish: only for filtering which is only available for online monitoring
            		#	pub_with_callbacks += '''\n\tif topic in topics_to_republish:\n\t\tpub_dict[topic].publish(data)\n'''
            		#else:
            		#	pub_with_callbacks += '\n'
            		            		             		
            		#if not self.silent:  #logging function takes care of this 
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

			pub_with_callbacks += '''\ndef callback_{tp}(data):'''.format(tp = tp_name.replace('/','_'))
			
			if not (tp_name in self.ordered_topics) and not self.offline:
				pub_with_callbacks += '''\n\tglobal ws, ws_lock'''
			
			pub_with_callbacks += '''\n\td = message_converter.convert_ros_message_to_dictionary(data)'''
			pub_with_callbacks += '''\n\td['topic'] = '{tp}' '''.format(tp = tp_name)
			if not self.silent:
            			pub_with_callbacks += '''\n\trospy.loginfo('monitor has observed the following message on topic '+d['topic']+ ":\\n" + str(data))'''	
			
			if tp_name in self.ordered_topics:
				#action_is_filter = ('publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action)
				pub_with_callbacks += '''\n\taddToBuffer(d['topic'], d, data)\n'''
			else:
				pub_with_callbacks += '''\n\td['time'] = rospy.get_time()\n\tws_lock.acquire()'''
				if self.oracle_action == 'nothing':
                				pub_with_callbacks += '''\n\twhile d['time'] in dict_msgs:\n\t\td['time'] += 0.01'''
            		
				if not self.offline:
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
            				#if tp_name in self.topics_to_republish: #irrelevant to offline monitoring? 
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
        		monitor_def = '\n'+f.read()+'\n'
        		f.close()
        		if self.ordered_interfaces:
        			monitor_def += "\trospy.init_node('{mon_id}', anonymous=True, disable_signals=True)".format(mon_id = self.monitor_id)
        			monitor_def += "\n\trospy.on_shutdown(shutdownhook)"
        		else:
        			monitor_def += "\trospy.init_node('{mon_id}', anonymous=True)".format(mon_id = self.monitor_id)
        		monitor_def += "\n\tpub_error = rospy.Publisher(name = '{mon_id}/monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)".format(mon_id = self.monitor_id)
        		monitor_def += "\n\tpub_verdict = rospy.Publisher(name = '{mon_id}/monitor_verdict', data_class = String, latch = True, queue_size = 1000)".format(mon_id = self.monitor_id)
		
        		for topic_with_types_and_action in self.topics_with_types_and_action:
        			tp_name = topic_with_types_and_action['name'] 
        			#if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action:
        			if tp_name in self.topics_to_republish:
        				tp_side = tp_name
        				if 'subscribers' in topic_with_types_and_action:
        					tp_side = tp_name + '_mon'
        			
        				tp_type = topic_with_types_and_action['type']
        				
        				monitor_def += '''\n\tpub{tp} = rospy.Publisher(name = '{tps}', data_class = {ty}, latch = True, queue_size = 1000)'''.format(tp = tp_name.replace('/','_'), tps = tp_side, ty = tp_type[tp_type.rfind('.')+1:])
        				monitor_def += '''\n\tpub_dict['{tp}'] = pub{tp_without_slash}\n'''.format(tp = tp_name, tp_without_slash = tp_name.replace('/','_'))
                        			
        		for topic_with_types_and_action in self.topics_with_types_and_action:
        			tp_name = topic_with_types_and_action['name']
        			tp_type = topic_with_types_and_action['type']
        			tp_side = tp_name
        			if 'publishers' in topic_with_types_and_action:
        				tp_side += '_mon'

        			monitor_def += '''\n\trospy.Subscriber('{tps}', {ty}, callback_{tp})'''.format(tp = tp_name.replace('/','_'), tps = tp_side, ty = tp_type[tp_type.rfind('.')+1:])
        		
        		for service_with_types_and_action in self.services_with_types_and_action:
        			srv_name = service_with_types_and_action['name']
        			if srv_name[0] != '/':
        				srv_name = '/'+srv_name
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
        		on_msg_request += '''\tlogging(json_dict)'''
		
        		on_msg_request+= '''\n\tif verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':'''
        		
        		if not self.silent:
        			on_msg_request += '''\n\t\trospy.loginfo('The event ' + message + ' is consistent and the service '+ str(service)+' is called.')'''
        		on_msg_request +='''\n\t\tdel json_dict['verdict']'''
        		on_msg_request +='''\n\t\trospy.wait_for_service(service)'''
        		on_msg_request +='''\n\t\tjson_dict = call_service(service, srv_type_dict[service], json_dict)'''
        		on_msg_request +='''\n\t\tdel json_dict['request']'''
        		
        		if self.ordered_services:
        			on_msg_request += '''\n\t\tif service in services_to_reorder:\n\t\t\taddToBuffer(service, json_dict, str(json_dict['response']))\n\t\telse:\n\t\t\tmsg = get_oracle_verdict(json_dict)\n\t\t\treturn on_message_service_response(msg)'''
        		else:
        			on_msg_request +='''\n\t\tmsg = get_oracle_verdict(json_dict)\n\t\treturn on_message_service_response(msg)'''
        		
        		on_msg_request +='''\n\telse:'''

        		if not self.silent:
        			on_msg_request += '''\n\t\trospy.loginfo('The request ' + message + ' is inconsistent.')'''
        				
        		on_msg_request += "\n\t\tpublish_error('service', service, json_dict)"	
        		on_msg_request += '''\n\t\tpublish_verdict(verdict)'''	
        		
        		on_msg_request +='''\n\t\tif actions[service][0] != 'filter':'''
        		on_msg_request +='''\n\t\t\tif 'verdict' in json_dict: del json_dict['verdict']'''
        		on_msg_request +='''\n\t\t\trospy.wait_for_service(service)'''
        		on_msg_request +='''\n\t\t\tjson_dict = call_service(service, srv_type_dict[service], json_dict)'''
        		on_msg_request +='''\n\t\t\tdel json_dict['request']'''
        		
        		if self.ordered_services:
        			on_msg_request += '''\n\t\t\tif service in services_to_reorder:\n\t\t\t\taddToBuffer(service, json_dict, str(json_dict['response']))\n\t\t\telse:\n\t\t\t\tmsg = get_oracle_verdict(json_dict)\n\t\t\t\treturn on_message_service_response(msg)'''
        		else:
        			on_msg_request += '''\n\t\t\tmsg = get_oracle_verdict(json_dict)\n\t\t\treturn on_message_service_response(msg)'''

        		on_msg_request += '''\n\t\telse:'''
        		
        		for service_with_types_and_action in self.services_with_types_and_action:
        			srv_name = service_with_types_and_action['name']
        			srv_type = service_with_types_and_action['type']
        			on_msg_request += '''\n\t\t\tif service == '{srv}':\n\t\t\t\t\tres = {srv_type}Response()'''.format(srv = srv_name, srv_type = srv_type[srv_type.rfind('.')+1:])
                				
        		on_msg_request += '''\n\t\t\tres.error = True\n\t\t\tdict_msgs[json_dict['time']] = res'''
        		if self.ordered_services:
        			on_msg_request += '''\n\t\t\tif service in services_to_reorder:\n\t\t\t\tsrv_res[service][getTime(json_dict)] = (res, True)'''
        		on_msg_request +='''\n\t\t\traise Exception('The request violates the monitor specification, so it has been filtered out.')\n\t\t\treturn res'''	
        			       		
        		return on_msg_request

	def get_on_message_service_response(self):
        		f = open(os.path.join(self.input_path, 'on_message_service_response.txt'), 'r')
        		on_msg_response = '\n\n'+f.read()
        		f.close()
        		
        		on_msg_response += '''\tlogging(json_dict)'''
        			
        		on_msg_response+= '''\n\tif verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':'''
        		
        		if not self.silent:
        			on_msg_response += '''\n\t\trospy.loginfo('The response ' + message + ' is consistent, the result is returned.')'''
        		
        		if self.ordered_services:
        			on_msg_response += '''\n\t\tif service in services_to_reorder:\n\t\t\tres = srv_res[service][getTime(json_dict)][0]\n\t\t\tsrv_res[service][getTime(json_dict)] = (res, True)'''
        		
        		on_msg_response +='''\n\t\treturn dict_msgs[json_dict['time']]'''
        		
        		on_msg_response +='''\n\telse:'''

        		if not self.silent:
        			on_msg_response += '''\n\t\trospy.loginfo('The response ' + message + ' is inconsistent.')'''
        				
        		on_msg_response += "\n\t\tpublish_error('service', service, json_dict)"	
        		on_msg_response += '''\n\t\tpublish_verdict(verdict)'''	
        		
        		on_msg_response +='''\n\t\tif actions[service][0] != 'filter':'''
        		
        		if self.ordered_services:
        			on_msg_response +='''\n\t\t\tif service in services_to_reorder:\n\t\t\t\tres = srv_res[service][getTime(json_dict)][0]\n\t\t\t\tsrv_res[service][getTime(json_dict)] = (res, True)'''
        		
        		on_msg_response +='''\n\t\t\treturn dict_msgs[json_dict['time']]'''
        		
        		on_msg_response += '''\n\t\telse:'''
        		
        		for service_with_types_and_action in self.services_with_types_and_action:
        			srv_name = service_with_types_and_action['name']
        			srv_type = service_with_types_and_action['type']
        			on_msg_response += '''\n\t\t\tif service == '{srv}':\n\t\t\t\t\tres = {srv_type}Response()'''.format(srv = srv_name, srv_type = srv_type[srv_type.rfind('.')+1:])
                				
        		on_msg_response += '''\n\t\t\tres.error = True\n\t\t\tdict_msgs[json_dict['time']] = res'''
        		
        		if self.ordered_services:
        			on_msg_response += '''\n\t\t\tif service in services_to_reorder:\n\t\t\t\tsrv_res[service][getTime(json_dict)] = (res, True)'''
        			
        		on_msg_response +='''\n\t\t\traise Exception('The response violates the monitor specification, so it has been filtered out.')\n\t\t\treturn res'''	
        			       		
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
        			srv_name = service_with_types_and_action['name'] 
        			#if srv_name[0] != '/':
        			#	srv_name = '/'+srv_name
        			main_fct += ', '
        			main_fct += '''\n\t\t'{srv}' : ('{act}', {w})'''.format(srv = srv_name, act = service_with_types_and_action['action'], w = self.warning)
        			
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
        		fct += '''\n\texcept Exception as e:\n\t\trospy.loginfo('Exception: '+str(e)+'\\nUnable to log the event.')'''
        		return fct
        			
	def get_pub_error_function(self):
		f = open(os.path.join(self.input_path, 'pub_error_function.txt'), 'r')
		functions = '\n\n'+f.read()
		f.close()		
		return functions

	def get_verdict_functions(self):
		f = open(os.path.join(self.input_path, 'verdict_functions.txt'), 'r')
		functions = '\n\n'+f.read()
		f.close()
		return functions

	def get_shutdownhook_function(self):
		f = open(os.path.join(self.input_path, 'shutdownhook.txt'), 'r')
		function = '\n\n'+f.read()
		f.close()
		return function
			
	def getNameWithSlash(self, name):
		if name[0] == '/':
			return name
		return '/'+name
		
	def getNameWithoutSlash(self, name):
		if name[0] == '/':
			return name[1:]
		return name
			
### [MGS]: add services_with_types_and_action to args
	def write_monitor(self):
        	# function which creates the python ROS monitor
        	with open(self.monitor_file, 'w') as monitor: # the monitor code will be in monitor.py
        		#if self.url != None and self.port != None:
        			# + self.get_dict_for_service_types()
        		monitor.write(self.get_standard_imports())
        		monitor.write(self.get_imports_for_topic_msg_types())
        		monitor.write(self.get_imports_for_service_msg_types())
        		monitor.write(self.get_initialisations())
        		monitor.write(self.get_dict_for_msg_types())
        		if self.ordered_interfaces:
        			monitor.write(self.get_reordering_inits_and_functions()) 
        			monitor.write(self.get_customised_send_earliest_msg_to_oracle_function())
        		if self.topics_with_types_and_action:
        			monitor.write(self.get_publisher_per_topic())
        		
        		if self.services_with_types_and_action:
        			monitor.write(self.get_callback_per_service())
        			monitor.write(self.get_callservice_fct())
        		
        		monitor.write(self.get_mon_node())
        		
        		if self.topics_with_types_and_action:
        			monitor.write(self.get_on_message_topic())
        		
        		if not self.offline:
        			monitor.write(self.get_on_message_service_request())
        			monitor.write(self.get_on_message_service_response())
        			monitor.write(self.get_verdict_functions())
        			
        		if self.ordered_interfaces:
        			monitor.write(self.get_shutdownhook_function())
        			
        		monitor.write(self.get_pub_error_function())
        		monitor.write(self.get_logging_function())
        		monitor.write(self.get_main_function())

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
