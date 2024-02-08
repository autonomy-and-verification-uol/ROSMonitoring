#!/usr/bin/env python

# begin imports
import json
import yaml
import websocket
import sys
import rclpy
import rosidl_runtime_py
from rclpy.node import Node
from threading import *
from rosmonitoring_interfaces.msg import MonitorError
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from example_interfaces.srv import *
# done import

class ROSMonitor_monitor_0(Node):


	def callbackchatter(self,data):
		self.get_logger().info("monitor has observed "+ str(data))
		dict= rosidl_runtime_py.message_to_ordereddict(data)
		dict['topic']='chatter'
		dict['time']=float(self.get_clock().now().to_msg().sec)
		self.ws_lock.acquire()
		while dict['time'] in self.dict_msgs:
			dict['time']+=0.01
		self.ws.send(json.dumps(dict))
		self.dict_msgs[dict['time']] = data
		message=self.ws.recv()
		self.ws_lock.release()
		self.get_logger().info("event propagated to oracle")
		self.on_message_topic(message)

	def callbackadd_two_ints_mon(self, request, response):
		self.get_logger().info("monitor has observed a service request with "+ str(request))
		dict = {}
		dict['request']= rosidl_runtime_py.message_to_ordereddict(request)
		dict['service']='add_two_ints'
		dict['time']=float(self.get_clock().now().to_msg().sec)
		self.ws_lock.acquire()
		while dict['time'] in self.dict_msgs:
			dict['time']+=0.01
		self.ws.send(json.dumps(dict))
		self.dict_msgs[dict['time']] = request
		message=self.ws.recv()
		self.ws_lock.release()
		self.get_logger().info("event propagated to oracle")
		try:
			return self.on_message_service_request(message)
		except:
			response.error = True
			return response

	def callbackadd_two_ints_1_mon(self, request, response):
		self.get_logger().info("monitor has observed a service request with "+ str(request))
		dict = {}
		dict['request']= rosidl_runtime_py.message_to_ordereddict(request)
		dict['service']='add_two_ints_1'
		dict['time']=float(self.get_clock().now().to_msg().sec)
		self.ws_lock.acquire()
		while dict['time'] in self.dict_msgs:
			dict['time']+=0.01
		self.ws.send(json.dumps(dict))
		self.dict_msgs[dict['time']] = request
		message=self.ws.recv()
		self.ws_lock.release()
		self.get_logger().info("event propagated to oracle")
		try:
			return self.on_message_service_request(message)
		except:
			response.error = True
			return response

	def __init__(self,monitor_name,log,actions):
		self.monitor_publishers={}
		self.config_publishers={}
		self.config_subscribers={}
		self.config_client_services={}
		self.config_server_services={}
		self.services_info={}
		self.dict_msgs={}
		self.ws_lock=Lock()
		self.name=monitor_name
		self.actions=actions
		self.logfn=log
		self.topics_info={}
		super().__init__(self.name)
		# creating the verdict and error publishers for the monitor
		self.monitor_publishers['error']=self.create_publisher(topic=self.name+'/monitor_error',msg_type=MonitorError,qos_profile=1000)

		self.monitor_publishers['verdict']=self.create_publisher(topic=self.name+'/monitor_verdict',msg_type=String,qos_profile=1000)

		# done creating monitor publishers

		self.publish_topics=False
		self.config_client_services['add_two_ints']=ServiceNode(AddTwoInts,'add_two_ints')
		self.config_client_services['add_two_ints_1']=ServiceNode(AddTwoInts,'add_two_ints_1')
		self.topics_info['chatter']={'package': 'std_msgs.msg', 'type': 'String'}
		self.services_info['add_two_ints']={'package': 'example_interfaces.srv', 'type': 'AddTwoInts'}
		self.services_info['add_two_ints_1']={'package': 'example_interfaces.srv', 'type': 'AddTwoInts'}
		self.config_subscribers['chatter']=self.create_subscription(topic='chatter',msg_type=String,callback=self.callbackchatter,qos_profile=1000)

		self.config_server_services['add_two_ints']=self.create_service(AddTwoInts, 'add_two_ints_mon', self.callbackadd_two_ints_mon, callback_group=MutuallyExclusiveCallbackGroup())

		self.config_server_services['add_two_ints_1']=self.create_service(AddTwoInts, 'add_two_ints_1_mon', self.callbackadd_two_ints_1_mon, callback_group=MutuallyExclusiveCallbackGroup())

		self.get_logger().info('Monitor' + self.name + ' started and ready' )
		self.get_logger().info('Logging at' + self.logfn )
		websocket.enableTrace(True)
		self.ws = websocket.WebSocket()
		self.ws.connect('ws://127.0.0.1:8080')
		self.get_logger().info('Websocket is open')


	def on_message_topic(self,message):
		json_dict = json.loads(message)
		verdict = str(json_dict['verdict'])
		if verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':
			if verdict == 'true' and not self.publish_topics:
				self.get_logger().info('The monitor concluded the satisfaction of the property under analysis and can be safely removed.')
				self.ws.close()
				exit(0)
			else:
				self.logging(json_dict)
				topic = json_dict['topic']
				self.get_logger().info('The event '+message+' is consistent and republished')
				if topic in self.config_publishers:
					self.config_publishers[topic].publish(self.dict_msgs[json_dict['time']])
				del self.dict_msgs[json_dict['time']]
		else:
			self.logging(json_dict)
			self.get_logger().info('The event' + message + ' is inconsistent' )
			error = MonitorError()
			error.m_topic = json_dict['topic']
			error.m_time = json_dict['time']
			error.m_property = json_dict['spec']
			error.m_content = str(self.dict_msgs[json_dict['time']])
			self.monitor_publishers['error'].publish(error)
			if verdict == 'false' and not self.publish_topics:
				self.get_logger().info('The monitor concluded the violation of the property under analysis and can be safely removed.')
				self.ws.close()
				exit(0)
			if self.actions[json_dict['topic']][0] != 'filter':
				topic = json_dict['topic']
				if topic in self.config_publishers:
					self.config_publishers[topic].publish(self.dict_msgs[json_dict['time']])
				del self.dict_msgs[json_dict['time']]
			error=True
		verdict_msg = String()
		verdict_msg.data = verdict
		self.monitor_publishers['verdict'].publish(verdict_msg)
	def on_message_service_request(self,message):
		json_dict = json.loads(message)
		verdict = str(json_dict['verdict'])
		service = json_dict['service'] = json_dict['service'].replace('_mon', '')
		verdict_msg = String()
		verdict_msg.data = verdict
		self.monitor_publishers['verdict'].publish(verdict_msg)
		if verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':
			del json_dict['verdict']
			self.logging(json_dict)
			self.get_logger().info('The request '+message+' is consistent, the service is called')
			if service in self.config_client_services:
				res = self.config_client_services[service].call_service(self.dict_msgs[json_dict['time']])
				json_dict['response'] = rosidl_runtime_py.message_to_ordereddict(res)
				self.dict_msgs[json_dict['time']] = res
			del json_dict['request']
			self.ws_lock.acquire()
			self.ws.send(json.dumps(json_dict))
			msg=self.ws.recv()
			self.ws_lock.release()
			return self.on_message_service_response(msg)
		else:
			self.logging(json_dict)
			self.get_logger().info('The event request' + message + ' is inconsistent' )
			error = MonitorError()
			error.m_service = json_dict['service'].replace('_mon', '')
			error.m_time = json_dict['time']
			error.m_property = json_dict['spec']
			error.m_content = str(self.dict_msgs[json_dict['time']])
			self.monitor_publishers['error'].publish(error)
			error=True
		if self.actions[json_dict['service']][0] != 'filter':
			service = json_dict['service'] = json_dict['service'].replace('_mon', '')
			if service in self.config_client_services:
				res = self.config_client_services[service].call_service(self.dict_msgs[json_dict['time']])
				self.dict_msgs[json_dict['time']] = res
				json_dict['response'] = rosidl_runtime_py.message_to_ordereddict(res)
			if 'verdict' in json_dict: del json_dict['verdict']
			del json_dict['request']
			self.ws_lock.acquire()
			self.ws.send(json.dumps(json_dict))
			msg=self.ws.recv()
			self.ws_lock.release()
			return self.on_message_service_response(msg)
		else:
			raise Exception('The request violates the monitor specification, so it has been filtered out.')

	def on_message_service_response(self,message):
		json_dict = json.loads(message)
		verdict = str(json_dict['verdict'])
		service = json_dict['service'] = json_dict['service'].replace('_mon', '')
		verdict_msg = String()
		verdict_msg.data = verdict
		self.monitor_publishers['verdict'].publish(verdict_msg)
		if verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':
			self.logging(json_dict)
			self.get_logger().info('The response '+message+' is consistent, the result is returned')
			return self.dict_msgs[json_dict['time']]
		else:
			self.logging(json_dict)
			self.get_logger().info('The event response' + message + ' is inconsistent' )
			error = MonitorError()
			error.m_service = json_dict['service'].replace('_mon', '')
			error.m_time = json_dict['time']
			error.m_property = json_dict['spec']
			error.m_content = str(self.dict_msgs[json_dict['time']])
			self.monitor_publishers['error'].publish(error)
			error=True
		if self.actions[json_dict['service']][0] != 'filter':
			service = json_dict['service'] = json_dict['service'].replace('_mon', '')
			return self.dict_msgs[json_dict['time']]
		else:
			raise Exception('The request violates the monitor specification, so it has been filtered out.')


	def logging(self,json_dict):
		try:
			with open(self.logfn,'a+') as log_file:
				log_file.write(json.dumps(json_dict)+'\n')
			self.get_logger().info('Event logged')
		except:
			self.get_logger().info('Unable to log the event')

def main(args=None):
	rclpy.init(args=args)
	log = './log.txt'
	actions = {}
	actions['chatter']=('log',0)
	actions['add_two_ints']=('filter',0)
	actions['add_two_ints_1']=('log',0)
	monitor = ROSMonitor_monitor_0('monitor_0',log,actions)
	rclpy.spin(monitor)
	monitor.ws.close()
	monitor.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
class ServiceNode(Node):
	def __init__(self, service_type, service_name):
		super().__init__('service_node_' + service_name.replace('/', '_'))
		self.cli = self.create_client(service_type, service_name)
		while not self.cli.wait_for_service(timeout_sec=1.0):
			self.get_logger().info('service not available, waiting again...')
	def call_service(self, request):
		self.future = self.cli.call_async(request)
		rclpy.spin_until_future_complete(self, self.future)
		return self.future.result()
