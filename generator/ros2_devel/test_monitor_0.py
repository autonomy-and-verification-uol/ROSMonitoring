#!/usr/bin/env python

# begin imports
import json
import yaml
import websocket
import sys
import rclpy
from rclpy.node import Node
from threading import *
from rosidl_runtime_py import message_converter
from rosmonitoring_interfaces.msg import MonitorError
from std_msgs.msg import String
# done import

class ROSMonitor_monitor_0(Node):

	def __init__(self,monitor_name,log,actions):
		self.monitor_publishers={}
		self.config_publishers={}
		self.config_subscribers={}
		self.dict_msgs={}
		self.ws_lock=Lock()
		self.name=monitor_name
		self.logfn=log
		super().__init__(self.name)
		# creating the verdict and error publishers for the monitor
		self.monitor_publishers[error]=self.create_publishers(topic=self.name+'/monitor_error',msg_type=MonitorError,qos_profile=1000)

		self.monitor_publishers[verdict]=self.create_publishers(topic=self.name+'/monitor_verdict',msg_type=String,qos_profile=1000)

		# done creating monitor publishers

		self.config_publishers[chatter]=self.create_publishers(topic=chatter_mon,msg_type=String,qos_profile=1000)



	def callbackchatter(data):
		self.get_logger().info("monitor has observed "+ str(data))
		dict= message_converter.message_to_ordereddict(data)
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
		self.on_message(message)
