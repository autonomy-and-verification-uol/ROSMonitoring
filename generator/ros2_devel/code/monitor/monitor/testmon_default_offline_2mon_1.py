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
# done import

class ROSMonitor_testmon_default_offline_2mon_1(Node):


	def callbackchatter(self,data):
		self.get_logger().info("monitor has observed "+ str(data))
		dict= rosidl_runtime_py.message_to_ordereddict(data)
		dict['topic']='chatter'
		dict['time']=float(self.get_clock().now().to_msg().sec)
		self.ws_lock.acquire()
		self.logging(dict)
		self.ws_lock.release()
		self.get_logger().info("event successfully logged")

	def __init__(self,monitor_name,log,actions):
		self.monitor_publishers={}
		self.config_publishers={}
		self.config_subscribers={}
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
		self.topics_info['chatter']={'package': 'std_msgs.msg', 'type': 'String'}
		self.config_subscribers['chatter']=self.create_subscription(topic='chatter',msg_type=String,callback=self.callbackchatter,qos_profile=1000)

		self.get_logger().info('Monitor' + self.name + ' started and ready' )
		self.get_logger().info('Logging at' + self.logfn )



	def logging(self,json_dict):
		try:
			with open(self.logfn,'a+') as log_file:
				log_file.write(json.dumps(json_dict)+'\n')
			self.get_logger().info('Event logged')
		except:
			self.get_logger().info('Unable to log the event')

def main(args=None):
	rclpy.init(args=args)
	log = '/home/parallels/code/ros2_rosmonintoring/ros2_code/logs/testmon_default_offline_2mon_1.txt'
	actions = {}
	actions['chatter']=('log',0)
	monitor = ROSMonitor_testmon_default_offline_2mon_1('testmon_default_offline_2mon_1',log,actions)
	rclpy.spin(monitor)
	monitor.ws.close()
	monitor.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
