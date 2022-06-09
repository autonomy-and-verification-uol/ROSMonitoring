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
# creating the verdict and error publishers for the monitor
self.monitor_publishers={}
self.monitor_publishers[error]=self.create_publishers(topic='monitor_0/monitor_error',msg_type=MonitorError,qos_profile=1000)
self.monitor_publishers[verdict]=self.create_publishers(topic='monitor_0/monitor_verdict',msg_type=String,qos_profile=1000)
# done creating monitor publishers

