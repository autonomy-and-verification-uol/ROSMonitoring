#!/usr/bin/env python
import rospy
import sys
import json
import yaml
import websocket
from threading import *

ws_lock = Lock()

from rospy_message_converter import message_converter
from monitor.msg import *
        
from std_msgs.msg import String
from std_msgs.msg import String
from std_msgs.msg import String
from std_msgs.msg import String
from std_msgs.msg import String
from std_msgs.msg import String
from std_msgs.msg import String
from std_msgs.msg import String
from std_msgs.msg import String
from std_msgs.msg import String
from std_msgs.msg import String
from std_msgs.msg import String
from std_msgs.msg import String

pubtopic_44 = rospy.Publisher(name = 'topic_44', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_44(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_44'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_32 = rospy.Publisher(name = 'topic_32', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_32(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_32'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_40 = rospy.Publisher(name = 'topic_40', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_40(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_40'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_28 = rospy.Publisher(name = 'topic_28', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_28(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_28'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_36 = rospy.Publisher(name = 'topic_36', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_36(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_36'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_12 = rospy.Publisher(name = 'topic_12', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_12(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_12'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_24 = rospy.Publisher(name = 'topic_24', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_24(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_24'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_48 = rospy.Publisher(name = 'topic_48', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_48(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_48'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_16 = rospy.Publisher(name = 'topic_16', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_16(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_16'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_20 = rospy.Publisher(name = 'topic_20', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_20(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_20'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_4 = rospy.Publisher(name = 'topic_4', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_4(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_4'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_0 = rospy.Publisher(name = 'topic_0', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_0(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_0'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_8 = rospy.Publisher(name = 'topic_8', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_8(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_8'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')

pub_dict = {
    'topic_44' : pubtopic_44, 
    'topic_32' : pubtopic_32, 
    'topic_40' : pubtopic_40, 
    'topic_28' : pubtopic_28, 
    'topic_36' : pubtopic_36, 
    'topic_12' : pubtopic_12, 
    'topic_24' : pubtopic_24, 
    'topic_48' : pubtopic_48, 
    'topic_16' : pubtopic_16, 
    'topic_20' : pubtopic_20, 
    'topic_4' : pubtopic_4, 
    'topic_0' : pubtopic_0, 
    'topic_8' : pubtopic_8
}
        
msg_dict = {
    'topic_44' : "std_msgs/String", 
    'topic_32' : "std_msgs/String", 
    'topic_40' : "std_msgs/String", 
    'topic_28' : "std_msgs/String", 
    'topic_36' : "std_msgs/String", 
    'topic_12' : "std_msgs/String", 
    'topic_24' : "std_msgs/String", 
    'topic_48' : "std_msgs/String", 
    'topic_16' : "std_msgs/String", 
    'topic_20' : "std_msgs/String", 
    'topic_4' : "std_msgs/String", 
    'topic_0' : "std_msgs/String", 
    'topic_8' : "std_msgs/String"
}
        
def monitor():
    global pub_error
    with open(log, 'w') as log_file:
        log_file.write('')
    rospy.init_node('monitor', anonymous=True)
    pub_error = rospy.Publisher(name = 'monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)
    rospy.Subscriber('topic_44_mon', String, callbacktopic_44)
    rospy.Subscriber('topic_32_mon', String, callbacktopic_32)
    rospy.Subscriber('topic_40_mon', String, callbacktopic_40)
    rospy.Subscriber('topic_28_mon', String, callbacktopic_28)
    rospy.Subscriber('topic_36_mon', String, callbacktopic_36)
    rospy.Subscriber('topic_12_mon', String, callbacktopic_12)
    rospy.Subscriber('topic_24_mon', String, callbacktopic_24)
    rospy.Subscriber('topic_48_mon', String, callbacktopic_48)
    rospy.Subscriber('topic_16_mon', String, callbacktopic_16)
    rospy.Subscriber('topic_20_mon', String, callbacktopic_20)
    rospy.Subscriber('topic_4_mon', String, callbacktopic_4)
    rospy.Subscriber('topic_0_mon', String, callbacktopic_0)
    rospy.Subscriber('topic_8_mon', String, callbacktopic_8)
    #rospy.loginfo('monitor started and ready')
        
def on_message(ws, message):
    global error, log, actions
    json_dict = json.loads(message)
    if 'error' in json_dict:
        logging(json_dict)
        print('The event ' + message + ' is inconsistent..')
        if actions[json_dict['topic']][1]:
            json_dict_copy = json_dict.copy()
            error = MonitorError()
            error.topic = json_dict_copy['topic']
            error.time = json_dict_copy['time']
            error.property = json_dict_copy['spec']
            del json_dict_copy['topic']
            del json_dict_copy['time']
            del json_dict_copy['spec']
            del json_dict_copy['error']
            error.content = json.dumps(json_dict_copy)
            pub_error.publish(error)
        #if actions[json_dict['topic']][0] == 'filter':
            #rospy.loginfo('Not republished..')
        #else:
            #rospy.loginfo('Let it go..')
        if actions[json_dict['topic']][0] != 'filter':
            topic = json_dict['topic']
            del json_dict['topic']
            del json_dict['time']
            del json_dict['error']
            del json_dict['spec']
            ROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)
            if topic in pub_dict:
                pub_dict[topic].publish(ROS_message)
    	error = True
    else:
        logging(json_dict)
        topic = json_dict['topic']
        del json_dict['topic']
        del json_dict['time']
    	#rospy.loginfo('The event ' + message + ' is consistent and republished')
        ROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)
    	if topic in pub_dict:
            pub_dict[topic].publish(ROS_message)

def on_error(ws, error):
    rospy.loginfo(error)

def on_close(ws):
	rospy.loginfo('### websocket closed ###')

def on_open(ws):
	rospy.loginfo('### websocket is open ###')
            
def logging(json_dict):
    try:
        with open(log, 'a+') as log_file:
            log_file.write(json.dumps(json_dict) + '\n')
        #rospy.loginfo('event logged')
    except:
        rospy.loginfo('Unable to log the event.')

def main(argv):
    global log, actions, ws
    log = './log_monitor_0.txt'
    
    actions = {
            'topic_44' : ('log', False), 
            'topic_32' : ('log', False), 
            'topic_40' : ('log', False), 
            'topic_28' : ('log', False), 
            'topic_36' : ('log', False), 
            'topic_12' : ('log', False), 
            'topic_24' : ('log', False), 
            'topic_48' : ('log', False), 
            'topic_16' : ('log', False), 
            'topic_20' : ('log', False), 
            'topic_4' : ('log', False), 
            'topic_0' : ('log', False), 
            'topic_8' : ('log', False)
    }
    monitor()
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp(
        'ws://127.0.0.1:8080',
        on_message = on_message,
        on_error = on_error,
        on_close = on_close,
        on_open = on_open)
    ws.run_forever()
if __name__ == '__main__':
    main(sys.argv)
        