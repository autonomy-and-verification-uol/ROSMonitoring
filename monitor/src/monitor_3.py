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

pubtopic_31 = rospy.Publisher(name = 'topic_31', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_31(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_31'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_19 = rospy.Publisher(name = 'topic_19', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_19(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_19'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_47 = rospy.Publisher(name = 'topic_47', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_47(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_47'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_35 = rospy.Publisher(name = 'topic_35', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_35(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_35'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_43 = rospy.Publisher(name = 'topic_43', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_43(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_43'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_39 = rospy.Publisher(name = 'topic_39', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_39(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_39'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_27 = rospy.Publisher(name = 'topic_27', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_27(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_27'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_11 = rospy.Publisher(name = 'topic_11', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_11(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_11'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_23 = rospy.Publisher(name = 'topic_23', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_23(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_23'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_15 = rospy.Publisher(name = 'topic_15', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_15(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_15'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_7 = rospy.Publisher(name = 'topic_7', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_7(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_7'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_3 = rospy.Publisher(name = 'topic_3', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_3(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_3'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')

pub_dict = {
    'topic_31' : pubtopic_31, 
    'topic_19' : pubtopic_19, 
    'topic_47' : pubtopic_47, 
    'topic_35' : pubtopic_35, 
    'topic_43' : pubtopic_43, 
    'topic_39' : pubtopic_39, 
    'topic_27' : pubtopic_27, 
    'topic_11' : pubtopic_11, 
    'topic_23' : pubtopic_23, 
    'topic_15' : pubtopic_15, 
    'topic_7' : pubtopic_7, 
    'topic_3' : pubtopic_3
}
        
msg_dict = {
    'topic_31' : "std_msgs/String", 
    'topic_19' : "std_msgs/String", 
    'topic_47' : "std_msgs/String", 
    'topic_35' : "std_msgs/String", 
    'topic_43' : "std_msgs/String", 
    'topic_39' : "std_msgs/String", 
    'topic_27' : "std_msgs/String", 
    'topic_11' : "std_msgs/String", 
    'topic_23' : "std_msgs/String", 
    'topic_15' : "std_msgs/String", 
    'topic_7' : "std_msgs/String", 
    'topic_3' : "std_msgs/String"
}
        
def monitor():
    global pub_error
    with open(log, 'w') as log_file:
        log_file.write('')
    rospy.init_node('monitor', anonymous=True)
    pub_error = rospy.Publisher(name = 'monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)
    rospy.Subscriber('topic_31_mon', String, callbacktopic_31)
    rospy.Subscriber('topic_19_mon', String, callbacktopic_19)
    rospy.Subscriber('topic_47_mon', String, callbacktopic_47)
    rospy.Subscriber('topic_35_mon', String, callbacktopic_35)
    rospy.Subscriber('topic_43_mon', String, callbacktopic_43)
    rospy.Subscriber('topic_39_mon', String, callbacktopic_39)
    rospy.Subscriber('topic_27_mon', String, callbacktopic_27)
    rospy.Subscriber('topic_11_mon', String, callbacktopic_11)
    rospy.Subscriber('topic_23_mon', String, callbacktopic_23)
    rospy.Subscriber('topic_15_mon', String, callbacktopic_15)
    rospy.Subscriber('topic_7_mon', String, callbacktopic_7)
    rospy.Subscriber('topic_3_mon', String, callbacktopic_3)
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
    log = './log_monitor_3.txt'
    
    actions = {
            'topic_31' : ('log', False), 
            'topic_19' : ('log', False), 
            'topic_47' : ('log', False), 
            'topic_35' : ('log', False), 
            'topic_43' : ('log', False), 
            'topic_39' : ('log', False), 
            'topic_27' : ('log', False), 
            'topic_11' : ('log', False), 
            'topic_23' : ('log', False), 
            'topic_15' : ('log', False), 
            'topic_7' : ('log', False), 
            'topic_3' : ('log', False)
    }
    monitor()
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp(
        'ws://127.0.0.1:8083',
        on_message = on_message,
        on_error = on_error,
        on_close = on_close,
        on_open = on_open)
    ws.run_forever()
if __name__ == '__main__':
    main(sys.argv)
        