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

pubtopic_45 = rospy.Publisher(name = 'topic_45', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_45(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_45'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_33 = rospy.Publisher(name = 'topic_33', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_33(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_33'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_41 = rospy.Publisher(name = 'topic_41', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_41(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_41'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_37 = rospy.Publisher(name = 'topic_37', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_37(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_37'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_29 = rospy.Publisher(name = 'topic_29', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_29(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_29'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_13 = rospy.Publisher(name = 'topic_13', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_13(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_13'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_25 = rospy.Publisher(name = 'topic_25', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_25(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_25'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_17 = rospy.Publisher(name = 'topic_17', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_17(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_17'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_49 = rospy.Publisher(name = 'topic_49', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_49(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_49'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_21 = rospy.Publisher(name = 'topic_21', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_21(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_21'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_5 = rospy.Publisher(name = 'topic_5', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_5(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_5'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_1 = rospy.Publisher(name = 'topic_1', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_1(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_1'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_9 = rospy.Publisher(name = 'topic_9', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_9(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_9'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')

pub_dict = {
    'topic_45' : pubtopic_45, 
    'topic_33' : pubtopic_33, 
    'topic_41' : pubtopic_41, 
    'topic_37' : pubtopic_37, 
    'topic_29' : pubtopic_29, 
    'topic_13' : pubtopic_13, 
    'topic_25' : pubtopic_25, 
    'topic_17' : pubtopic_17, 
    'topic_49' : pubtopic_49, 
    'topic_21' : pubtopic_21, 
    'topic_5' : pubtopic_5, 
    'topic_1' : pubtopic_1, 
    'topic_9' : pubtopic_9
}
        
msg_dict = {
    'topic_45' : "std_msgs/String", 
    'topic_33' : "std_msgs/String", 
    'topic_41' : "std_msgs/String", 
    'topic_37' : "std_msgs/String", 
    'topic_29' : "std_msgs/String", 
    'topic_13' : "std_msgs/String", 
    'topic_25' : "std_msgs/String", 
    'topic_17' : "std_msgs/String", 
    'topic_49' : "std_msgs/String", 
    'topic_21' : "std_msgs/String", 
    'topic_5' : "std_msgs/String", 
    'topic_1' : "std_msgs/String", 
    'topic_9' : "std_msgs/String"
}
        
def monitor():
    global pub_error
    with open(log, 'w') as log_file:
        log_file.write('')
    rospy.init_node('monitor', anonymous=True)
    pub_error = rospy.Publisher(name = 'monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)
    rospy.Subscriber('topic_45_mon', String, callbacktopic_45)
    rospy.Subscriber('topic_33_mon', String, callbacktopic_33)
    rospy.Subscriber('topic_41_mon', String, callbacktopic_41)
    rospy.Subscriber('topic_37_mon', String, callbacktopic_37)
    rospy.Subscriber('topic_29_mon', String, callbacktopic_29)
    rospy.Subscriber('topic_13_mon', String, callbacktopic_13)
    rospy.Subscriber('topic_25_mon', String, callbacktopic_25)
    rospy.Subscriber('topic_17_mon', String, callbacktopic_17)
    rospy.Subscriber('topic_49_mon', String, callbacktopic_49)
    rospy.Subscriber('topic_21_mon', String, callbacktopic_21)
    rospy.Subscriber('topic_5_mon', String, callbacktopic_5)
    rospy.Subscriber('topic_1_mon', String, callbacktopic_1)
    rospy.Subscriber('topic_9_mon', String, callbacktopic_9)
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
    log = './log_monitor_1.txt'
    
    actions = {
            'topic_45' : ('log', False), 
            'topic_33' : ('log', False), 
            'topic_41' : ('log', False), 
            'topic_37' : ('log', False), 
            'topic_29' : ('log', False), 
            'topic_13' : ('log', False), 
            'topic_25' : ('log', False), 
            'topic_17' : ('log', False), 
            'topic_49' : ('log', False), 
            'topic_21' : ('log', False), 
            'topic_5' : ('log', False), 
            'topic_1' : ('log', False), 
            'topic_9' : ('log', False)
    }
    monitor()
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp(
        'ws://127.0.0.1:8081',
        on_message = on_message,
        on_error = on_error,
        on_close = on_close,
        on_open = on_open)
    ws.run_forever()
if __name__ == '__main__':
    main(sys.argv)
        