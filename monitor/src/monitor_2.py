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

pubtopic_30 = rospy.Publisher(name = 'topic_30', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_30(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_30'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_46 = rospy.Publisher(name = 'topic_46', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_46(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_46'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_18 = rospy.Publisher(name = 'topic_18', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_18(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_18'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_34 = rospy.Publisher(name = 'topic_34', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_34(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_34'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_42 = rospy.Publisher(name = 'topic_42', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_42(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_42'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_26 = rospy.Publisher(name = 'topic_26', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_26(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_26'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_38 = rospy.Publisher(name = 'topic_38', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_38(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_38'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_10 = rospy.Publisher(name = 'topic_10', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_10(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_10'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_22 = rospy.Publisher(name = 'topic_22', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_22(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_22'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_14 = rospy.Publisher(name = 'topic_14', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_14(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_14'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_6 = rospy.Publisher(name = 'topic_6', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_6(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_6'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')
pubtopic_2 = rospy.Publisher(name = 'topic_2', data_class = String, latch = True, queue_size = 1000)
def callbacktopic_2(data):
    global ws, ws_lock
    #rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'topic_2'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    #rospy.loginfo('event propagated to oracle')

pub_dict = {
    'topic_30' : pubtopic_30, 
    'topic_46' : pubtopic_46, 
    'topic_18' : pubtopic_18, 
    'topic_34' : pubtopic_34, 
    'topic_42' : pubtopic_42, 
    'topic_26' : pubtopic_26, 
    'topic_38' : pubtopic_38, 
    'topic_10' : pubtopic_10, 
    'topic_22' : pubtopic_22, 
    'topic_14' : pubtopic_14, 
    'topic_6' : pubtopic_6, 
    'topic_2' : pubtopic_2
}
        
msg_dict = {
    'topic_30' : "std_msgs/String", 
    'topic_46' : "std_msgs/String", 
    'topic_18' : "std_msgs/String", 
    'topic_34' : "std_msgs/String", 
    'topic_42' : "std_msgs/String", 
    'topic_26' : "std_msgs/String", 
    'topic_38' : "std_msgs/String", 
    'topic_10' : "std_msgs/String", 
    'topic_22' : "std_msgs/String", 
    'topic_14' : "std_msgs/String", 
    'topic_6' : "std_msgs/String", 
    'topic_2' : "std_msgs/String"
}
        
def monitor():
    global pub_error
    with open(log, 'w') as log_file:
        log_file.write('')
    rospy.init_node('monitor', anonymous=True)
    pub_error = rospy.Publisher(name = 'monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)
    rospy.Subscriber('topic_30_mon', String, callbacktopic_30)
    rospy.Subscriber('topic_46_mon', String, callbacktopic_46)
    rospy.Subscriber('topic_18_mon', String, callbacktopic_18)
    rospy.Subscriber('topic_34_mon', String, callbacktopic_34)
    rospy.Subscriber('topic_42_mon', String, callbacktopic_42)
    rospy.Subscriber('topic_26_mon', String, callbacktopic_26)
    rospy.Subscriber('topic_38_mon', String, callbacktopic_38)
    rospy.Subscriber('topic_10_mon', String, callbacktopic_10)
    rospy.Subscriber('topic_22_mon', String, callbacktopic_22)
    rospy.Subscriber('topic_14_mon', String, callbacktopic_14)
    rospy.Subscriber('topic_6_mon', String, callbacktopic_6)
    rospy.Subscriber('topic_2_mon', String, callbacktopic_2)
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
    log = './log_monitor_2.txt'
    
    actions = {
            'topic_30' : ('log', False), 
            'topic_46' : ('log', False), 
            'topic_18' : ('log', False), 
            'topic_34' : ('log', False), 
            'topic_42' : ('log', False), 
            'topic_26' : ('log', False), 
            'topic_38' : ('log', False), 
            'topic_10' : ('log', False), 
            'topic_22' : ('log', False), 
            'topic_14' : ('log', False), 
            'topic_6' : ('log', False), 
            'topic_2' : ('log', False)
    }
    monitor()
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp(
        'ws://127.0.0.1:8082',
        on_message = on_message,
        on_error = on_error,
        on_close = on_close,
        on_open = on_open)
    ws.run_forever()
if __name__ == '__main__':
    main(sys.argv)
        