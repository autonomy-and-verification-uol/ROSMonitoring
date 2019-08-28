#!/usr/bin/env python
import rospy
import sys
import json
import yaml
import websocket
from threading import *
from rospy_message_converter import message_converter
from monitor.msg import *

ws_lock = Lock()
dict_msgs = {}
            
from std_msgs.msg import String
from std_msgs.msg import Int32
from rosmon.msg import Person

def callbackchatter(data):
    global ws, ws_lock
    rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'chatter'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    while dict['time'] in dict_msgs:
        dict['time'] += 0.01
    ws.send(json.dumps(dict))
    dict_msgs[dict['time']] = data
    ws_lock.release()
    rospy.loginfo('event propagated to oracle')
pubcount = rospy.Publisher(name = 'count_mon', data_class = Int32, latch = True, queue_size = 1000)
def callbackcount(data):
    global ws, ws_lock
    rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'count'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    while dict['time'] in dict_msgs:
        dict['time'] += 0.01
    ws.send(json.dumps(dict))
    dict_msgs[dict['time']] = data
    ws_lock.release()
    rospy.loginfo('event propagated to oracle')
pubperson = rospy.Publisher(name = 'person', data_class = Person, latch = True, queue_size = 1000)
def callbackperson(data):
    global ws, ws_lock
    rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'person'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    while dict['time'] in dict_msgs:
        dict['time'] += 0.01
    ws.send(json.dumps(dict))
    dict_msgs[dict['time']] = data
    ws_lock.release()
    rospy.loginfo('event propagated to oracle')

pub_dict = {
    'count' : pubcount, 
    'person' : pubperson
}
        
msg_dict = {
    'chatter' : "std_msgs/String", 
    'count' : "std_msgs/Int32", 
    'person' : "rosmon/Person"
}
        
def monitor():
    global pub_error
    with open(log, 'w') as log_file:
        log_file.write('')
    rospy.init_node('monitor', anonymous=True)
    pub_error = rospy.Publisher(name = 'monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)
    rospy.Subscriber('chatter', String, callbackchatter)
    rospy.Subscriber('count', Int32, callbackcount)
    rospy.Subscriber('person_mon', Person, callbackperson)
    rospy.loginfo('monitor started and ready')
        
def on_message(ws, message):
    global error, log, actions
    json_dict = json.loads(message)
    if 'error' in json_dict:
        logging(json_dict)
        rospy.loginfo('The event ' + message + ' is inconsistent..')
        if actions[json_dict['topic']][1]:
            error = MonitorError()
            error.topic = json_dict['topic']
            error.time = json_dict['time']
            error.property = json_dict['spec']
            error.content = dict_msgs[json_dict['time']]
            pub_error.publish(error)
        if actions[json_dict['topic']][0] != 'filter':
            topic = json_dict['topic']
            if topic in pub_dict:
                pub_dict[topic].publish(dict_msgs[json_dict['time']])
            del dict_msgs[json_dict['time']]
    	error = True
    else:
        logging(json_dict)
        topic = json_dict['topic']
    	rospy.loginfo('The event ' + message + ' is consistent and republished')
        if topic in pub_dict:
            pub_dict[topic].publish(dict_msgs[json_dict['time']])
        del dict_msgs[json_dict['time']]
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
        rospy.loginfo('event logged')
    except:
        rospy.loginfo('Unable to log the event.')

def main(argv):
    global log, actions, ws
    log = './log_0.txt'
    
    actions = {
            'chatter' : ('log', True), 
            'count' : ('filter', False), 
            'person' : ('filter', True)
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
        