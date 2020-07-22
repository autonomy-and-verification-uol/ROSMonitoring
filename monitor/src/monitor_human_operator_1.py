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
            
from nist_gear.msg import Snapshot

def callback_monitor_snapshot_1(data):
    global ws, ws_lock
    rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = '/monitor/snapshot_1'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    while dict['time'] in dict_msgs:
        dict['time'] += 0.01
    ws.send(json.dumps(dict))
    dict_msgs[dict['time']] = data
    ws_lock.release()
    rospy.loginfo('event propagated to oracle')

pub_dict = {
}
        
msg_dict = {
    '/monitor/snapshot_1' : "nist_gear/Snapshot"
}
        
def monitor():
    global pub_error
    with open(log, 'w') as log_file:
        log_file.write('')
    rospy.init_node('monitor', anonymous=True)
    pub_error = rospy.Publisher(name = 'monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)
    rospy.Subscriber('/monitor/snapshot_1', Snapshot, callback_monitor_snapshot_1)
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
            error.content = str(dict_msgs[json_dict['time']])
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
    log = '/home/angelo/ariac_ws/src/ARIAC/ariac_example/log.txt'
    
    actions = {
            '/monitor/snapshot_1' : ('log', False)
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
        