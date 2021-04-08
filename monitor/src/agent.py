#!/usr/bin/env python
import rospy
import sys
import json
import yaml
import websocket
from threading import *
from rospy_message_converter import message_converter
from monitor.msg import *
from std_msgs.msg import String

ws_lock = Lock()
dict_msgs = {}
            
from gazebo_radiation_plugins.msg import Inspection
from gazebo_radiation_plugins.msg import Command
from std_msgs.msg import String
from std_msgs.msg import Int16

def callbackcurrentLoc(data):
    global ws, ws_lock
    rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'currentLoc'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    while dict['time'] in dict_msgs:
        dict['time'] += 0.01
    ws.send(json.dumps(dict))
    dict_msgs[dict['time']] = data
    ws_lock.release()
    rospy.loginfo('event propagated to oracle')
def callbackradiationStatus(data):
    global ws, ws_lock
    rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'radiationStatus'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    while dict['time'] in dict_msgs:
        dict['time'] += 0.01
    ws.send(json.dumps(dict))
    dict_msgs[dict['time']] = data
    ws_lock.release()
    rospy.loginfo('event propagated to oracle')
def callbackcommand(data):
    global ws, ws_lock
    rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'command'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()
    while dict['time'] in dict_msgs:
        dict['time'] += 0.01
    ws.send(json.dumps(dict))
    dict_msgs[dict['time']] = data
    ws_lock.release()
    rospy.loginfo('event propagated to oracle')
def callbackinspected(data):
    global ws, ws_lock
    rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'inspected'
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
    'currentLoc' : "std_msgs/Int16", 
    'radiationStatus' : "std_msgs/String", 
    'command' : "gazebo_radiation_plugins/Command", 
    'inspected' : "gazebo_radiation_plugins/Inspection"
}
        
def monitor():
    global pub_error, pub_verdict
    with open(log, 'w') as log_file:
        log_file.write('')
    rospy.init_node('agent', anonymous=True)
    pub_error = rospy.Publisher(name = 'agent/monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)
    pub_verdict = rospy.Publisher(name = 'agent/monitor_verdict', data_class = String, latch = True, queue_size = 1000)
    rospy.Subscriber('currentLoc', Int16, callbackcurrentLoc)
    rospy.Subscriber('radiationStatus', String, callbackradiationStatus)
    rospy.Subscriber('command', Command, callbackcommand)
    rospy.Subscriber('inspected', Inspection, callbackinspected)
    rospy.loginfo('monitor started and ready')
        
def on_message(ws, message):
    global error, log, actions
    json_dict = json.loads(message)
    if json_dict['verdict'] == 'true' or json_dict['verdict'] == 'currently_true' or json_dict['verdict'] == 'unknown':
        if json_dict['verdict'] == 'true' and not pub_dict:
            rospy.loginfo('The monitor concluded the satisfaction of the property under analysis, and can be safely removed.')
            ws.close()
            exit(0)
        else:
            logging(json_dict)
            topic = json_dict['topic']
            rospy.loginfo('The event ' + message + ' is consistent and republished')
            if topic in pub_dict:
                pub_dict[topic].publish(dict_msgs[json_dict['time']])
            del dict_msgs[json_dict['time']]
    else:
        logging(json_dict)
        if (json_dict['verdict'] == 'false' and actions[json_dict['topic']][1] >= 1) or (json_dict['verdict'] == 'currently_false' and actions[json_dict['topic']][1] == 1):
            rospy.loginfo('The event ' + message + ' is inconsistent..')
            error = MonitorError()
            error.topic = json_dict['topic']
            error.time = json_dict['time']
            error.property = json_dict['spec']
            error.content = str(dict_msgs[json_dict['time']])
            pub_error.publish(error)
            if json_dict['verdict'] == 'false' and not pub_dict:
                rospy.loginfo('The monitor concluded the violation of the property under analysis, and can be safely removed.')
                ws.close()
                exit(0)
        if actions[json_dict['topic']][0] != 'filter':
            if json_dict['verdict'] == 'currently_false':
                rospy.loginfo('The event ' + message + ' is consistent ')
            topic = json_dict['topic']
            if topic in pub_dict:
                pub_dict[topic].publish(dict_msgs[json_dict['time']])
            del dict_msgs[json_dict['time']]
    	error = True
    pub_verdict.publish(json_dict['verdict'])

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
    log = '/media/angelo/WorkData/git/radiation_ws/src/monitor/log_agent.txt'
    
    actions = {
            'currentLoc' : ('log', 1), 
            'radiationStatus' : ('log', 1), 
            'command' : ('log', 1), 
            'inspected' : ('log', 1)
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
        