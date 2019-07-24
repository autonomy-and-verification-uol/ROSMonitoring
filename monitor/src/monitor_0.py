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
from std_msgs.msg import Int32
from rosmon.msg import Person

pubchatter = rospy.Publisher(name = 'chatter', data_class = String, latch = True, queue_size = 1000)
def callbackchatter(data):
    global online, ws, ws_lock
    rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'chatter'
    dict['time'] = rospy.get_time()
    if online:
        ws_lock.acquire()
        ws.send(json.dumps(dict))
        ws_lock.release()
        rospy.loginfo('event propagated to oracle')
    else:
        logging(dict)
        pub_dict['chatter'].publish(data)

pubcount = rospy.Publisher(name = 'count', data_class = Int32, latch = True, queue_size = 1000)
def callbackcount(data):
    global online, ws, ws_lock
    rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'count'
    dict['time'] = rospy.get_time()
    if online:
        ws_lock.acquire()
        ws.send(json.dumps(dict))
        ws_lock.release()
        rospy.loginfo('event propagated to oracle')
    else:
        logging(dict)
        pub_dict['count'].publish(data)

pubperson = rospy.Publisher(name = 'person', data_class = Person, latch = True, queue_size = 1000)
def callbackperson(data):
    global online, ws, ws_lock
    rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = 'person'
    dict['time'] = rospy.get_time()
    if online:
        ws_lock.acquire()
        ws.send(json.dumps(dict))
        ws_lock.release()
        rospy.loginfo('event propagated to oracle')
    else:
        logging(dict)
        pub_dict['person'].publish(data)

pub_dict = {
    'chatter' : pubchatter,
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
    rospy.Subscriber('chatter_mon', String, callbackchatter)
    rospy.Subscriber('count_mon', Int32, callbackcount)
    rospy.Subscriber('person_mon', Person, callbackperson)
    rospy.loginfo('monitor started and ready: ' + ('Online' if online else 'Offline'))

def on_message(ws, message):
    global error, log, actions
    json_dict = json.loads(message)
    if 'error' in json_dict:
        logging(json_dict)
        print('The event ' + message + ' is inconsistent..')
        print('actions:' + actions[json_dict['topic']])
        if actions[json_dict['topic']] == 'filter':
            rospy.loginfo('Not republished..')
        elif actions[json_dict['topic']] == 'warning':
            error = MonitorError()
            error.topic = json_dict['topic']
            error.time = json_dict['time']
            error.property = json_dict['spec']
            del json_dict['topic']
            del json_dict['time']
            del json_dict['spec']
            error.content = json.dumps(json_dict)
            pub_error.publish(error)
        else:
            rospy.loginfo('Let it go..')
            topic = json_dict['topic']
            del json_dict['topic']
            del json_dict['time']
            del json_dict['error']
            del json_dict['spec']
            ROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)
            pub_dict[topic].publish(ROS_message)
    	error = True
    else:
        logging(json_dict)
        topic = json_dict['topic']
        del json_dict['topic']
        del json_dict['time']
    	rospy.loginfo('The event ' + message + ' is consistent and republished')
        ROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)
    	pub_dict[topic].publish(ROS_message)

def logging(json_dict):
    try:
        with open(log, 'a+') as log_file:
            log_file.write(json.dumps(json_dict) + '\n')
        rospy.loginfo('event logged')
    except:
        rospy.loginfo('Unable to log the event.')

def on_error(ws, error):
    rospy.loginfo(error)

def on_close(ws):
	rospy.loginfo('### websocket closed ###')

def on_open(ws):
	rospy.loginfo('### websocket is open ###')

def main(argv):
    global log, actions, online, ws
    with open('/media/angelo/WorkData/git/ROSMonitoringCuriosity/ROSMonitoring/monitor/src/monitor_0.yaml', 'r') as stream:
        try:
            config = yaml.safe_load(stream) # load the config file
            if 'monitor' in config:
                if 'when' in config['monitor']:
                    if 'log' in config['monitor']:
                        log = config['monitor']['log']
                    else:
                        log = './monitor_0_log.txt'
                    if config['monitor']['when'] == 'offline': #offline RV
                        online = False
                        monitor()
                        rospy.spin()
                    else: # online RV
                        online = True
                        if 'oracle' in config['monitor'] and 'url' in config['monitor']['oracle']:
                            url = config['monitor']['oracle']['url']
                        else:
                            url = '127.0.0.1'
                        if 'oracle' in config['monitor'] and 'port' in config['monitor']['oracle']:
                            port = config['monitor']['oracle']['port']
                        else:
                            port = '8080'

                        actions = {
                            'chatter' : "log",
                            'count' : "filter",
                            'person' : "warning"
                        }
                        monitor()
                    	websocket.enableTrace(True)
                    	ws = websocket.WebSocketApp(
                            'ws://' + url + ':' + str(port),
                            on_message = on_message,
                            on_error = on_error,
                            on_close = on_close,
                            on_open = on_open)
                    	ws.run_forever()
                else:
                    print('monitor config file must contain the key \'when\' with values \'offline\' or \'online\'')
            else:
                print('monitor config file must contain the key \'monitor\'')
        except yaml.YAMLError as exc:
            print(exc)

if __name__ == '__main__':
    main(sys.argv)
