# MIT License
#
# Copyright (c) [2019] [Angelo Ferrando]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
import yaml

def create_monitor(monitor_id, topics_with_types_and_action, log, url, port): # function which creates the python ROS monitor
    with open('../monitor/src/' + monitor_id + '.py', 'w') as monitor: # the monitor code will be in monitor.py
    # write the imports the monitor is gonna need
        imports = '''#!/usr/bin/env python
import rospy
import sys
import json
import yaml
import websocket
from threading import *

ws_lock = Lock()

from rospy_message_converter import message_converter
from monitor.msg import *
        '''
    # write the imports for the msg types used by the monitor (extracted by the previous instrumentation)
        msg_type_imports = ''
        for topic_with_types_and_action in topics_with_types_and_action:
            package = topic_with_types_and_action['type'][0:topic_with_types_and_action['type'].rfind('.')]
            type = topic_with_types_and_action['type'][topic_with_types_and_action['type'].rfind('.')+1:]
            msg_type_imports += '''
from {p} import {t}'''.format(p = package, t = type)
    # write the creation of the publisher for each topic (and the callback function for the instrumented one)
        pub_with_callbacks = '\n'
        for topic_with_types_and_action in topics_with_types_and_action:
            if 'side' in topic_with_types_and_action and topic_with_types_and_action['side'] == 'subscriber':
                tp_side = topic_with_types_and_action['name'] + '_mon'
            else:
                tp_side = topic_with_types_and_action['name']
            if 'side' in topic_with_types_and_action:
                if topic_with_types_and_action['side'] == 'subscriber':
                    tp_side = topic_with_types_and_action['name'] + '_mon'
                else:
                    tp_side = topic_with_types_and_action['name']
                pub_with_callbacks += '''
pub{tp} = rospy.Publisher(name = '{tps}', data_class = {ty}, latch = True, queue_size = 1000)'''.format(tp = topic_with_types_and_action['name'], tps = tp_side, ty = topic_with_types_and_action['type'][topic_with_types_and_action['type'].rfind('.')+1:])
            pub_with_callbacks += '''
def callback{tp}(data):
    global ws, ws_lock
    rospy.loginfo('monitor has observed: ' + str(data))
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = '{tp}'
    dict['time'] = rospy.get_time()'''.format(tp = topic_with_types_and_action['name'])
            if url != None and port != None:
                pub_with_callbacks += '''
    ws_lock.acquire()
    ws.send(json.dumps(dict))
    ws_lock.release()
    rospy.loginfo('event propagated to oracle')'''
            else:
                pub_with_callbacks += '''
    logging(dict)'''
                if 'side' in topic_with_types_and_action:
                    pub_with_callbacks += '''
    pub_dict['{tp}'].publish(data)'''.format(tp = topic_with_types_and_action['name'])
    # write the dictionary for dynamically keeping track of the publishers
        pub_dict = '''

pub_dict = {'''
        first_time = True
        for topic_with_types_and_action in topics_with_types_and_action:
        #for (topic, (type, _), _, _, _, _, _) in topics_with_types:
            if 'side' in topic_with_types_and_action:
                if(first_time):
                    first_time = False
                else:
                    pub_dict += ', '
                pub_dict += '''
    '{tp}' : pub{tp}'''.format(tp = topic_with_types_and_action['name'])
        pub_dict += '''
}
        '''
        first_time = True
    # write the dictionary for dynamically keeping track of the message types (we need it for the message converter)
        msg_dict = '''
msg_dict = {'''
        for topic_with_types_and_action in topics_with_types_and_action:
        #for (topic, (type, imp), _, _, _, _, _) in topics_with_types:
            if(first_time):
                first_time = False
            else:
                msg_dict += ', '
            msg_dict += '''
    '{tp}' : "{ty}"'''.format(tp = topic_with_types_and_action['name'], ty = topic_with_types_and_action['type'][0:topic_with_types_and_action['type'].rfind('.')].replace('.msg', '') + '/' + topic_with_types_and_action['type'][topic_with_types_and_action['type'].rfind('.')+1:])
        msg_dict += '''
}
        '''
    # write the definition of the monitor function which will be used to initialize the rosnode, and create the Subscribers for all the instrumented topics.
    # In short, the monitor observes the instrumented topics generated by the real nodes, and then it publishes (propagates) them
    # to the usual Subscribers
        monitor_def = '''
def monitor():
    global pub_error
    with open(log, 'w') as log_file:
        log_file.write('')
    rospy.init_node('monitor', anonymous=True)
    pub_error = rospy.Publisher(name = 'monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)'''
        for topic_with_types_and_action in topics_with_types_and_action:
            if 'side' in topic_with_types_and_action:
                if topic_with_types_and_action['side'] == 'subscriber':
                    tp_side = topic_with_types_and_action['name']
                else:
                    tp_side = topic_with_types_and_action['name'] + '_mon'
            else:
                tp_side = topic_with_types_and_action['name']
            monitor_def += '''
    rospy.Subscriber('{tps}', {ty}, callback{tp})'''.format(tp = topic_with_types_and_action['name'], tps = tp_side, ty = topic_with_types_and_action['type'][topic_with_types_and_action['type'].rfind('.')+1:])
        monitor_def += '''
    rospy.loginfo('monitor started and ready')
        '''
    # write the auxiliary callbacks functions called by the websocket used by the monitor
    # when a topic is observed by the monitor, if we are doing online RV, it propagates the topic to the
    # oracle. The oracle checks the event and returns the outcome to the monitor
    # the monitor then propagates the event to the other nodes (unless we decided to filter the errors,
    # in that case the monitor does not propagate teh event)
        if url != None and port != None:
            other_callbacks = '''
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
        if actions[json_dict['topic']][0] == 'filter':
            rospy.loginfo('Not republished..')
        else:
            rospy.loginfo('Let it go..')
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
    	rospy.loginfo('The event ' + message + ' is consistent and republished')
        ROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)
    	if topic in pub_dict:
            pub_dict[topic].publish(ROS_message)

def on_error(ws, error):
    rospy.loginfo(error)

def on_close(ws):
	rospy.loginfo('### websocket closed ###')

def on_open(ws):
	rospy.loginfo('### websocket is open ###')
            '''
        else:
            other_callbacks = ''
        other_callbacks += '''
def logging(json_dict):
    try:
        with open(log, 'a+') as log_file:
            log_file.write(json.dumps(json_dict) + '\\n')
        rospy.loginfo('event logged')
    except:
        rospy.loginfo('Unable to log the event.')

def main(argv):
    global log, actions, ws
    log = '{l}'
    '''.format(l = log)
        other_callbacks += '''
    actions = {'''
        first_time = True
        for topic_with_types_and_action in topics_with_types_and_action:
            if 'warning' in topic_with_types_and_action:
                warning = topic_with_types_and_action['warning']
            else:
                warning = False
            if(first_time):
                first_time = False
            else:
                other_callbacks += ', '
            other_callbacks += '''
            '{tp}' : ('{act}', {w})'''.format(tp = topic_with_types_and_action['name'], act = topic_with_types_and_action['action'], w = warning)
        if url != None and port != None:
            other_callbacks += '''
    }}
    monitor()
    websocket.enableTrace(True)
    ws = websocket.WebSocketApp(
        'ws://{u}:{p}',
        on_message = on_message,
        on_error = on_error,
        on_close = on_close,
        on_open = on_open)
    ws.run_forever()'''.format(u = url, p = port)
        else:
            other_callbacks += '''
    }
    monitor()
    rospy.spin()
            '''
        other_callbacks += '''
if __name__ == '__main__':
    main(sys.argv)
        '''
        monitor.write(imports + msg_type_imports + pub_with_callbacks + pub_dict + msg_dict + monitor_def + other_callbacks)

def create_launch_file(monitor_ids):
    with open('../monitor/run.launch', 'w') as launch_file:
        str = '''
<launch>
        '''
        for id in monitor_ids:
            str += '''
<node pkg="monitor" type="{monitor_id}.py" name="{monitor_id}" output="screen"/>
            '''.format(monitor_id = id)
        str += '''
</launch>
        '''
        launch_file.write(str)
