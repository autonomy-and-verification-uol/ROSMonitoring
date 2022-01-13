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
import xml.etree.ElementTree as ET

def create_monitor(monitor_id, topics_with_types_and_action, log, url, port, oracle_action, silent, warning): # function which creates the python ROS monitor
    with open('../monitor/src/' + monitor_id + '.py', 'w') as monitor: # the monitor code will be in monitor.py
    # write the imports the monitor is gonna need
        imports = '''#!/usr/bin/env python
import rospy
import sys
import json
import yaml
import websocket
from threading import *
from rospy_message_converter import message_converter
from monitor.msg import *
from std_msgs.msg import String

ws_lock = Lock()'''
        if oracle_action == 'nothing':
            imports += '''
dict_msgs = {}
            '''
    # write the imports for the msg types used by the monitor (extracted by the previous instrumentation)
        msg_type_imports = ''
        msg_import_set = set()
        for topic_with_types_and_action in topics_with_types_and_action:
            package = topic_with_types_and_action['type'][0:topic_with_types_and_action['type'].rfind('.')]
            type = topic_with_types_and_action['type'][topic_with_types_and_action['type'].rfind('.')+1:]
            msg_import_set.update([(package, type)])
        for (package, type) in msg_import_set:
            msg_type_imports += '''
from {p} import {t}'''.format(p = package, t = type)
    # write the creation of the publisher for each topic (and the callback function for the instrumented one)
        pub_with_callbacks = '\n'
        for topic_with_types_and_action in topics_with_types_and_action:
            if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action:
                if 'subscribers' in topic_with_types_and_action:
                    tp_side = topic_with_types_and_action['name'] + '_mon'
                else:
                    tp_side = topic_with_types_and_action['name']
                pub_with_callbacks += '''
pub{tp} = rospy.Publisher(name = '{tps}', data_class = {ty}, latch = True, queue_size = 1000)'''.format(tp = topic_with_types_and_action['name'].replace('/','_'), tps = tp_side, ty = topic_with_types_and_action['type'][topic_with_types_and_action['type'].rfind('.')+1:])
            pub_with_callbacks += '''
def callback{tp}(data):
    global ws, ws_lock'''.format(tp = topic_with_types_and_action['name'].replace('/','_'))
            if not silent:
                pub_with_callbacks += '''
    rospy.loginfo('monitor has observed: ' + str(data))'''
            pub_with_callbacks += '''
    dict = message_converter.convert_ros_message_to_dictionary(data)
    dict['topic'] = '{tp}'
    dict['time'] = rospy.get_time()
    ws_lock.acquire()'''.format(tp = topic_with_types_and_action['name'])

            if oracle_action == 'nothing':
                pub_with_callbacks += '''
    while dict['time'] in dict_msgs:
        dict['time'] += 0.01'''
            if url != None and port != None:
                pub_with_callbacks += '''
    ws.send(json.dumps(dict))'''
                if oracle_action == 'nothing':
                    pub_with_callbacks += '''
    dict_msgs[dict['time']] = data'''
                pub_with_callbacks += '''
    ws_lock.release()'''
                if not silent:
                    pub_with_callbacks += '''
    rospy.loginfo('event propagated to oracle')'''
            else:
                pub_with_callbacks += '''
    logging(dict)'''
		pub_with_callbacks += '''
    ws_lock.release()'''
                if not silent:
                    pub_with_callbacks += '''
    rospy.loginfo('event has been successfully logged')'''
                if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action:
                    pub_with_callbacks += '''
    pub_dict['{tp}'].publish(data)'''.format(tp = topic_with_types_and_action['name'])
    # write the dictionary for dynamically keeping track of the publishers
        pub_dict = '''

pub_dict = {'''
        first_time = True
        for topic_with_types_and_action in topics_with_types_and_action:
        #for (topic, (type, _), _, _, _, _, _) in topics_with_types:
            if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action:
                if(first_time):
                    first_time = False
                else:
                    pub_dict += ', '
                pub_dict += '''
    '{tp1}' : pub{tp2}'''.format(tp1 = topic_with_types_and_action['name'], tp2 = topic_with_types_and_action['name'].replace('/','_'))
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
    global pub_error, pub_verdict
    with open(log, 'w') as log_file:
        log_file.write('')
    rospy.init_node('{id}', anonymous=True)
    pub_error = rospy.Publisher(name = '{id}/monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)
    pub_verdict = rospy.Publisher(name = '{id}/monitor_verdict', data_class = String, latch = True, queue_size = 1000)'''.format(id = monitor_id)
        for topic_with_types_and_action in topics_with_types_and_action:
            if 'subscribers' in topic_with_types_and_action:
                tp_side = topic_with_types_and_action['name']
            elif 'publishers' in topic_with_types_and_action:
                tp_side = topic_with_types_and_action['name'] + '_mon'
            else:
                tp_side = topic_with_types_and_action['name']
            monitor_def += '''
    rospy.Subscriber('{tps}', {ty}, callback{tp})'''.format(tp = topic_with_types_and_action['name'].replace('/','_'), tps = tp_side, ty = topic_with_types_and_action['type'][topic_with_types_and_action['type'].rfind('.')+1:])
        if not silent:
            monitor_def += '''
    rospy.loginfo('monitor started and ready')
        '''
    # write the auxiliary callbacks functions called by the websocket used by the monitor
    # when a topic is observed by the monitor, if we are doing online RV, it propagates the topic to the
    # oracle. The oracle checks the event and returns the outcome to the monitor
    # the monitor then propagates the event to the other nodes (unless we decided to filter the errors,
    # in that case the monitor does not propagate the event)
        if url != None and port != None:
            other_callbacks = '''
def on_message(ws, message):
    global error, log, actions
    json_dict = json.loads(message)
    if json_dict['verdict'] == 'true' or json_dict['verdict'] == 'currently_true' or json_dict['verdict'] == 'unknown' or (json_dict['verdict'] == 'currently_false' and actions[json_dict['topic']][1] > 1):
        if json_dict['verdict'] == 'true' and not pub_dict:
            rospy.loginfo('The monitor concluded the satisfaction of the property under analysis, and can be safely removed.')
            ws.close()
            exit(0)
        else:
            logging(json_dict)
            topic = json_dict['topic']'''
            if not silent:
                other_callbacks += '''
            rospy.loginfo('The event ' + message + ' is consistent and republished')'''
            if oracle_action == 'nothing':
                other_callbacks += '''
            if topic in pub_dict:
                pub_dict[topic].publish(dict_msgs[json_dict['time']])
            del dict_msgs[json_dict['time']]'''
            else:
                other_callbacks += '''
            del json_dict['topic']
            del json_dict['time']
            ROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)
        	if topic in pub_dict:
                pub_dict[topic].publish(ROS_message)
            '''
            other_callbacks += '''
    else:
        logging(json_dict)
        # if (json_dict['verdict'] == 'false' and actions[json_dict['topic']][1] >= 1) or (json_dict['verdict'] == 'currently_false' and actions[json_dict['topic']][1] == 1):'''
            if not silent:
                other_callbacks += '''
        rospy.loginfo('The event ' + message + ' is inconsistent..')'''
            other_callbacks += '''
        error = MonitorError()
        error.topic = json_dict['topic']
        error.time = json_dict['time']
        error.property = json_dict['spec']'''
            if oracle_action == 'nothing':
                other_callbacks += '''
        error.content = str(dict_msgs[json_dict['time']])'''
            else:
                other_callbacks += '''
        json_dict_copy = json_dict.copy()
        del json_dict_copy['topic']
        del json_dict_copy['time']
        del json_dict_copy['spec']
        del json_dict_copy['error']
        error.content = json.dumps(json_dict_copy)'''
            other_callbacks += '''
        pub_error.publish(error)
        if json_dict['verdict'] == 'false' and not pub_dict:
            rospy.loginfo('The monitor concluded the violation of the property under analysis, and can be safely removed.')
            ws.close()
            exit(0)
        if actions[json_dict['topic']][0] != 'filter':
            # if json_dict['verdict'] == 'currently_false':
                # rospy.loginfo('The event ' + message + ' is consistent ')
            topic = json_dict['topic']'''
            if oracle_action == 'nothing':
                other_callbacks += '''
            if topic in pub_dict:
                pub_dict[topic].publish(dict_msgs[json_dict['time']])
            del dict_msgs[json_dict['time']]'''
            else:
                other_callbacks += '''
            del json_dict['topic']
            del json_dict['time']
            del json_dict['error']
            del json_dict['spec']
            ROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)
            if topic in pub_dict:
                pub_dict[topic].publish(ROS_message)'''
            other_callbacks += '''
    error = True'''
            other_callbacks += '''
    pub_verdict.publish(json_dict['verdict'])

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
            log_file.write(json.dumps(json_dict) + '\\n')'''
        if not silent:
            other_callbacks += '''
        rospy.loginfo('event logged')'''
        other_callbacks += '''
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
            # if 'warning' in topic_with_types_and_action:
            #     warning = topic_with_types_and_action['warning']
            # else:
            #     warning = 0
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
    websocket.enableTrace(False)
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

# instrument the launch files through adding/removing remap params
def instrument_launch_files(nodes):
    if not nodes: return
    launch_files = {}
    for name in nodes:
        (package, path, topics) = nodes[name]
        if path not in launch_files:
            launch_files[path] = []
        launch_files[path].append((name, package, topics))
    for path in launch_files:
        file_name = path.replace('.launch', '_instrumented.launch')
        tree = ET.parse(path)
        launch = tree.getroot()
        for node in launch.findall('node'):
            for (name, package, topics) in launch_files[path]:
                if node.get('name') == name and node.get('pkg') == package:
                    for topic in topics:
                        remap = ET.SubElement(node, 'remap')
                        remap.set('from', topic)
                        remap.set('to', topic + '_mon')
                    break
        tree.write(file_name)
