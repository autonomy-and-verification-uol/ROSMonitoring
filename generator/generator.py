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
        imports = '''#!/usr/bin/env python\nimport rospy\nimport sys'''
        imports += '''\nimport json\nimport yaml\nimport websocket'''
        imports += '''\nfrom threading import *\nfrom rospy_message_converter import message_converter\nfrom monitor.msg import *'''
        imports += '''\nfrom std_msgs.msg import String\n\nws_lock = Lock()'''
        if oracle_action == 'nothing':
            imports += '''\ndict_msgs = {}'''
    # write the imports for the msg types used by the monitor (extracted by the previous instrumentation)
        msg_type_imports = ''
        msg_import_set = set()
        for topic_with_types_and_action in topics_with_types_and_action:
            package = topic_with_types_and_action['type'][0:topic_with_types_and_action['type'].rfind('.')]
            type = topic_with_types_and_action['type'][topic_with_types_and_action['type'].rfind('.')+1:]
            msg_import_set.update([(package, type)])
        for (package, type) in msg_import_set:
            msg_type_imports += '''\nfrom {p} import {t}'''.format(p = package, t = type)
    # write the creation of the publisher for each topic (and the callback function for the instrumented one)
        pub_with_callbacks = '\n'
        for topic_with_types_and_action in topics_with_types_and_action:
            if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action:
                if 'subscribers' in topic_with_types_and_action:
                    tp_side = topic_with_types_and_action['name'] + '_mon'
                else:
                    tp_side = topic_with_types_and_action['name']
                pub_with_callbacks += '''\npub{tp} = rospy.Publisher(name = '{tps}', data_class = {ty}, latch = True, queue_size = 1000)'''.format(tp = topic_with_types_and_action['name'].replace('/','_'), tps = tp_side, ty = topic_with_types_and_action['type'][topic_with_types_and_action['type'].rfind('.')+1:])
            pub_with_callbacks += '''\ndef callback{tp}(data):\n\tglobal ws, ws_lock'''.format(tp = topic_with_types_and_action['name'].replace('/','_'))
            if not silent:
                pub_with_callbacks += '''\n\trospy.loginfo('monitor has observed: ' + str(data))'''
            pub_with_callbacks += '''\n\tdict = message_converter.convert_ros_message_to_dictionary(data)\n\tdict['topic'] = '{tp}'\n\tdict['time'] = rospy.get_time()\n\tws_lock.acquire()'''.format(tp = topic_with_types_and_action['name'])

            if oracle_action == 'nothing':
                pub_with_callbacks += '''\n\twhile dict['time'] in dict_msgs:\n\t\tdict['time'] += 0.01'''
            if url != None and port != None:
                pub_with_callbacks += '''\n\tws.send(json.dumps(dict))'''
                if oracle_action == 'nothing':
                    pub_with_callbacks += '''\n\tdict_msgs[dict['time']] = data'''
                pub_with_callbacks += '''\n\tws_lock.release()'''
                if not silent:
                    pub_with_callbacks += '''\n\trospy.loginfo('event propagated to oracle')'''
            else:
                pub_with_callbacks += '''\n\tlogging(dict)'''
                pub_with_callbacks += '''\n\tws_lock.release()'''
                if not silent:
                    pub_with_callbacks += '''\n\trospy.loginfo('event has been successfully logged')'''
                if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action:
                    pub_with_callbacks += '''\n\tpub_dict['{tp}'].publish(data)'''.format(tp = topic_with_types_and_action['name'])
    # write the dictionary for dynamically keeping track of the publishers
        pub_dict = '''\npub_dict = {'''
        first_time = True
        for topic_with_types_and_action in topics_with_types_and_action:
        #for (topic, (type, _), _, _, _, _, _) in topics_with_types:
            if 'publishers' in topic_with_types_and_action or 'subscribers' in topic_with_types_and_action:
                if(first_time):
                    first_time = False
                else:
                    pub_dict += ', '
                pub_dict += ''' '{tp1}' : pub{tp2}'''.format(tp1 = topic_with_types_and_action['name'], tp2 = topic_with_types_and_action['name'].replace('/','_'))
        pub_dict += '''}'''
        first_time = True
    # write the dictionary for dynamically keeping track of the message types (we need it for the message converter)
        msg_dict = '''\nmsg_dict = {'''
        for topic_with_types_and_action in topics_with_types_and_action:
        #for (topic, (type, imp), _, _, _, _, _) in topics_with_types:
            if(first_time):
                first_time = False
            else:
                msg_dict += ', '
            msg_dict += ''' '{tp}' : "{ty}"'''.format(tp = topic_with_types_and_action['name'], ty = topic_with_types_and_action['type'][0:topic_with_types_and_action['type'].rfind('.')].replace('.msg', '') + '/' + topic_with_types_and_action['type'][topic_with_types_and_action['type'].rfind('.')+1:])
        msg_dict += '''}'''
    # write the definition of the monitor function which will be used to initialize the rosnode, and create the Subscribers for all the instrumented topics.
    # In short, the monitor observes the instrumented topics generated by the real nodes, and then it publishes (propagates) them
    # to the usual Subscribers
        monitor_def = '''\ndef monitor():\n\tglobal pub_error, pub_verdict\n\twith open(log, 'w') as log_file:'''
        monitor_def += '''\n\t\tlog_file.write('')\n\trospy.init_node('{id}', anonymous=True)'''.format(id = monitor_id)
        monitor_def += '''\n\tpub_error = rospy.Publisher(name = '{id}/monitor_error', data_class = MonitorError, latch = True, queue_size = 1000)\n\tpub_verdict = rospy.Publisher(name = '{id}/monitor_verdict', data_class = String, latch = True, queue_size = 1000)'''.format(id = monitor_id)
        for topic_with_types_and_action in topics_with_types_and_action:
            if 'subscribers' in topic_with_types_and_action:
                tp_side = topic_with_types_and_action['name']
            elif 'publishers' in topic_with_types_and_action:
                tp_side = topic_with_types_and_action['name'] + '_mon'
            else:
                tp_side = topic_with_types_and_action['name']
            monitor_def += '''\n\trospy.Subscriber('{tps}', {ty}, callback{tp})'''.format(tp = topic_with_types_and_action['name'].replace('/','_'), tps = tp_side, ty = topic_with_types_and_action['type'][topic_with_types_and_action['type'].rfind('.')+1:])
        if not silent:
            monitor_def += '''\n\trospy.loginfo('monitor started and ready')'''
    # write the auxiliary callbacks functions called by the websocket used by the monitor
    # when a topic is observed by the monitor, if we are doing online RV, it propagates the topic to the
    # oracle. The oracle checks the event and returns the outcome to the monitor
    # the monitor then propagates the event to the other nodes (unless we decided to filter the errors,
    # in that case the monitor does not propagate the event)
        if url != None and port != None:
            other_callbacks = '''\ndef on_message(ws, message):\n\tglobal error, log, actions\n\tjson_dict = json.loads(message)'''
            other_callbacks+= '''\n\tif json_dict['verdict'] == 'true' or json_dict['verdict'] == 'currently_true' or json_dict['verdict'] == 'unknown':'''
            other_callbacks+= '''\n\t\tif json_dict['verdict'] == 'true' and not pub_dict:\n\t\t\trospy.loginfo('The monitor concluded the satisfaction of the property under analysis, and can be safely removed.')'''
            other_callbacks+= '''\n\t\t\tws.close()\n\t\t\texit(0)'''
            other_callbacks+= '''\n\t\telse:\n\t\t\tlogging(json_dict)\n\t\t\ttopic = json_dict['topic']'''
            if not silent:
                other_callbacks += '''\n\t\t\trospy.loginfo('The event ' + message + ' is consistent and republished')'''
            if oracle_action == 'nothing':
                other_callbacks += '''\n\t\t\tif topic in pub_dict:\n\t\t\t\tpub_dict[topic].publish(dict_msgs[json_dict['time']])\n\t\t\tdel dict_msgs[json_dict['time']]'''
            else:
                other_callbacks += '''\n\t\t\tdel json_dict['topic']\n\t\t\tdel json_dict['time']\n\t\t\tROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)'''
                other_callbacks += '''\n\t\t\ttopic in pub_dict:\n\t\t\t\tpub_dict[topic].publish(ROS_message)'''
            other_callbacks += '''\n\telse:\n\t\tlogging(json_dict)\n\t\tif (json_dict['verdict'] == 'false' and actions[json_dict['topic']][1] >= 1) or (json_dict['verdict'] == 'currently_false' and actions[json_dict['topic']][1] == 1):'''
            if not silent:
                other_callbacks += '''\n\t\t\trospy.loginfo('The event ' + message + ' is inconsistent..')'''
            other_callbacks += '''\n\t\t\terror = MonitorError()\n\t\t\terror.topic = json_dict['topic']\n\t\t\terror.time = json_dict['time']\n\t\t\terror.property = json_dict['spec']'''
            if oracle_action == 'nothing':
                other_callbacks += '''\n\t\t\terror.content = str(dict_msgs[json_dict['time']])'''
            else:
                other_callbacks += '''\n\t\t\tjson_dict_copy = json_dict.copy()\n\t\t\tdel json_dict_copy['topic']\n\t\t\tdel json_dict_copy['time']'''
                other_callbacks+='''\n\t\t\tdel json_dict_copy['spec']\n\t\t\tdel json_dict_copy['error']\n\t\t\terror.content = json.dumps(json_dict_copy)'''
            other_callbacks += '''\n\t\t\tpub_error.publish(error)\n\t\t\tif json_dict['verdict'] == 'false' and not pub_dict:'''
            other_callbacks += '''\n\t\t\t\trospy.loginfo('The monitor concluded the violation of the property under analysis, and can be safely removed.')\n\t\t\t\tws.close()'''
            other_callbacks += '''\n\t\t\t\texit(0)'''
            other_callbacks +='''\n\t\tif actions[json_dict['topic']][0] != 'filter':'''
            other_callbacks +='''\n\t\t\tif json_dict['verdict'] == 'currently_false':\n\t\t\t\trospy.loginfo('The event ' + message + ' is consistent ')'''
            other_callbacks +='''\n\t\t\ttopic = json_dict['topic']'''
            if oracle_action == 'nothing':
                other_callbacks += '''\n\t\t\tif topic in pub_dict:\n\t\t\t\tpub_dict[topic].publish(dict_msgs[json_dict['time']])'''
                other_callbacks += '''\n\t\t\tdel dict_msgs[json_dict['time']]'''
            else:
                other_callbacks += '''\n\t\t\tdel json_dict['topic']\n\t\t\tdel json_dict['time']'''
                other_callbacks +='''\n\t\t\tdel json_dict['error']\n\t\t\tdel json_dict['spec']'''
                other_callbacks +='''\n\t\t\tROS_message = message_converter.convert_dictionary_to_ros_message(msg_dict[topic], json_dict)'''
                other_callbacks +='''\n\t\t\tif topic in pub_dict:\n\t\t\t\tpub_dict[topic].publish(ROS_message)'''
            other_callbacks += '''\n\t\terror = True'''
            other_callbacks += '''\n\tpub_verdict.publish(json_dict['verdict'])'''
            other_callbacks += '''\n\ndef on_error(ws, error):\n\trospy.loginfo(error)'''
            other_callbacks += '''\n\ndef on_close(ws):\n\trospy.loginfo('### websocket closed ###')'''
            other_callbacks += '''\n\ndef on_open(ws):\n\trospy.loginfo('### websocket is open ###')'''
        else:
            other_callbacks = ''
        other_callbacks += '''\n\ndef logging(json_dict):\n\ttry:\n\t\twith open(log, 'a+') as log_file:\n\t\t\tlog_file.write(json.dumps(json_dict) + '\\n')'''
        if not silent:
            other_callbacks += '''\n\t\trospy.loginfo('event logged')'''
        other_callbacks += '''\n\texcept:\n\t\trospy.loginfo('Unable to log the event.')'''
        other_callbacks +='''\n\ndef main(argv):\n\tglobal log, actions, ws\n\tlog = '{l}' '''.format(l = log)
        other_callbacks += '''\n\tactions = {'''
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
            other_callbacks += '''\n\t\t'{tp}' : ('{act}', {w})'''.format(tp = topic_with_types_and_action['name'], act = topic_with_types_and_action['action'], w = warning)
        if url != None and port != None:
            other_callbacks += '''\n\t}\n\tmonitor()\n\twebsocket.enableTrace(False)'''
            other_callbacks += '''\n\tws = websocket.WebSocketApp(\n\t\t'ws://{u}:{p}',\n\t\ton_message = on_message,
            \n\t\ton_error = on_error,\n\t\ton_close = on_close,\n\t\ton_open = on_open)\n\tws.run_forever()'''.format(u = url, p = port)
        else:
            other_callbacks += '''\n\t}\n\tmonitor()\n\trospy.spin()'''
        other_callbacks += '''\n\nif __name__ == '__main__':\n\tmain(sys.argv)'''
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
