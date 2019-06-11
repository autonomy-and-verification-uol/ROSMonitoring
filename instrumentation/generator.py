import os
import yaml

def instrument_files(path, topics = None): # function which instruments all the python file in the path
    files = []
    topics_with_types = []
    for r, d, f in os.walk(os.path.expanduser(path), followlinks = True):
        for file in f: # for all the python files
            if '.py' in file and 'instrumented' not in file: # excluding the instrumented ones
                update_topics(os.path.join(r, file), topics_with_types, topics) # instrument the files generating for each one the corresponding '_instrumented' one
    return topics_with_types # return the topics instrumented with their additional information

def update_topics(file, topics_with_types, topics = None): # instrument the topics inside a single python file
    with open(file, 'r') as content_file: # open the file
        content = content_file.read() # read the content
        index = content.find('Publisher(') # search for a generic Publisher [start]
        index1 = content.find(')', index) # search for a generic Publisher [end]
        while index != -1:
            # extract the arguments from the Publisher instantiation
            name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size = parse_publisher_instantiation(content, index, index1)
            name = name.replace("'", '') # we remove the ' just because we use it as a key for the generation of monitor.py later on
            if topics == None or name in topics:
                typeImport = ''
                if name not in [t for (t, _, _, _, _, _, _) in topics_with_types]: # extract the import of the type of the topic
                    imp1 = content.find('from ')
                    while imp1 != -1:
                        imp2 = content.find('\n', imp1)
                        if content.find('import ' + data_class, imp1, imp2) != -1:
                            typeImport = content[imp1:imp2] # the import string used later for importing the type into the monitor python file generated
                            break
                        else:
                            imp1 = content.find('from ', (imp1+1))
                    topics_with_types.append((name, (data_class, typeImport), subscriber_listener, tcp_nodelay, latch, headers, queue_size)) # add the topic, its type, and the queue size
                    print((name, (data_class, typeImport), subscriber_listener, tcp_nodelay, latch, headers, queue_size)) # debug log
                # replace the content of the file with the new topic name (we decided to call it <topic>_mon, where <topic> is the previous value)
                content = content.replace(
                    content[index:(index1+1)],
                    '''Publisher(name = '{tp}', data_class = {ty}, subscriber_listener = {sbl}, tcp_nodelay = {tcpn}, latch = {la}, headers = {hd}, queue_size = {qs})'''.format(tp = (name + '_mon'), ty = data_class, sbl = subscriber_listener, tcpn = tcp_nodelay, la = latch, hd = headers, qs = queue_size))
            index = content.find('Publisher(', (index+1)) # search for the next Publisher defined into the file (if there is one)
            index1 = content.find(')', index)
    with open(file.replace(".py", "_instrumented.py"), 'w') as fout:
        fout.write(content) # update the instrumented file

def parse_publisher_instantiation(content, begin, end):
    # name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None
    name = 'None'
    data_class = 'None'
    subscriber_listener = 'None'
    tcp_nodelay = 'False'
    latch = 'False'
    headers = 'None'
    queue_size = 'None'
    args = [name, data_class, subscriber_listener, tcp_nodelay, latch, headers, queue_size]
    dict = {'name':0, 'data_class':1, 'subscriber_listener':2, 'tcp_nodelay':3, 'latch':4, 'headers': 5, 'queue_size':6}
    round = 0
    begin += 10
    stop = False
    while(not stop):
        comma = content.find(',', begin, end)
        if comma == -1:
            comma = end
            stop = True
        assignment = content.find('=', begin, comma)
        if assignment == -1:
            value = content[begin:comma].replace(' ', '')
            args[round] = value
        else:
            key = content[begin:assignment].replace(' ', '')
            value = content[(assignment+1):comma].replace(' ', '')
            if key in dict:
                args[dict[key]] = value
            else:
                print('Argument ' + key + ' in the Publisher instantiation has not been recognized.. ignored..')
        round += 1
        begin = comma + 1
    return args[0], args[1], args[2], args[3], args[4], args[5], args[6]

def create_monitor(topics_with_types, path): # function which creates the python ROS monitor
    if not os.path.exists('../ROSMonitor'):
        os.mkdir('../ROSMonitor')
    with open('../ROSMonitor/monitor.py', 'w') as monitor: # the monitor code will be in monitor.py
    # write the imports the monitor is gonna need
        imports = '''#!/usr/bin/env python
import rospy
import sys
import json
import yaml
import websocket
        '''
    # write the imports for the msg types used by the monitor (extracted by the previous instrumentation)
        msg_type_imports = ''
        for imp in set([imp for (_, (_, imp), _, _, _, _, _) in topics_with_types]):
            msg_type_imports += '''
{i}'''.format(i = imp)
    # write the creation of the publisher for each topic (and the callback function for the instrumented one)
        pub_with_callbacks = '\n'
        #name, data_class, subscriber_listener=None, tcp_nodelay=False, latch=False, headers=None, queue_size=None
        for (topic, (type, _), subscriber_listener, tcp_nodelay, latch, headers, queue_size) in topics_with_types:
            pub_with_callbacks += '''
pub{tp} = rospy.Publisher(name = '{tp}', data_class = {ty}, subscriber_listener = {sbl}, tcp_nodelay = {tcpn}, latch = {la}, headers = {hd}, queue_size = {qs})
def callback{tp}(data):
    global online, ws
    rospy.loginfo('monitor has observed: ' + str(data.data))
    if online:
        ws.send(json.dumps({{'topic' : '{tp}', 'data': data.data}}))
        rospy.loginfo('event propagated to oracle')
    else:
        logging({{ 'time' : rospy.get_time(), 'topic' : '{tp}', 'data' : data.data }})
        pub_dict['{tp}'].publish(data.data)
            '''.format(tp = topic, ty = type, sbl = subscriber_listener, tcpn = tcp_nodelay, la = latch, hd = headers, qs = queue_size)
    # write the dictionary for dynamically keeping track of the publishers
        pub_dict = '''
pub_dict = {'''
        first_time = True
        for (topic, (type, _), _, _, _, _, _) in topics_with_types:
            if(first_time):
                first_time = False
            else:
                pub_dict += ', '
            pub_dict += '''
    '{tp}' : pub{tp}'''.format(tp = topic)
        pub_dict += '''
}
        '''
    # write the definition of the monitor function which will be used to initialize the rosnode, and create the Subscribers for all the instrumented topics.
    # In short, the monitor observes the instrumented topics generated by the real nodes, and then it publishes (propagates) them
    # to the usual Subscribers
        monitor_def = '''
def monitor():
    with open(log, 'w') as log_file:
        log_file.write('')
    rospy.init_node('monitor', anonymous=True)'''
        for (topic, (type, _), _, _, _, _, _) in topics_with_types:
            monitor_def += '''
    rospy.Subscriber('{tp}_mon', {ty}, callback{tp})'''.format(tp = topic, ty = type)
        monitor_def += '''
    rospy.loginfo('monitor started and ready: ' + ('Online' if online else 'Offline'))
        '''
    # write the auxiliary callbacks functions called by the websocket used by the monitor
    # when a topic is observed by the monitor, if we are doing online RV, it propagates the topic to the
    # oracle. The oracle checks the event and returns the outcome to the monitor
    # the monitor then propagates the event to the other nodes (unless we decided to filter the errors,
    # in that case the monitor does not propagate teh event)
        other_callbacks = '''
def on_message(ws, message):
    global error, log, action
    jsonMsg = json.loads(message)
    if 'error' in jsonMsg:
        logging({{ 'time' : rospy.get_time(), 'topic' : jsonMsg['msg']['topic'], 'data' : jsonMsg['msg']['data'], 'error' : True }})
        print('The event ' + message + ' is inconsistent..')
        if action == 'filter':
            print('Not republished..')
        else:
            print('Let it go..')
            pub_dict[jsonMsg['msg']['topic']].publish(jsonMsg['msg']['data'])
    	error = True
    else:
        logging({{ 'time' : rospy.get_time(), 'topic' :  jsonMsg['topic'], 'data' : jsonMsg['data'] }})
    	print('The event ' + message + ' is consistent and republished')
    	pub_dict[jsonMsg['topic']].publish(jsonMsg['data'])

def logging(json_event):
    try:
        with open(log, 'a+') as log_file:
            log_file.write(json.dumps(json_event) + '\\n')
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
    global log, action, online, ws
    with open('{yaml_file}', 'r') as stream:
        try:
            config = yaml.safe_load(stream) # load the config file
            if 'monitor' in config:
                if 'when' in config['monitor']:
                    if 'log' in config['monitor']:
                        log = config['monitor']['log']
                    else:
                        log = './log.txt'
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
                        if 'action' in config['monitor']:
                            action = config['monitor']['action']
                        else:
                            action = 'log'
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
                    print('monitor config file must contain the key \\'when\\' with values \\'offline\\' or \\'online\\'')
            else:
                print('monitor config file must contain the key \\'monitor\\'')
        except yaml.YAMLError as exc:
            print(exc)

if __name__ == '__main__':
    main(sys.argv)
        '''.format(yaml_file =  os.path.abspath('../ROSMonitor/monitor.yaml'),ros_project = path, topics = [topic for (topic, _, _, _, _, _, _) in topics_with_types])
        monitor.write(imports + msg_type_imports + pub_with_callbacks + pub_dict + monitor_def + other_callbacks)

def create_monitor_config(): # function which creates the YAML config file whoch will be used by the monitor
    str = '''
monitor: # offline RV
  log: ./log.txt # file where the monitor will log the observed events
  when: offline # when the RV will be applied

# monitor: # online RV
#   action: log # default action (optional) # the other possible value is: filter
#   log: ./log.txt # file where the monitor will log the observed events
#   oracle: # the oracle running and ready to check the specification
#     port: 8080 # the port where it is listening
#     url: 127.0.0.1 # the url where it is listening
#   when: online # when the RV will be applied
  '''
    with open('../ROSMonitor/monitor.yaml', 'w') as yaml_file:
        yaml_file.write(str)
    #     yaml.dump(offline_config, yaml_file, default_flow_style=False)
    # with open('online.yaml', 'w') as yaml_file:
    #     yaml.dump(online_config, yaml_file, default_flow_style=False)
