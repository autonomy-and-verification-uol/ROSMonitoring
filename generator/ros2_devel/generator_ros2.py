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
from jedi.inference.names import AbstractNameDefinition
from prompt_toolkit.layout.controls import GetLinePrefixCallable


class MonitorGenerator():
    
    def __init__(self):
        self.queue_size = 1000
        self.mon_pubs_dict_name = 'self.monitor_publishers'
        self.config_pubs_dict_name = 'self.config_publishers'
        self.config_subs_dict_name = 'self.config_subscribers'
        self.messages_dict_name = 'self.dict_msgs'
        self.threading_loc_name = 'self.ws_lock'
        self.websocket_name = 'self.ws'
        self.logging_fname = 'self.logging'
        self.message_received_fname = 'self.on_message'
        self.monitor_id_vname = 'self.name'
        self.mon_name_input = 'monitor_name'
        self.log_name_input = 'log'
        self.actions_name_input = 'actions'
        self.log_name = 'self.logfn'
        self.pub_topics_name = 'self.publish_topics'
        self.publish_topics = None
        self.topics_info = 'self.topics_info'
        self.indent_level=0
    

    def reset_indent(self,msg):
        print(msg+" Resetting indent level from {0} to 0".format(self.indent_level))
        self.indent_level=0
        
    def check_indent(self,msg):
        print(msg+" Indent level: {0}".format(self.indent_level))
        
    def create_logging_func(self):
        self.reset_indent("logging func start")
        lineprefix = ''
        input_var = 'json_dict'
        header = "def {logfuncname}({invar}):\n".format(logfuncname=self.logging_fname.replace("self.", ""),invar=input_var)
        lines=[header]
        
        lineprefix = self.inc_indent(lineprefix)
        line = "try:\n"
        lines.append(lineprefix+line)
        lineprefix = self.inc_indent(lineprefix)
        line = "with open({logname},'a+') as log_file:\n".format(logname=self.log_name)
        lines.append(lineprefix+line)
        lineprefix = self.inc_indent(lineprefix)
        line = "log_file.write(json.dumps({jd})+'\\n')\n".format(jd=input_var)
        lines.append(lineprefix+line)
        lineprefix = self.dec_indent(lineprefix)
        msg = "'Event logged'"
        line = self.get_ros_info_logging_line(msg)
        lines.append(lineprefix+line)
        lineprefix = self.dec_indent(lineprefix)
        
        
        line="except:\n"
        lines.append(lineprefix+line)
        lineprefix = self.inc_indent(lineprefix)
        msg = "'Unable to log the event'"
        line = self.get_ros_info_logging_line(msg)
        lines.append(lineprefix+line)
        
        lineprefix = self.dec_indent(lineprefix)
        lineprefix = self.dec_indent(lineprefix)
        
        self.check_indent("logging func done")
        return lines
        

    def create_on_message(self,silent,oracle_action,tp_lists):
        self.reset_indent("on message func start")
        lineprefix =''
        msg_input_var = 'message'
        header ="def {onmsgfunc}({msg_input}):\n".format(onmsgfunc = self.message_received_fname.replace("self.",""),msg_input = msg_input_var)
        lines=[header]
        
        lineprefix = self.inc_indent(lineprefix)
        jsondict = 'json_dict'
        line = "{jd} = json.loads({invar})\n".format(jd=jsondict,invar=msg_input_var)
        lines.append(lineprefix+line)
        line = "verdict = {jd}['verdict']\n".format(jd=jsondict)
        lines.append(lineprefix+line)
        line = "if verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown':\n"
        lines.append(lineprefix+line)
        
        lineprefix = self.inc_indent(lineprefix)
        
        line = "if verdict == 'true' and not {pt_var}:\n".format(pt_var=self.pub_topics_name)
        lines.append(lineprefix+line)
        lineprefix=self.inc_indent(lineprefix)
        msg = "'The monitor concluded the satisfaction of the property under analysis and can be safely removed.'"
        line = self.get_ros_info_logging_line(msg)
        lines.append(lineprefix+line)
        line = "{ws}.close()\n".format(ws=self.websocket_name)
        lines.append(lineprefix+line)
        line = "exit(0)\n"
        lines.append(lineprefix+line)
        
        lineprefix = self.dec_indent(lineprefix)
        line = "else:\n"
        lines.append(lineprefix+line)
        lineprefix = self.inc_indent(lineprefix)
        line = "{logging_fname}({data_dname})\n".format(logging_fname=self.logging_fname, data_dname=jsondict)
        lines.append(lineprefix+line)
        line = "topic = {jsondict}['topic']\n".format(jsondict=jsondict)
        lines.append(lineprefix+line)
        
        if not silent:
            msg = "'The event {data} is consistent and republished'".format(data = msg_input_var)
            line = self.get_ros_info_logging_line(msg)
            lines.append(lineprefix+line)
        if oracle_action == 'nothing':
            line = "if topic in {pubdict}:\n".format(pubdict=self.config_pubs_dict_name)
            lines.append(lineprefix+line)
            lineprefix = self.inc_indent(lineprefix)
            line = "{pubdict}[topic].publish({msgdict}[{jsond}['time']])\n".format(pubdict=self.config_pubs_dict_name,msgdict=self.messages_dict_name,jsond=jsondict)
            lines.append(lineprefix+line)
            lineprefix = self.dec_indent(lineprefix)
            line = "del {msgdict}[{jsond}['time']]\n".format(msgdict=self.messages_dict_name,jsond=jsondict)
            lines.append(lineprefix+line)
            lineprefix = self.dec_indent(lineprefix)
        else:
            lineprefix = self.inc_indent(lineprefix)
            line = "del {jsond}['topic']\n".format(jsond=jsondict)
            lines.append(lineprefix+line)
            line = "del {jsond}['time']\n".format(jsond=jsondict)
            lines.append(lineprefix+line)
            line = "ROS_message = eval({topicsinfo}[topic]['type']())\n".format(topicsinfo=self.topics_info)
            lines.append(lineprefix+line)
            line = "rosidl_runtime_py.set_message_fields(ROS_message,{jsond})\n".format(jsond=jsondict)
            lines.append(lineprefix+line)
           
            line = "if topic in {pubdict}:\n".format(pubdict=self.config_pubs_dict_name)
            lines.append(lineprefix+line)
            lineprefix = self.inc_indent(lineprefix)
            line = "{pubdict}.publish(ROS_message)\n".format(pubdict=self.config_pubs_dict_name)
            lines.append(lineprefix+line)
            lineprefix=self.dec_indent(lineprefix)
            lineprefix=self.dec_indent(lineprefix)
        
        lineprefix = self.dec_indent(lineprefix)
        
        # in the else for the first if 
        line = "else:\n"
        lines.append(lineprefix+line)
        lineprefix = self.inc_indent(lineprefix)
        
        # if the verdict is not ture or unknown 
        line = "{logfunc}({jsond})\n".format(logfunc=self.logging_fname,jsond=jsondict)
        lines.append(lineprefix+line)
        
        if not silent:
            msg = "'The event' + {msg} + ' is inconsistent' ".format(msg=msg_input_var)
            line = self.get_ros_info_logging_line(msg)
            lines.append(lineprefix+line)
        
        manylines = ["error = MonitorError()\n",
                     "error.m_topic = {0}['topic']\n".format(jsondict),
                     "error.m_time = {0}['time']\n".format(jsondict),
                     "error.m_property = {0}['spec']\n".format(jsondict),
                     ]
        lines=self.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        
        if oracle_action == 'nothing':
            line = "error.m_content = str({dmsgs}[{jsond}['time']])\n".format(dmsgs=self.messages_dict_name,jsond=jsondict)
            lines.append(lineprefix+line)
        else:
            manylines = ["{jsond}_copy = {jsond}.copy()\n".format(jsond=jsondict),
                         "del {jsond}_copy['topic']\n".format(jsond=jsondict),
                         "del {jsond}_copy['time']\n".format(jsond=jsondict),
                         "del {jsond}_copy['spec']\n".format(jsond=jsondict),
                         "del {jsond}_copy['error']\n".format(jsond=jsondict),
                         "error.m_content = json.dumps({jsond}_copy)\n".format(jsond=jsondict)
                         ]
            lines = self.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
        line = "{monpubs}['error'].publish(error)\n".format(monpubs=self.mon_pubs_dict_name)
        lines.append(lineprefix+line)
        line = "if verdict == 'false' and not {pt_var}:\n".format(pt_var=self.pub_topics_name)
        lines.append(lineprefix+line)
        lineprefix = self.inc_indent(lineprefix)
        
        msg = "'The monitor concluded the violation of the property under analysis and can be safely removed.'"
        line = self.get_ros_info_logging_line(msg)
        lines.append(lineprefix+line)
        line = "{ws}.close()\n".format(ws=self.websocket_name)
        lines.append(lineprefix+line)
        line = "exit(0)\n"
        lines.append(lineprefix+line)
        
        lineprefix = self.dec_indent(lineprefix)
        line = "if actions[{jsond}['topic']][0] != 'filter':\n".format(jsond=jsondict)
        lines.append(lineprefix+line)
        lineprefix = self.inc_indent(lineprefix)
        line = "topic = {jsond}['topic']\n".format(jsond=jsondict)
        lines.append(lineprefix+line)
        
        if oracle_action == 'nothing':
            line = "if topic in {pubd}:\n".format(pubd=self.config_pubs_dict_name)
            lines.append(lineprefix+line)
            lineprefix = self.inc_indent(lineprefix)
            line = "{pubd}[topic].publish({dmsgs}[{jsond}['time']])\n".format(pubd=self.config_pubs_dict_name, dmsgs=self.messages_dict_name,jsond=jsondict)
            lines.append(lineprefix+line)
            lineprefix = self.dec_indent(lineprefix)
            line = "del {dmsgs}[{jsond}['time']]\n".format( dmsgs=self.messages_dict_name,jsond=jsondict)
            lines.append(lineprefix+line)
            
            
        else:
            manylines = [   "del {jsond}['topic']\n".format(jsond=jsondict),
                         "del {jsond}['time']\n".format(jsond=jsondict),
                         "del {jsond}['spec']\n".format(jsond=jsondict),
                         "del {jsond}['error']\n".format(jsond=jsondict)
                ]
            lines=self.append_lines_to_list_with_prefix(lines, manylines, lineprefix)
            
            line = "ROS_message = eval({topicsinfo}[topic]['type']())\n".format(topicsinfo=self.topics_info)
            lines.append(lineprefix+line)
            line = "rosidl_runtime_py.set_message_fields(ROS_message,{jsond})\n".format(jsond=jsondict)
            lines.append(lineprefix+line)
           
            line = "if topic in {pubdict}:\n".format(pubdict=self.config_pubs_dict_name)
            lines.append(lineprefix+line)
            lineprefix = self.inc_indent(lineprefix)
            line = "{pubdict}.publish(ROS_message)\n".format(pubdict=self.config_pubs_dict_name)
            lines.append(lineprefix+line)
            lineprefix = self.dec_indent(lineprefix)
            
        lineprefix = self.dec_indent(lineprefix)
            
        line="error=True\n"
        lines.append(lineprefix+line)   
        lineprefix = self.dec_indent(lineprefix)
        line = "verdict_msg = String()\n"
        lines.append(lineprefix+line)
        line = "verdict_msg.data = verdict\n"
        lines.append(lineprefix+line)
        line = "{monpubs}['verdict'].publish(verdict_msg)\n".format(monpubs=self.mon_pubs_dict_name)
        lines.append(lineprefix+line)
        
        self.check_indent("on message func done")    
        return lines
            
    
    def get_variable_init_lines(self):
        lines = [
            "{dname}={{}}\n".format(dname=self.mon_pubs_dict_name),
            "{dname}={{}}\n".format(dname=self.config_pubs_dict_name),
            "{dname}={{}}\n".format(dname=self.config_subs_dict_name),
            "{dname}={{}}\n".format(dname=self.messages_dict_name),
            "{varname}=Lock()\n".format(varname=self.threading_loc_name),
            "{0}={1}\n".format(self.monitor_id_vname,self.mon_name_input),
            
            "{0}={1}\n".format(self.log_name,self.log_name_input),
            "{tpinfo}={{}}\n".format(tpinfo=self.topics_info)
            ]
        
        return lines
    
    def create_init_func(self,monitor_id,subscribers,tp_lists):
        self.reset_indent("init func start")
        lineprefix = ''
        
        header = "def __init__(self,{0},{1},{2}):\n".format(self.mon_name_input,self.log_name_input,self.actions_name_input)
        lines=[header]
        
        lineprefix = self.inc_indent(lineprefix)
        
        v_init_lines = self.get_variable_init_lines()
        lines = self.append_lines_to_list_with_prefix(lines, v_init_lines, lineprefix)
        
        # ros init line
        rosline = "super().__init__({monname})\n".format(monname=self.monitor_id_vname)
        lines.append(lineprefix+rosline)
        
        mon_publishers = self.create_inherent_monitor_publisher_lines()
        lines = self.append_lines_to_list_with_prefix(lines, mon_publishers, lineprefix)

        publines = self.create_config_publishers_lines(subscribers,tp_lists)
        lines = self.append_lines_to_list_with_prefix(lines, publines, lineprefix)
        
        if self.publish_topics is not None:
            line = "{pb}={val}\n".format(pb=self.pub_topics_name,val=self.publish_topics)
            lines.append(lineprefix+line)
            
        tlines = self.create_topics_info_dict(tp_lists)
        lines = self.append_lines_to_list_with_prefix(lines, tlines, lineprefix)
        
        self.check_indent("init funct end")
        return lines
    
    def create_topics_info_dict(self,tp_lists):
        lines=[]
        for t in tp_lists:
            line = "{t_info_var}[{tname}]={tdict}\n".format(t_info_var=self.topics_info,tname=t,tdict=tp_lists[t])
            lines.append(line)
            
        return lines
    
    
    def append_lines_to_list_with_prefix(self,linelist,lines,lineprefix):
        for l in lines:
            linelist.append(lineprefix+l)
        return linelist
     
    def create_mon_file_lines(self,topics_with_types_and_action,monitor_id,silent,oracle_action,oracle_url,oracle_port):
        self.reset_indent("mon file creation start")
        lineprefix = ''
        lines = []
        new_line = '\n'
        # create the python header 
        h_line = self.create_python_header()
        lines.append(lineprefix+h_line)
        lines.append(new_line)
        # add the import lines  
        tp_lists= self.get_topic_msg_types(topics_with_types_and_action)
        subscribers = self.get_subscribers(topics_with_types_and_action)
        i_lines = self.create_import_lines(tp_lists)
        lines = self.append_lines_to_list_with_prefix(lines, i_lines, lineprefix)
        lines.append(new_line)
        # create the class header
        c_line = self.create_class_header(monitor_id)
        lines.append(lineprefix+c_line)
        lines.append(new_line)
        # increment the line prefix 
        lineprefix = self.inc_indent(lineprefix)
        
        # do the init funciton 
        init_lines = self.create_init_func(monitor_id,subscribers,tp_lists)
        lines = self.append_lines_to_list_with_prefix(lines, init_lines, lineprefix)
        lines.append(new_line)
        
        # do the callbacks 
        config_callbacks = self.create_config_callbacks(subscribers, tp_lists, silent, oracle_action, oracle_url, oracle_port)
        # for now just print the call back functions 
        for t in config_callbacks:
            cblines = config_callbacks[t]['lines']
            lines.append(new_line)
            lines=self.append_lines_to_list_with_prefix(lines, cblines, lineprefix)
        
        # do on message 
        lines.append(new_line)    
        if oracle_url !=None and oracle_port != None:
            on_message_lines = self.create_on_message(silent, oracle_action, tp_lists)
            lines=self.append_lines_to_list_with_prefix(lines,on_message_lines,lineprefix)
            
        lines.append(new_line)
        
        # do logging 
        logging_lines = self.create_logging_func()
        lines=self.append_lines_to_list_with_prefix(lines, logging_lines, lineprefix)
        lines.append(new_line)
        
        self.check_indent("mon file creation func done")
        return lines
    
        
    def create_python_header(self):
        return '#!/usr/bin/env python\n'
    
    
    
    ''' get the message types for the topics '''
    def get_topic_msg_types(self, topics_with_types_and_action):
        tp_lists = {}
        for topic_msg_details in topics_with_types_and_action:
            package = topic_msg_details['type'][0:topic_msg_details['type'].rfind('.')]
            type = topic_msg_details['type'][topic_msg_details['type'].rfind('.') + 1:]
            topic = topic_msg_details['name']
            tp_lists[topic] = {'package':package, 'type':type}
        return tp_lists
                  
    def create_import_lines(self, tp_lists):
        plain_import = ['json',
                        'yaml',
                        'websocket',
                        'sys',
                        'rclpy',
                        'rosidl_runtime_py']
        from_import = {'rclpy.node':'Node',
                     'threading':'*',
                     'rosmonitoring_interfaces.msg':'MonitorError',
                     'std_msgs.msg':'String'}
        
        ''' generate import lines for all the other message types '''
        for tp in tp_lists:
            package = tp_lists[tp]['package']
            type = tp_lists[tp]['type']
            if not package in from_import:
                from_import[package] = type
        
        ''' now lets generate the lines  '''
        import_lines = ["# begin imports\n"]
        for package in plain_import:
            line = "import {0}\n".format(package)
            import_lines.append(line)
            
        for package in from_import:
            line = "from {p} import {t}\n".format(p=package, t=from_import[package])
            import_lines.append(line)
        import_lines.append("# done import\n")
            
        return import_lines
    
    def create_class_header(self,monitor_id):
        return "class ROSMonitor_{0}(Node):\n".format(monitor_id)
    
    def get_subscribers(self, topics_with_types_and_action):
        subscribers = {}
        for tp_info in topics_with_types_and_action:
            if 'publishers' in tp_info:
                subscribers[tp_info['name']] = {'remapped':True, 'callback':True, 'republish':True}
            elif 'subscribers' in tp_info:
                subscribers[tp_info['name']] = {'remapped':False, 'callback':True, 'republish':True}
            else:
                subscribers[tp_info['name']] = {'remapped':False, 'callback':True, 'republish':False}
        return subscribers
    
    def create_config_publishers(self,subscribers,tp_lists):
        publishers={}
        for topic in subscribers:
            publishers[topic] = self.create_publisher_line(topic, subscribers[topic], tp_lists[topic])
            
        return publishers
    
    def create_config_publishers_lines_from_dict(self,publishers):
        lines = []
        for t in publishers:
            if publishers[t] != None:
                line = "{config_pub_dname}[{tname}]={publine}\n".format(config_pub_dname=self.config_pubs_dict_name,tname=t,publine=publishers[t])
                lines.append(line)
        if len(lines) == 0:
            self.publish_topics = False
        else:
            self.publish_topics = True
        return lines
    
    def create_config_publishers_lines(self,subscribers,tp_lists):
        publishers = self.create_config_publishers(subscribers,tp_lists)
        return self.create_config_publishers_lines_from_dict(publishers)    
        
    def create_config_callbacks(self, subscribers, tp_lists,silent,oracle_action,oracle_url,oracle_port):
        callbacks = {}
        for topic in subscribers:
            callbacks[topic] = self.create_callback_func(topic,subscribers[topic],tp_lists[topic],silent,oracle_action,oracle_url,oracle_port)
            
        return callbacks
    
    def get_remapped_name(self, name):
        return name + "_mon"
    
    def get_ros_info_logging_line(self, text):
        return 'self.get_logger().info({0})\n'.format(text)
    
    def get_ros_time_line(self):
        return 'float(self.get_clock().now().to_msg().sec)'
    
    def inc_indent(self, current_indent):
        self.indent_level+=1
        return current_indent + "\t"
    
    def dec_indent(self, current_indent):
        if self.indent_level <=0:
            print("Error with the indentation perhaps, trying to decrement an indent incorrectly")
        if current_indent == '':
            return current_indent
        else:
            # from https://stackoverflow.com/questions/2556108/rreplace-how-to-replace-the-last-occurrence-of-an-expression-in-a-string
            toremove = "\t"
            replacewith = ""
            maxreplace = 1
            new_indent = replacewith.join(current_indent.rsplit(toremove, maxreplace))
            self.indent_level-=1
            return new_indent
    
    def create_callback_func(self, tname, tinfo, tmsg_type, silent, oracle_action, oracle_url, oracle_port):
        self.reset_indent("callback func start")
        tpname = tname
        if tinfo['remapped']:
            tpname = self.get_remapped_name(tpname)
        lineprefix = self.inc_indent('')
        func_name = "callback{tname}".format(tname=tname)
        func_input_varname = 'data'
        header = "def {fname}({f_input}):\n".format(fname=func_name, f_input=func_input_varname)
        lines = [header]
        
        data_dict_name = "dict"

        # log output if not silent
        if not silent:
            message = '"monitor has observed "+ str({0})'.format(func_input_varname)
            
            line = self.get_ros_info_logging_line(message)
            lines.append(lineprefix + line)
        # convert the data to send to the oracle or log
        line = "{0}= rosidl_runtime_py.message_to_ordereddict({1})\n".format(data_dict_name, func_input_varname)
        lines.append(lineprefix + line)
        
        line = "{data_dict_name}['topic']='{tname}'\n".format(data_dict_name=data_dict_name, tname=tname)
        lines.append(lineprefix + line)
        
        line = "{data_dict_name}['time']={ros_time}\n".format(data_dict_name=data_dict_name, ros_time=self.get_ros_time_line())
        lines.append(lineprefix + line)
        
        line = "{ws_lock}.acquire()\n".format(ws_lock=self.threading_loc_name)
        lines.append(lineprefix + line)
        # making sure we don't overwrite in the dictionary
        if oracle_action == 'nothing':
            line = "while {data_dname}['time'] in {msg_dname}:\n".format(data_dname=data_dict_name, msg_dname=self.messages_dict_name)
            lines.append(lineprefix + line)
            lineprefix = self.inc_indent(lineprefix)
            line = "{data_dname}['time']+=0.01\n".format(data_dname=data_dict_name)
            lines.append(lineprefix + line)
            lineprefix = self.dec_indent(lineprefix)
            
        do_oracle = oracle_url != None and oracle_port != None
        log_msg = "event "
        oracle_response_varname = "message"   
        # if online monitor then we need to send things to the oracle
        if do_oracle:
            log_msg += "propagated to oracle" 
            line = "{ws}.send(json.dumps({data_dname}))\n".format(ws=self.websocket_name, data_dname=data_dict_name)
            lines.append(lineprefix + line)
            if oracle_action == 'nothing':
                line = "{msgs_dname}[{data_dname}['time']] = {input_varname}\n".format(msgs_dname=self.messages_dict_name, data_dname=data_dict_name, input_varname=func_input_varname)
                lines.append(lineprefix + line)
            line = "{msg}={ws}.recv()\n".format(msg=oracle_response_varname, ws=self.websocket_name)
            lines.append(lineprefix + line)
        else:
            log_msg += "successfully logged"
            line = "{logging_fname}({data_dname})\n".format(logging_fname=self.logging_fname, data_dname=data_dict_name)
            lines.append(lineprefix + line)
            if tinfo['republish']:
                # if we need to republish then go ahead and do that
                # TODO: add line here
                line = ""
                
        line = "{ws_lock}.release()\n".format(ws_lock=self.threading_loc_name)
        lines.append(lineprefix + line)
        
        if not silent:
            line = self.get_ros_info_logging_line('"{0}"'.format(log_msg))
            lines.append(lineprefix + line)
            
        if do_oracle:
            line = "{msg_fname}({msg_vname})\n".format(msg_fname=self.message_received_fname, msg_vname=oracle_response_varname)
            lines.append(lineprefix + line)
        
        self.check_indent("create callback func done")   
        return {'name':func_name, 'lines':lines}
        
    def ros_publisher_creation_command(self, pubname, pubtype, qsize):
        return "self.create_publishers(topic={tname},msg_type={ttype},qos_profile={qs})\n".format(tname=pubname, ttype=pubtype, qs=qsize)
    
    def create_publisher_line(self, name, tinfo, tmsg_type):
        if not tinfo['republish']:
            return None
        tpname = name 
        if tinfo['remapped']:
            tpname = self.get_remapped_name(tpname)
        pubtype = tmsg_type['type']
        line = self.ros_publisher_creation_command(tpname, pubtype, self.queue_size)
        return line
        
    def create_inherent_monitor_publisher_lines(self):
        pub_types = {'error':'MonitorError', 'verdict':'String'}
        comments = "# creating the verdict and error publishers for the monitor\n"
        lines = [comments]

        for pt in pub_types:
            pubname =  "{monname}+'/monitor_{pt}'".format(monname=self.monitor_id_vname,pt=pt)
            # "{}" + "'"+'/monitor_' + pt + "'"
            ros_pub_creation_line = self.ros_publisher_creation_command(pubname, pub_types[pt], self.queue_size)
            line = "{dictname}[{pubtype}]={ros_pub_line}\n".format(dictname=self.mon_pubs_dict_name, pubtype=pt, ros_pub_line=ros_pub_creation_line)
            lines.append(line)
        comments = "# done creating monitor publishers\n\n"
        lines.append(comments)

        return lines
    
    ''' this is an array of lines'''

    def write_lines(self, lines, monitor_id):
        with open('test_' + monitor_id + '.py', 'w') as mon:
            for l in lines:
                mon.write(l)
            
