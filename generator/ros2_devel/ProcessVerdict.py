#!/usr/bin/env python

import json

class ProcessVerdict(object):
    
    def __init__(self,websocket,error,log,actions,publish_topics,publishers,roslogger):
        self.ws = websocket 
        self.error = error 
        self.logfn = log
        self.actions = actions 
        self.publish_topics = publish_topics
        self.publishers = publishers
        
    
    def true_or_unknown(self,verdict):
        return (verdict == 'true' or verdict == 'currently_true' or verdict == 'unknown')
        
    def on_message(self,message):
        json_dict = json.loads(message)
        verdict = json_dict['verdict']
        if self.true_or_unknown(verdict):
            if verdict == 'true' and not self.publish_topics:
                
                
            
    

