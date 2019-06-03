# ROSMonitoring
Repository containing the Python implementation for integrating RML (Runtime Monitoring Language, https://github.com/RMLatDIBRIS) verification and ROS (https://www.ros.org/).

This repository contains two folders:
 - instrumentation
 - webserver

# Instrumentation

The instrumentation folder contains the generator program (Python). It can be used for instrumenting a ROS project (where the nodes are implemented in Python) and generating a monitor node for achieving the Runtime Verification of our ROS nodes.
This generator program takes a configuration file in input (the config.yaml contained in the same folder). Using this simple configuration file we can customize the instrumentation process.

The default config.yaml is:

 ROS:
   
   path: <path_to_ROS_project>
   
   topics: all
 



