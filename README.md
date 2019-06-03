# ROSMonitoring
Repository containing the Python implementation for integrating RML (Runtime Monitoring Language, https://github.com/RMLatDIBRIS) verification and ROS (https://www.ros.org/).

This repository contains two folders:
 - instrumentation
 - webserver

# Instrumentation

The instrumentation folder contains the generator program (Python). It can be used for instrumenting a ROS project (where the nodes are implemented in Python) and generating a monitor node for achieving the Runtime Verification of our ROS nodes.
This generator program takes a configuration file in input (the config.yaml contained in the same folder). Using this simple configuration file we can customize the instrumentation process.

The default config.yaml is:
```yaml
ROS:
 path: <path_to_ROS_project>
 topics: all
```
The 'path' item refers to the path to the ROS project we want to instrument.
The 'topics' is the list of topics we are interested in instrumenting (what the monitor will check at runtime).
The keyword 'all' is used instead of listing all the topics. Using 'all', we do not limit which topics will be 
instrumented, and we instrument all the topics used in all the Python nodes contained in the ROS project.

# Webserver

The webserver folder contains two subfolders: prolog and rml

The Prolog folder contains the prolog files implementing the semantics of the specification language chosen: RML.
In this folder we can find the semantics of the Trace Expression formalism and the implementation of a Webserver
prolog. This Webserver Prolog can be used as a bridge between ROS nodes and our specifications. Thanks to the 
instrumentation part, we can generate automatically a monitor which will communicate to the WebServer Prolog using WebSockets (in the case of Online Runtime Verification).

The other folder contains example of specifications using RML.

# How to use ROSMonitoring (through an example extracted by ROS Tutorial)

In order to show how to use ROSMonitoring, the fisrt thing we need to do is to create a ROS project.
We can get the example of ROS nodes in Python from:







