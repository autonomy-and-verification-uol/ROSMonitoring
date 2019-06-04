# ROSMonitoring
Repository containing the Python implementation for integrating RML (Runtime Monitoring Language, https://github.com/RMLatDIBRIS) verification and ROS (https://www.ros.org/).

This repository contains two folders:
 - instrumentation
 - monitor

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

# Monitor

The monitor folder contains two subfolders: prolog and rml

The Prolog folder contains the prolog files implementing the semantics of the specification language chosen: RML.
In this folder we can find the semantics of the Trace Expression formalism (the lower level calculus obtained compiling RML specifications). Beside the semantics, we have the implementation of a monitor in Prolog, both for Online and Offline RV. The Online RV is achieved through the use of Websockets; the monitor in Prolog consists in a Webserver listening on a chosen url and port. The ROS monitor generated through instrumentation will communicate the observed events at Runtime through this websocket connection. The Offline implementation is simpler, it simply consists in a Prolog implementation where a log file can be analysed offline (after the execution of the ROS system). Also in this case, the events checked by the monitor are obtained by the ROS monitor, which in the Offline scenario logs the observed events inside a log file. The same log file will be later analysed by the prolog monitor.

The other folder contains example of specifications using RML.

# How to use ROSMonitoring (through an example extracted by ROS Tutorial)

First things first..
Before going on we need a machine with ROS installed. It is not important which ROS distribution, as long as rospy is supported.

In the following we are going to use ROS Melodic with Catkin, but as mentioned before, you can use another distribution.

## Install ROS Kinetic
http://wiki.ros.org/melodic/Installation

## Create a workspace for catkin
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

## Create ROS package
http://wiki.ros.org/ROS/Tutorials/CreatingPackage
We need the 'beginner_tutorials' package, so do not forget to create it!

## Writing simple Publisher and Subscriber using rospy
http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
 








