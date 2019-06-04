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

At the end of this tutorial you should have the talker and listener node working.
To run the example, follow the instructions at:
http://wiki.ros.org/ROS/Tutorials/ExaminingPublisherSubscriber

Ath the end of the tutorial, the talker and listener nodes should be able to communicate freely. 

Now we are ready to start monitoring our talker and listener nodes!

## Clone the ROSMonitoring repository

We need the ROSMonitoring implementation in order to instrument and verify our nodes. So, now is the time to clone the repository, if you have not already.

In the terminal:

 $ roscd beginner_tutorials/
 $ git clone https://github.com/autonomy-and-verification-uol/ROSMonitoring.git

Now you should have your local ROSMonitoring folder.

### Instrument talker and listener nodes

The first thing to do in order to monitor our nodes is to instrument them. Thanks to this step, our monitor will be able to intercept the topics of our interest (even though for now we have only the 'chatter' topic).

 $ cd ROSMonitoring/instrumentation/

Inside this folder you should find: config.yaml, generator, and generator.py.
The Python program we are going to execute is generator. But, before doing that, we need to change the configuration file, config.yaml. This configuration file allows us to select which ROS project we want to instrument, and which topics we are interested in ('all' is the keyword for considering all the topics used by the nodes). If we are interested in checking only a subset of the topics used by our nodes, following the syntax of YAML, we can list all the topics one by one instead.
We need to change <path_to_ROS_project> into ~/catkin_ws/src/beginner_tutorials
We can leave the topics list as it is. Note that in this case we could remove the keyword all, and add chatter, without changing the final outcome.

The new config.yaml file should look like this:
```yaml
#config file for the instrumentation of ROS
#this file is given in input to generator.py

ROS:
  path: ~/catkin_ws/src/beginner_tutorials/scripts/
  topics: all
```

Now we are ready to execute the generator.

$ chmod +x generator
$ ./generator
$ ./generator
{'path': '~/catkin_ws/src/beginner_tutorials/scripts/', 'topics': 'all'}
('chatter', ('String', 'from std_msgs.msg import String'), 'queue_size=10')







