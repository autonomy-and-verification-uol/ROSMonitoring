# Prerequisities

# Python:

## pip (https://pypi.org/project/pip/)
```bash
$ sudo apt install pip
```
Using pip we can then install the websocket library.
```bash
$ pip install websocket
```
# Prolog (http://www.swi-prolog.org/build/PPA.html):
```bash
$ sudo apt-get install software-properties-common
$ sudo apt-add-repository ppa:swi-prolog/stable
$ sudo apt-get update
$ sudo apt-get install swi-prolog
```

# Java (https://openjdk.java.net/install/):
For instance, installing openjdk 11 would be:
```bash
$ sudo add-apt-repository ppa:openjdk-r/ppa
$ sudo apt-get update
$ sudo apt-get install openjdk-11-jdk
```

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

At the end of the tutorial, the talker and listener nodes should be able to communicate freely. 

In order to simplify the monitoring process and make it easier, we need to change a small thing inside talker.py.

Line 47 must become:
```python
...
hello_str = "hello"
...
```

Now we are ready to start monitoring our talker and listener nodes!

## Clone the ROSMonitoring repository

We need the ROSMonitoring implementation in order to instrument and verify our nodes. So, now is the time to clone the repository, if you have not already.

In the terminal:

```bash
 $ roscd beginner_tutorials/
 $ git clone https://github.com/autonomy-and-verification-uol/ROSMonitoring.git
```
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

If we go back to the parent folder, we should now find a new folder called ROSMonitor. Inside this folder, two new files have been automatically generated: monitor.py and monitor.yaml
 - monitor.py is the Python definition of the monitor for ROS; its objective is to intercept the topics and log them (for Offline RV) or propagate them to the Webserver Prolog.
 - monitor.yaml is the configuration file for the monitor node.
 
Before going on, let us have a look at monitor.yaml
 
```yaml
monitor: # offline RV
 log: ./log.txt # file where the monitor will log the observed events
 when: offline # when the RV will be applied

# monitor: # online RV
#   action: log # default action (optional) # the other possible value is: filter
#   log: ./log.txt # file where the monitor will log the observed events
#   webserver: # the webserver running and ready to check the specification
#     port: 8080 # the port where it is listening
#     url: 127.0.0.1 # the url where it is listening
#   when: online # when the RV will be applied
```
 
The default configuration file for the monitor is set for Offline RV. In the commented part we have a possible use for the Online version. The YAML syntax is very intuitive, focusing for now on the Offline parameters, we can set where the events observed by the monitor will be saved (default here is log.txt), and at which time the RV will be applied (in this case Offline, setting the corresponding 'when' item).

The generator has not created the ROSMonitor folder, but it has also instrumented our nodes.
Let us have a look inside the scripts folder.
```bash
$ cd ~/catkin_ws/src/beginner_tutorials/scripts/
$ ls
listener_instrumented.py  listener.py  talker_instrumented.py  talker.py
```
As we can see, now we have two new files: talker_instrumented.py and listener_instrumented.py

These two instrumented files are equal to the previous ones. The only difference is in the substitution of the topics which are published by talker. If we compare talker.py with talker_instrumented.py, we find a small difference.

In talker.py we have:
```python
...
pub = rospy.Publisher('chatter', String, queue_size=10)
...
```
While in talker_instrumented.py we have:
```python
...
pub = rospy.Publisher('chatter_mon', String, queue_size=10)
...
```

Even though this can seem as a worthless modification, it allows us to put a monitor in the middle of the communication.
In fact, the instrumented talker publishes on a different topic now ('chatter_mon'), while the listener (which in this specific case is totally unchanged since it does not publish anything) listens on the old one ('chatter'). If we run the two instrumented nodes as we did before with the normal ones, we would observe that the two nodes are not able to communicate anymore. Because the talker publishes a topic that is not subscribed by the listener. 

Remember: roscore must be running on another terminal..
```bash
$ cd ~/catkin_ws/src/beginner_tutorials/scripts/
$ chmod +x talker_instrumented.py
$ chmod +x listener_instrumented.py
```
In a terminal then
```bash
$ cd ~/catkin_ws/
$ rosrun beginner_tutorials talker_instrumented.py
```
and in a different one
```bash
$ cd ~/catkin_ws/
$ rosrun beginner_tutorials listener_instrumented.py
```
The talker should print the topics on the terminal as before. But, the listener should print nothing.

### Adding the monitor in the middle (Offline version).

In order to re-establish the communication between our nodes, we have to execute the monitor which has been created by the generator program.

On a different terminal:
```bash
$ cd ~/catkin_ws/src/beginner_tutorials/ROSMonitoring/ROSMonitor/
$ chmod +x monitor.py
$ cd ~/catkin_ws/
$ rosrun beginner_tutorials monitor.py
[INFO] [1559652181.670203]: monitor started and ready: Offline
```

The monitor is now ready to intercept the messages.

Let us execute again the instrumented nodes (talker_instrumented.py and listener_instrumented.py) as we did before.
This time they will be able to communicate.

The monitor should print on the terminal something like this:
```bash
[INFO] [1559638171.740409]: /listener_27375_1559638153394I heard hello 
[INFO] [1559638171.840524]: /listener_27375_1559638153394I heard hello 
[INFO] [1559638171.941144]: /listener_27375_1559638153394I heard hello 
[INFO] [1559638172.040488]: /listener_27375_1559638153394I heard hello 
```

Since we have selected Offline RV, the monitor is only logging the events.
We can find the automatically generated log file (log.txt) inside ~/catkin_ws folder.

The log file should look like this:

```json
{"topic": "chatter", "data": "hello", "time": 1559638159.43485}
{"topic": "chatter", "data": "hello", "time": 1559638159.534461}
{"topic": "chatter", "data": "hello", "time": 1559638159.635648}
...
```

The last step for the Offline version is to check the log file against a formal specification.
To do this, first we copy the log file into the prolog folder, and then we run the monitor (using the already given sh file).

```bash
$ cp ~/catkin_ws/log.txt ~/catkin_ws/src/beginner_tutorials/ROSMonitoring/monitor/
$ cd ~/catkin_ws/src/beginner_tutorials/ROSMonitoring/monitor/prolog/
$ sh offline_monitor.sh ../rml/test.pl ../log.txt

```
 
 
 





