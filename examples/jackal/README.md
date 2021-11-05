# Installation

Other than all ROSMonitoring's dependencies, you need to download the jackal example.
In a terminal:
``bash
-$ sudo apt-get install -y ros-<your-ROS-distribution>-desktop-full
-$ sudo apt-get install -y ros-<your-ROS-distribution>-jackal-simulator
-$ sudo apt-get install -y ros-<your-ROS-distribution>-jackal-desktop
-$ sudo apt-get install -y ros-<your-ROS-distribution>-jackal-navigation
``
This has been tested on ROS Melodic.

# How to run

Copy `oracle/jackal.py` into `ROSMonitoring/oracle/TLOracle/`.

Then, `cd ROSMonitoring/oracle/TLOracle/`, and run `python3 oracle.py --online --discrete --port 8080 --property jackal`

On another terminal, copy `generator/jackal.yaml` into `ROSMonitoring/generator/`.

Then, `cd ROSMonitoring/generator/`, and run `./generator --config_file jackal.yaml`.

This will generate the corresponding monitor inside `ROSMonitoring/monitor/src/`.

Copy the entire `ROSMonitoring/monitor/` folder into a catkin workspace. Compile it and make sure the monitor script is executable (`chmod +x ...`), and that the `devel/setup.bash` file is sourced.

Now, run the monitor `roslaunch monitor run.launch`.

Open another terminal, and run `roslaunch jackal_gazebo jackal_world.launch config:=front_laser`.

Optional: If you want to see the monitor's verdict at runtime, open another terminal and run `rostopic echo /safety_distance_monitor/monitor_verdict`.
