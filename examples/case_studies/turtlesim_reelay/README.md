# Case Study: Turtlesim + TL/Reelay Oracle

This case study exercises ROSMonitoring on a standard ROS2 graphical system
without requiring a Gazebo world or robot-specific packages. It uses
`turtlesim`, the trusted TL/Reelay oracle, a generated ROS2 monitor, the browser
dashboard, topic filtering, passive topic logging, ordered stamped messages,
service filtering, and the legacy ROS verdict topic.

The case study is intentionally a lightweight analogue of a Gazebo/Jackal
workflow: it has a visible robot-like process, command topics, state topics, and
service calls, but it can be reproduced on a standard ROS2 desktop setup.

## What Is Monitored

The monitor configuration is [`monitor.yaml`](monitor.yaml).

It creates one monitor, `turtlesim_safety_monitor`, with:

- `/turtle1/cmd_vel_mon -> /turtle1/cmd_vel`: filtered velocity command topic;
- `/turtle1/pose`: passive pose logging from turtlesim;
- `/turtle1/pose_stamped`: ordered passive logging from a timestamp bridge;
- `/turtle1/teleport_absolute_mon -> /turtle1/teleport_absolute`: filtered
  service proxy.

The real TL/Reelay oracle runs with
[`turtlesim_property.py`](turtlesim_property.py). The property is:

```text
{safe}
```

The abstraction marks filtering-relevant events unsafe if:

- a velocity command has `linear.x > 2.0` or `abs(angular.z) > 2.8`;
- a teleport request targets a point outside that safety window.

Pose and stamped-pose topics are logged for context and ordering, but they do
not drive the filtering verdict. This keeps the velocity and service examples
independent even if the turtle is already near a window edge.

This property checks the current filtering-relevant event. An unsafe command or
service request gets a negative verdict, while later safe requests can pass
again.

## Dependencies

Use a terminal with ROS2 sourced. For Humble:

```bash
source /opt/ros/humble/setup.bash
```

Install ROS and Python dependencies if needed:

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-turtlesim
python3 -m pip install -e '.[dev]'
python3 -m pip install websocket-server reelay
```

From the repository root:

```bash
export ROSMONITORING_HOME="$(pwd)"
```

## Terminal A: Start The TL/Reelay Oracle

```bash
cd "$ROSMONITORING_HOME/oracle/TLOracle"
PYTHONPATH="$ROSMONITORING_HOME/examples/case_studies/turtlesim_reelay:$PYTHONPATH" \
  python3 oracle.py --online --discrete --property turtlesim_property --port 8080
```

Leave this running.

You can also smoke-test the property without ROS:

```bash
cd "$ROSMONITORING_HOME/oracle/TLOracle"
PYTHONPATH="$ROSMONITORING_HOME/examples/case_studies/turtlesim_reelay:$PYTHONPATH" \
  python3 oracle.py --offline --discrete --property turtlesim_property \
  --trace "$ROSMONITORING_HOME/examples/case_studies/turtlesim_reelay/unsafe_trace.jsonl"
```

This trace contains one unsafe velocity command and should report a violation
for that event. Later safe events can recover to positive verdicts.

## Terminal B: Generate And Run The Monitor

```bash
export ROSMONITORING_HOME="${ROSMONITORING_HOME:-$(pwd)}"
cd "$ROSMONITORING_HOME"
test -d src/rosmonitoring || { echo "Run this block from the ROSMonitoring repository root or set ROSMONITORING_HOME first."; exit 1; }

unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
export PYTHONPATH="$ROSMONITORING_HOME/src:${PYTHONPATH:-}"

python3 -m rosmonitoring.cli validate examples/case_studies/turtlesim_reelay/monitor.yaml --ros-version ros2
python3 -m rosmonitoring.cli generate examples/case_studies/turtlesim_reelay/monitor.yaml --ros-version ros2 --output turtlesim_case_ws/src

cd turtlesim_case_ws
rm -rf build install log
colcon build
. install/setup.bash

export ROSMONITORING_FRESH_SESSION=1
export ROSMONITORING_SESSION_ID="turtlesim-case-$(date +%Y%m%d-%H%M%S)"
ros2 run monitor turtlesim_safety_monitor -- --dashboard
```

Open:

```text
http://127.0.0.1:8765
```

## Terminal C: Start Turtlesim

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

You should see the turtlesim window.

If this command is launched from a Snap-packaged IDE terminal and fails with a
Snap library or GTK/GIO error such as `undefined symbol: __libc_pthread_init`,
use a normal system terminal or start turtlesim with the Snap variables removed:

```bash
source /opt/ros/humble/setup.bash
env \
  -u SNAP \
  -u SNAP_NAME \
  -u SNAP_REVISION \
  -u SNAP_ARCH \
  -u SNAP_INSTANCE_NAME \
  -u SNAP_INSTANCE_KEY \
  -u SNAP_COOKIE \
  -u SNAP_USER_DATA \
  -u SNAP_USER_COMMON \
  -u GTK_PATH \
  -u GTK_EXE_PREFIX \
  -u GTK_IM_MODULE_FILE \
  -u GIO_MODULE_DIR \
  XDG_DATA_DIRS=/usr/local/share:/usr/share \
  ros2 run turtlesim turtlesim_node
```

The case study itself does not require Snap; it only needs the standard ROS2
turtlesim package.

## Terminal D: Start The Ordered Pose Bridge

The native turtlesim pose message is not stamped. This bridge republishes it as
`PoseStamped` so the ordered-monitor path uses real source timestamps.

```bash
cd "$ROSMONITORING_HOME"
source /opt/ros/humble/setup.bash
python3 examples/case_studies/turtlesim_reelay/pose_stamped_bridge.py
```

## Terminal E: Observe Application Outputs

```bash
cd "$ROSMONITORING_HOME/turtlesim_case_ws"
. install/setup.bash
ros2 topic echo /turtle1/cmd_vel geometry_msgs/msg/Twist
```

In another shell you can also inspect the application-facing verdict topic:

```bash
cd "$ROSMONITORING_HOME/turtlesim_case_ws"
. install/setup.bash
ros2 topic echo /turtlesim_safety_monitor/monitor_verdict std_msgs/msg/String
```

## Terminal F: Drive Safe And Unsafe Traces

Safe velocity trace:

```bash
cd "$ROSMONITORING_HOME"
source /opt/ros/humble/setup.bash
python3 examples/case_studies/turtlesim_reelay/cmd_vel_driver.py --trace safe
```

The command topic should be forwarded from `/turtle1/cmd_vel_mon` to
`/turtle1/cmd_vel`, the turtle should move, and the dashboard should show
positive verdicts.

Unsafe velocity trace:

```bash
python3 examples/case_studies/turtlesim_reelay/cmd_vel_driver.py --trace unsafe_velocity
```

The high-speed command should produce a negative verdict. Since `/turtle1/cmd_vel`
is configured as `action: filter`, the unsafe command is not forwarded to the
real turtlesim command topic.

## Service Filtering: Teleport Requests

Allowed teleport through the monitor:

```bash
cd "$ROSMONITORING_HOME"
source /opt/ros/humble/setup.bash
python3 examples/case_studies/turtlesim_reelay/teleport_client.py 5.5 5.5 0.0
```

Rejected teleport through the monitor:

```bash
python3 examples/case_studies/turtlesim_reelay/teleport_client.py 11.0 11.0 0.0
```

The second request targets a point outside the safety window. The oracle returns
a negative verdict, so the monitor does not call the original
`/turtle1/teleport_absolute` service.

## Inspect Logs

```bash
cd "$ROSMONITORING_HOME/turtlesim_case_ws"
cat logs/turtlesim_safety_monitor.jsonl
cat logs/status.jsonl
```

Expected observations:

- `cmd_vel` events include flattened `linear` and `angular` payloads;
- `pose` events show passive turtlesim state and do not block communication;
- `pose_stamped` events are ordered by source timestamp and do not block
  communication;
- `teleport_absolute` service request events appear before accepted response
  events;
- rejected service requests have no corresponding original-service response;
- `/turtlesim_safety_monitor/monitor_verdict` publishes raw verdict strings.

In the latest full ROS2 run, the unsafe velocity command with `linear.x = 3.2`
appeared in the monitor logs and received `currently_false`, but it did not
appear on `/turtle1/cmd_vel`. The unsafe teleport to `x = 11.0` was also logged
with a negative verdict and the turtle remained at the previous accepted safe
pose.
