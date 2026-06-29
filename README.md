# ROSMonitoring

ROSMonitoring is a runtime-verification framework for ROS applications. It
generates monitor nodes that observe the communication between ROS nodes,
serialize observed events as JSON, optionally ask an external oracle for a
verdict, and either log or filter the observed communication.

This version is intended to become the integrated master version of the
framework. It brings together the feature lines that previously lived across
separate branches:

- ROS1 topic monitoring
- ROS1 service monitoring
- ROS2 topic monitoring
- ROS2 service monitoring
- ordered publication-time traces
- online WebSocket oracle integration
- offline JSONL logs
- legacy ROS verdict topics for application-level reactions
- monitor status reporting and a browser dashboard
- a tested Python generator package

The design remains formalism-agnostic. ROSMonitoring does not require a specific
logic or monitor synthesis tool. Any oracle that speaks the JSON-over-WebSocket
protocol described below can be used.

## Why Runtime Monitoring for ROS?

ROS systems are distributed: publishers, subscribers, service clients, and
service servers can run on different processes and machines. Static verification
alone is often not enough for this execution model. ROSMonitoring adds runtime
checks at the communication layer:

- observe real executions without rewriting application nodes;
- check traces online through an oracle;
- keep offline logs for later analysis;
- filter unsafe messages or service requests when a property is violated;
- inspect monitor health through status logs or a browser dashboard.

The original ROSMonitoring papers describe the framework and the later extension
to services and ordered topics. This repository implements those ideas in a
cleaner generator with regression and ROS2 integration tests.

## Repository Layout

```text
.
├── src/rosmonitoring/        # importable generator/runtime support package
├── oracle/                   # trusted legacy oracle bundle: RML, TL, LamaConv
├── examples/                 # ROS1 and ROS2 YAML configurations
│   ├── README.md             # runnable tutorial catalog
│   ├── ros2_system/          # tiny ROS2 nodes used by examples
│   ├── tutorials/            # feature-focused monitor YAML files
│   └── oracle_runs/          # JSONL traces for offline oracle checks
├── tests/                    # unit, regression, and ROS2 integration tests
├── docs/architecture.md      # architecture notes and event shape
├── docs/test_plan.md         # release-critical coverage matrix
├── docs/implementation_history.md # migration and future-maintainer handoff
├── pyproject.toml            # package metadata and dependencies
└── README.md
```

Generated ROS packages are written to the output directory selected by the user.
For example, ROS2 generation produces an `ament_python` package at
`generated_ros2/monitor`.

For runnable examples beyond the quick-start path, see
[`examples/README.md`](examples/README.md). It includes small ROS2 application
nodes and tutorials for TL/Reelay, RML/Prolog, LamaConv, services, ordering, and
the dashboard.

## Main Concepts

**Instrumentation**

Instrumentation is driven by a YAML configuration. The generator creates monitor
nodes and ROS package files. For invasive monitoring, the application side should
publish or call the remapped interface, such as `/chatter_mon`, while the monitor
republishes or forwards to the original interface, such as `/chatter`.

**Monitor**

A generated monitor is a ROS node. It can observe topics, serve proxy services,
forward accepted events, and write JSONL logs. Monitors can be generated for ROS1
or ROS2.

**Oracle**

An oracle is an optional WebSocket server. The monitor sends one legacy-compatible
flat JSON event per observed message/request/response. The oracle replies with a
verdict. A verdict of false with `action: filter` prevents propagation.

The `oracle/` directory is the trusted legacy ROSMonitoring oracle bundle and is
kept as-is. Generated monitors preserve the old event protocol so existing RML,
TL/Reelay, and LamaConv oracles can still be used.

**Offline Mode**

If a monitor has no `oracle` block, it writes events to its log and does not ask
for verdicts. Status entries for these log-only events use `verdict_raw:
unknown`, meaning no oracle verdict exists; it is not a violation.

**Online Mode**

If a monitor has an `oracle` block, it connects to the configured WebSocket
server and sends each event before deciding whether to propagate it.

**Verdict Topic**

Every generated monitor also publishes the raw oracle verdict on the legacy ROS
topic:

```text
/<monitor_id>/monitor_verdict
```

The message type is `std_msgs/String`. This is separate from local JSONL logs
and the browser dashboard: ROS application nodes can subscribe to this topic and
react online. ROS1 uses a latched publisher, matching the old implementation.
ROS2 uses transient-local QoS so late subscribers can inspect the latest
verdict. Values are normalized strings such as `true`, `false`,
`currently_true`, `currently_false`, and `unknown`.

**Ordered Mode**

If `ordered: true` or `ordering.enabled: true` is set, the monitor buffers events
from all ordered topics and services in that monitor into one source-time
stream. It uses `header.stamp` when available, then `stamp`, then the
observation time. This preserves the publication/source order across multiple
ordered interfaces and also reorders events on one topic when multiple
publishers provide stamped messages.

For passive `log` monitors, buffered events are sent to the log/oracle when the
source-time watermark advances far enough, or when the monitor stops. For
`filter` topics and service requests, the monitor waits for the ordered verdict
for that specific event before forwarding it. The wait is bounded by the
monitor-level ordered window, which is the largest `max_delay_ms` among ordered
interfaces in that monitor.

## Installation

### Python Development Install

```bash
python3 -m pip install -e .[dev]
```

The `dev` extra installs the unit-test and WebSocket packages used by the test
harness. If you only need online monitor clients, this is enough:

```bash
python3 -m pip install -e .[online]
```

### ROS2

Install a ROS2 distribution appropriate for your OS, then build generated
packages with `colcon`. On Ubuntu 22.04, ROS2 Humble is the usual choice.

Every terminal that builds, runs, or tests generated ROS2 monitors should source
ROS2 first:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
colcon --help
```

### ROS1

ROS1 generation creates a Catkin-style package. You need a normal ROS1
environment with `rospy` and the message/service packages named in your YAML.

## Quick Start: ROS2 Topic and Service Monitor

Validate a configuration:

```bash
python3 -m rosmonitoring.cli validate examples/tutorials/ros2_topics_services_ordered.yaml
```

Generate a ROS2 package:

```bash
python3 -m rosmonitoring.cli generate \
  examples/tutorials/ros2_topics_services_ordered.yaml \
  --ros-version ros2 \
  --output generated_ros2
```

Build it in a ROS2 workspace:

```bash
cd generated_ros2
colcon build
. install/setup.bash
ros2 launch monitor monitor.launch.py
```

Run one generated monitor directly:

```bash
ros2 run monitor battery_guard
```

## Reproducible ROS2 Tutorials

The tutorials below use four roles:

- **oracle**: a WebSocket process that receives JSON events and returns verdicts;
- **monitor**: the generated ROSMonitoring node;
- **ROS system**: the application being monitored; in the tutorial this is
  simulated with `ros2 topic pub` or a tiny service server;
- **observer/client**: commands used to inspect what passed through the monitor.

The checked-in `oracle/` directory is the trusted legacy oracle bundle. The
tutorials use `examples/oracles/tutorial_oracle.py` so the legacy oracle files
remain unchanged.

### One-Time Setup

Clone the repository, then run this once from the repository root:

```bash
python3 -m pip install -e .[dev]
```

For every new terminal used in the tutorials, start from a clean ROS2
environment. Set `ROSMONITORING_HOME` to the path where you cloned this
repository:

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME"
```

Do not build a tutorial workspace from a terminal where another generated
workspace, such as `generated_ros2/install`, is already sourced. Otherwise
`colcon` may record that overlay in the new workspace.

The `PYTHONPATH` reset is intentional. Generated tutorial workspaces all contain
a Python package named `monitor`; leaving an older generated workspace in
`PYTHONPATH` can make `ros2 run monitor <executable>` find the right executable
wrapper but load stale Python entry-point metadata.

### Tutorial 1: Topic Filtering

This tutorial monitors a topic named `/chatter`.

Flow:

```text
ROS system publishes /chatter_mon
        |
        v
monitor chatter_guard asks oracle
        |
        v
accepted messages are republished on /chatter
```

The tutorial monitor is configured in
`examples/tutorials/tutorial_topic.yaml`. Because the topic has a `publishers`
entry, the monitor intercepts publisher-side traffic:

- the ROS system must publish to `/chatter_mon`;
- the monitor republishes accepted messages to `/chatter`;
- the oracle rejects messages whose `data` is `drop`.

#### Terminal A: Start the Oracle

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME"

rm -rf tutorial_ws/logs
mkdir -p tutorial_ws/logs
python3 examples/oracles/tutorial_oracle.py --port 8080 --log ./tutorial_ws/logs/oracle.jsonl
```

Leave this terminal running.

#### Terminal B: Generate, Build, and Run the Monitor

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME"

python3 -m rosmonitoring.cli validate examples/tutorials/tutorial_topic.yaml
python3 -m rosmonitoring.cli generate examples/tutorials/tutorial_topic.yaml --ros-version ros2 --output tutorial_ws/src

cd tutorial_ws
rm -rf build install log
colcon build
. install/setup.bash
ros2 pkg executables monitor
ros2 run monitor chatter_guard -- --dashboard
```

Expected executable list:

```text
monitor chatter_guard
```

Leave this terminal running. It now runs both the monitor and the dashboard.
Open the dashboard at:

```text
http://127.0.0.1:8765
```

If port `8765` is already in use, choose another one:

```bash
ros2 run monitor chatter_guard -- --dashboard --dashboard-port 8766
```

#### Terminal C: Observe the Final Topic

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME/tutorial_ws"
. install/setup.bash

ros2 topic echo /chatter std_msgs/msg/String
```

Leave this terminal running. It prints only messages accepted by the monitor.

#### Terminal D: Simulate the Monitored ROS System

Publish an accepted message:

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME/tutorial_ws"
. install/setup.bash

ros2 topic pub --once /chatter_mon std_msgs/msg/String "{data: hello}"
```

Expected result in Terminal C:

```text
data: hello
```

Publish a rejected message:

```bash
ros2 topic pub --once /chatter_mon std_msgs/msg/String "{data: drop}"
```

Expected result: Terminal C does not print `drop`.

#### Inspect Topic Logs

Stop the monitor with `Ctrl+C`, then inspect logs from the repository root:

```bash
cd "$ROSMONITORING_HOME"
cat tutorial_ws/logs/chatter_guard.jsonl
cat tutorial_ws/logs/oracle.jsonl
cat tutorial_ws/logs/status.jsonl
```

Expected behavior:

- `chatter_guard.jsonl` contains both `hello` and `drop`, because both were
  observed;
- `oracle.jsonl` contains verdict `currently_true` for `hello` and
  `currently_false` for `drop`;
- `status.jsonl` records the monitor events and the violation.

### Tutorial 2: Service Filtering

This tutorial monitors a service named `/add_two_ints`.

Flow:

```text
client calls /add_two_ints_mon
        |
        v
monitor add_two_ints_guard asks oracle
        |
        v
accepted requests are forwarded to real /add_two_ints server
```

The tutorial monitor is configured in
`examples/tutorials/tutorial_service.yaml`. The oracle rejects requests where
`a < 0`.

#### Terminal A: Start the Oracle

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME"

rm -rf tutorial_service_ws/logs
mkdir -p tutorial_service_ws/logs
python3 examples/oracles/tutorial_oracle.py --port 8080 --log ./tutorial_service_ws/logs/oracle.jsonl
```

Leave this terminal running.

#### Terminal B: Start the Real ROS Service

This is the ROS system being monitored:

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME"

python3 examples/ros2_system/add_two_ints_server.py
```

Leave this terminal running. It serves `/add_two_ints`.

#### Terminal C: Generate, Build, and Run the Monitor

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME"

python3 -m rosmonitoring.cli validate examples/tutorials/tutorial_service.yaml
python3 -m rosmonitoring.cli generate examples/tutorials/tutorial_service.yaml --ros-version ros2 --output tutorial_service_ws/src

cd tutorial_service_ws
rm -rf build install log
colcon build
. install/setup.bash
ros2 pkg executables monitor
ros2 run monitor add_two_ints_guard -- --dashboard
```

Expected executable list:

```text
monitor add_two_ints_guard
```

Leave this terminal running. It now runs both the monitor and the dashboard.
Open the dashboard at:

```text
http://127.0.0.1:8765
```

#### Terminal D: Call the Monitored Service

Accepted request:

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME/tutorial_service_ws"
. install/setup.bash

ros2 service call /add_two_ints_mon example_interfaces/srv/AddTwoInts "{a: 2, b: 3}"
```

Expected response:

```text
sum=5
```

Rejected request:

```bash
ros2 service call /add_two_ints_mon example_interfaces/srv/AddTwoInts "{a: -1, b: 7}"
```

Expected response is the default service response, typically:

```text
sum=0
```

The real service server in Terminal B should not print a served request for
`-1 + 7`.

#### Inspect Service Logs

Stop the monitor with `Ctrl+C`, then inspect logs from the repository root:

```bash
cd "$ROSMONITORING_HOME"
cat tutorial_service_ws/logs/add_two_ints_guard.jsonl
cat tutorial_service_ws/logs/oracle.jsonl
cat tutorial_service_ws/logs/status.jsonl
```

Expected behavior:

- `add_two_ints_guard.jsonl` contains both service requests;
- `oracle.jsonl` contains verdict `currently_true` for `{a: 2, b: 3}` and
  `currently_false` for `{a: -1, b: 7}`;
- the real service server only handles the accepted request.

### Tutorial 3: Ordered Topic Logging

Ordered monitoring matters when a property depends on publication/source order
rather than receive order. The first example logs one `geometry_msgs/PoseStamped`
topic by `header.stamp`.

Use one terminal for the monitor:

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME"

python3 -m rosmonitoring.cli validate examples/tutorials/ordered_pose_log.yaml
python3 -m rosmonitoring.cli generate examples/tutorials/ordered_pose_log.yaml --ros-version ros2 --output tutorial_ordered_ws/src
cd tutorial_ordered_ws
rm -rf build install log
colcon build
. install/setup.bash
ros2 run monitor ordered_pose_log
```

In a second terminal, publish a later source timestamp first and an earlier
source timestamp second:

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME/tutorial_ordered_ws"
. install/setup.bash

ros2 topic pub --once /ordered_pose geometry_msgs/msg/PoseStamped \
  "{header: {stamp: {sec: 20, nanosec: 0}, frame_id: later}, pose: {orientation: {w: 1.0}}}"

ros2 topic pub --once /ordered_pose geometry_msgs/msg/PoseStamped \
  "{header: {stamp: {sec: 10, nanosec: 0}, frame_id: earlier}, pose: {orientation: {w: 1.0}}}"
```

Stop the monitor with `Ctrl+C`, then inspect:

```bash
cd "$ROSMONITORING_HOME"
cat tutorial_ordered_ws/logs/ordered_pose_log.jsonl
```

The event with `sec: 10` should appear before the event with `sec: 20`.

The same mechanism is global inside one generated monitor. If several topics or
services in the same monitor have `ordering.enabled: true`, the monitor sends one
merged source-time stream to the log/oracle. This example publishes
`/ordered_alpha` with stamp `20`, then `/ordered_beta` with stamp `10`; the log
records `/ordered_beta` first:

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME"

python3 -m rosmonitoring.cli validate examples/tutorials/global_ordered_topics.yaml
python3 -m rosmonitoring.cli generate examples/tutorials/global_ordered_topics.yaml --ros-version ros2 --output tutorial_global_ordered_ws/src
cd tutorial_global_ordered_ws
rm -rf build install log
colcon build
. install/setup.bash
ros2 run monitor global_ordered_pose_log
```

In a second terminal:

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME/tutorial_global_ordered_ws"
. install/setup.bash
python3 ../examples/ros2_system/ordered_multi_publisher.py
```

Stop the monitor and inspect:

```bash
cd "$ROSMONITORING_HOME"
cat tutorial_global_ordered_ws/logs/global_ordered_pose_log.jsonl
```

The first two rows should be `ordered_beta` at time `10.0`, then
`ordered_alpha` at time `20.0`.

### Browser Dashboard

The preferred tutorial command starts the dashboard inside the monitor process:

```bash
ros2 run monitor chatter_guard -- --dashboard
```

Available dashboard flags:

```bash
--dashboard
--dashboard-host 127.0.0.1
--dashboard-port 8765
--fresh-session
--session-id demo-1
```

You can also enable it without editing the command:

```bash
ROSMONITORING_DASHBOARD=1 ros2 run monitor chatter_guard
```

For a generated package with multiple monitors, launch all monitors and enable
the dashboard on one of them:

```bash
ros2 launch monitor monitor.launch.py dashboard:=true fresh_session:=true
```

The default `dashboard_monitor:=first` starts HTTP in the first generated
monitor. To choose a specific monitor:

```bash
ros2 launch monitor monitor.launch.py dashboard:=true dashboard_monitor:=my_monitor_id
```

The dashboard reads the shared status log, so the **All monitors** view shows
events from every launched monitor. Use the monitor selector to inspect one
monitor at a time.

Use a fresh session when you do not want the dashboard to show events from a
previous run:

```bash
export ROSMONITORING_FRESH_SESSION=1
export ROSMONITORING_SESSION_ID="$(date +%Y%m%d-%H%M%S)"
ros2 run monitor chatter_guard -- --dashboard
```

`ROSMONITORING_FRESH_SESSION=1` clears the generated monitor's event log and
clears the shared status log once for the exported `ROSMONITORING_SESSION_ID`.
This is useful when several generated monitors are launched together.

The separate dashboard command remains available when you want to inspect an
existing status log without starting a monitor:

```bash
python3 -m rosmonitoring.cli status --status-log tutorial_ws/logs/status.jsonl
```

The dashboard is interactive and refreshes automatically. It includes:

- overview cards for active monitors, observed events, and violations;
- an all-monitors view plus a selector for one monitor at a time;
- statistics for selected events, verdicts, interfaces, and event rates;
- browser-side plots for events by monitor, events by interface, verdict mix,
  and event timing;
- a sequence diagram tab that updates with the dashboard data. With **All
  monitors** selected it shows the observed sequence across all launched
  monitors; selecting one monitor focuses the diagram on only that monitor's
  observed events;
- configured interface details, including remapped names, action, type, and
  ordering;
- recent observed event payloads from the monitor JSONL log;
- recent status timeline entries, including oracle errors and verdicts;
- raw JSON for debugging and copying into tests;
- text filtering across interfaces, verdicts, payloads, and status entries.

The generated monitor also exposes JSON endpoints:

```text
/api/status
/api/dashboard
```

## Quick Start: ROS1 Monitor

Generate a ROS1 Catkin package:

```bash
python3 -m rosmonitoring.cli generate \
  examples/tutorials/ros1_legacy_compatible.yaml \
  --ros-version ros1 \
  --output generated_ros1
```

Build it in a Catkin workspace using your normal ROS1 workflow, then run the
generated launch file:

```bash
roslaunch monitor run_monitor_0_ros1.launch
```

## YAML Configuration

Minimal offline topic monitor:

```yaml
ros_version: ros2
monitors:
  - monitor:
      id: offline_text_log
      log: ./logs/offline_text_log.jsonl
      topics:
        - name: chatter
          type: std_msgs.msg.String
          action: log
```

Online filtering topic monitor:

```yaml
monitors:
  - monitor:
      id: chatter_guard
      log: ./logs/chatter_guard.jsonl
      oracle:
        url: 127.0.0.1
        port: 8080
        action: nothing
      topics:
        - name: chatter
          type: std_msgs.msg.String
          action: filter
          publishers:
            - talker
```

The monitor subscribes to `/chatter_mon` and republishes accepted messages to
`/chatter`. The application publisher must publish to the remapped topic.

Service monitor:

```yaml
monitors:
  - monitor:
      id: add_two_ints_guard
      log: ./logs/add_two_ints_guard.jsonl
      oracle:
        url: 127.0.0.1
        port: 8080
        action: nothing
      services:
        - name: add_two_ints
          type: example_interfaces.srv.AddTwoInts
          action: filter
```

The monitor serves `/add_two_ints_mon` and forwards accepted requests to
`/add_two_ints`.

Ordered monitor:

```yaml
monitors:
  - monitor:
      id: ordered_pose_guard
      log: ./logs/ordered_pose_guard.jsonl
      topics:
        - name: pose_a
          type: geometry_msgs.msg.PoseStamped
          action: log
          ordering:
            enabled: true
            source: header_stamp
            max_delay_ms: 100
        - name: pose_b
          type: geometry_msgs.msg.PoseStamped
          action: log
          ordering:
            enabled: true
            source: header_stamp
            max_delay_ms: 100
```

Legacy syntax remains supported:

```yaml
ordered: true
```

## Configuration Reference

Top-level fields:

- `ros_version`: optional; `ros1` or `ros2`.
- `path`: optional legacy workspace path field.
- `nodes`: optional legacy node metadata used by older instrumentation flows.
- `monitors`: required list of monitor definitions.

Monitor fields:

- `id`: required Python identifier. Also used as the generated executable name.
- `log`: required JSONL event log path.
- `silent`: optional boolean; default `false`.
- `warning`: optional integer; default `1`.
- `oracle`: optional; absence means offline mode.
- `status`: optional status JSONL settings.
- `topics`: optional list of topic interfaces.
- `services`: optional list of service interfaces.

Topic/service fields:

- `name`: required ROS interface name, with or without a leading `/`.
- `type`: required Python import path such as `std_msgs.msg.String`.
- `action`: `log` or `filter`.
- `publishers`: optional list; marks an intercepted publisher-side topic.
- `subscribers`: optional list; marks an intercepted subscriber-side topic.
- `ordered`: optional legacy boolean.
- `ordering`: optional structured ordering settings.

Ordering fields:

- `enabled`: boolean.
- `source`: currently documented as `header_stamp`; the implementation falls
  back through `header.stamp`, `stamp`, then observation time.
- `max_delay_ms`: reorder window in milliseconds. Ordered interfaces in the
  same monitor share one global buffer; the monitor uses the largest configured
  `max_delay_ms` across those interfaces.

Status fields:

- `enabled`: boolean; default `true`.
- `log`: JSONL status log path; default `rosmonitoring-status.jsonl`.

## Event Format

Generated monitors write JSON Lines using the legacy ROSMonitoring event shape.
The same shape is sent to the oracle over WebSocket.

Topic event:

```json
{
  "topic": "chatter",
  "time": 1782458598.685226,
  "data": "hello"
}
```

Service request event:

```json
{
  "service": "add_two_ints",
  "time": 1782458599.132891,
  "request": {"a": 2, "b": 3}
}
```

Service response event:

```json
{
  "service": "add_two_ints",
  "time": 1782458599.144110,
  "response": {"sum": 5}
}
```

For stamped messages, `time` is the source timestamp used by ordered monitoring.
Otherwise it is the monitor observation time. Generated monitors keep additional
monitor metadata internally for status reporting, but it is not sent to legacy
oracles.

## Oracle Protocol

The oracle is a WebSocket server. The monitor sends one JSON event as a text
message. The oracle replies with either:

```json
{"verdict": true}
```

or:

```json
{"verdict": false}
```

The monitor also accepts textual verdicts such as `false`, `violation`, and
`violated` as negative verdicts. Legacy oracle verdicts such as
`currently_true`, `currently_false`, and `unknown` are also accepted; only
`false`, `currently_false`, `violation`, and `violated` are treated as negative.
The raw oracle verdict is preserved in status entries as `verdict_raw`, so
`false` and `currently_false` remain distinguishable in logs and the dashboard.
For filtering, both are treated as negative and block propagation. For warning
reporting, the legacy `warning` level is preserved: `warning: 1` reports both
`currently_false` and `false`, while `warning: 2` reports only final-style
negative verdicts such as `false`.

The legacy terminal-verdict behavior is also preserved. Verdicts `true` and
`false` are definitive. A passive monitor that only observes/logs can stop after
one of these verdicts to save resources. A monitor that sits in the communication
path, for example a remapped filtering topic or a proxy service, records the
terminal verdict but keeps running; stopping it would break the ROS application
path it is protecting.

Example oracle:

```python
import asyncio
import json
import websockets


async def handler(websocket):
    async for message in websocket:
        event = json.loads(message)
    verdict = "currently_true"
    if event.get("topic") == "chatter" and event.get("data") == "drop":
        verdict = "currently_false"
    await websocket.send(json.dumps({"verdict": verdict}))


async def main():
    async with websockets.serve(handler, "127.0.0.1", 8080):
        await asyncio.Future()


asyncio.run(main())
```

## Propagation Semantics

For topics:

- non-intercepting monitor: subscribe to the original topic and log/check;
- intercepted publisher-side topic: subscribe to `<topic>_mon`, publish accepted
  messages to `<topic>`;
- intercepted subscriber-side topic: subscribe to `<topic>`, publish accepted
  messages to `<topic>_mon`.

For services:

- serve `<service>_mon`;
- send request events to the oracle/log;
- if accepted, call the original `<service>`;
- send response events to the oracle/log;
- return the original response to the client.

If an online oracle is unavailable at startup, the generated monitor logs a
warning and runs in log-only mode. Install `websocket-client` for online use.

## Runtime Verdict Topic

Generated monitors preserve the old ROSMonitoring verdict publication contract.
After each handled event, the monitor publishes the raw normalized verdict to:

```text
/<monitor_id>/monitor_verdict
```

The topic type is `std_msgs/String`. For example:

```bash
ros2 topic echo /chatter_guard/monitor_verdict std_msgs/msg/String
```

For ROS1:

```bash
rostopic echo /chatter_guard/monitor_verdict
```

This topic is the ROS-facing integration point for application nodes that need
to react to monitor verdicts. JSONL event logs and status logs are still written
for diagnostics and dashboard inspection, but they are not the only output of a
monitor. ROS1 verdict publishers are latched. ROS2 verdict publishers use
transient-local QoS with depth `1000`.

## Status Dashboard

Generated monitors can append status events to a JSONL file. Serve a browser
view with:

```bash
python3 -m rosmonitoring.cli status --status-log ./logs/rosmonitoring-status.jsonl
```

Open:

```text
http://127.0.0.1:8765
```

The dashboard reports monitor status, event counts, violation counts, observed
interfaces, per-monitor statistics, plots, recent payloads, verdict timelines,
and a live sequence diagram.

## Testing and Experiments

For the full release-critical coverage matrix and the recommended hardening
workflow, see [`docs/test_plan.md`](docs/test_plan.md).

Fast no-ROS regression tests:

```bash
python3 -m pytest -q
```

ROS2 integration experiment with system ROS2:

```bash
source /opt/ros/humble/setup.bash
unset GTK_PATH GIO_EXTRA_MODULES
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src:$PYTHONPATH \
  python3 -m pytest -q tests/test_ros2_integration.py -s
```

`PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` avoids unrelated ROS Humble pytest plugin
compatibility problems with newer pytest versions.
`PYTHONPATH=src:$PYTHONPATH` prepends this checkout while preserving the ROS2
Python paths added by `/opt/ros/humble/setup.bash`.

The ROS2 integration experiment generates a fresh workspace, builds the monitor
package with `colcon`, starts real ROS2 processes, and checks:

- online WebSocket connection from monitor to oracle;
- positive topic verdict propagates `/guarded_text_mon` to `/guarded_text`;
- negative topic verdict blocks propagation;
- offline topic monitor writes JSONL events;
- service monitor serves `/add_two_ints_mon`;
- accepted service request is forwarded to `/add_two_ints`;
- rejected service request is not forwarded;
- service responses are returned to the ROS2 caller;
- oracle receives topic and service events;
- status JSONL records positive and negative verdicts;
- ordered `PoseStamped` events are flushed in source-time order on one topic
  and across two topics in the same monitor.

Current verified results on this checkout:

```text
unsourced unit/regression tests: 59 passed, 1 skipped
full suite with /opt/ros/humble/setup.bash sourced: 60 passed
standalone ROS2 integration: 1 passed
ROS1 Docker smoke: 1 passed
all tutorial YAML files validated
10 ROS2 tutorial workspaces generated and built
ROS1 tutorial package generated
dashboard JavaScript syntax check passed
```

## Known Engineering Notes

- Generated ROS2 service monitors use `ReentrantCallbackGroup` and
  `MultiThreadedExecutor` so forwarding a service request does not deadlock.
- Ordered monitors use one global source-time buffer per generated monitor. This
  preserves ordering across ordered topics and services, and also reorders
  stamped events on one topic.
- Ordered filters and service requests wait for their own ordered oracle verdict
  before forwarding. The wait is bounded by the monitor-level ordered window.
- Generated ROS2 packages declare `ament_python` and message/service package
  dependencies inferred from the YAML interface types.
- The generator and most tests do not require ROS to be installed.
- Real ROS1 runtime testing is available through `tests/ros1_docker_smoke.sh`
  when Docker can run `ros:noetic-ros-base`.

## Migration From Older Branches

This version accepts the legacy YAML shape used by previous branches:

- `monitors: [{monitor: ...}]`
- `nodes: [{node: ...}]`
- `topics`
- `services`
- topic `publishers` and `subscribers`
- legacy `ordered: true`
- `oracle.url`, `oracle.port`, and `oracle.action`

The recommended new additions are:

- top-level `ros_version`;
- structured `ordering`;
- structured `status`;
- explicit `--output` directory for generation.

## Citing ROSMonitoring

If you use ROSMonitoring in academic work, please cite the original framework
paper:

```bibtex
@inproceedings{DBLP:conf/taros/FerrandoC0AFM20,
  author       = {Angelo Ferrando and
                  Rafael C. Cardoso and
                  Michael Fisher and
                  Davide Ancona and
                  Luca Franceschini and
                  Viviana Mascardi},
  editor       = {Abdelkhalick Mohammad and
                  Xin Dong and
                  Matteo Russo},
  title        = {ROSMonitoring: {A} Runtime Verification Framework for {ROS}},
  booktitle    = {Towards Autonomous Robotic Systems - 21st Annual Conference, {TAROS}
                  2020, Nottingham, UK, September 16, 2020, Proceedings},
  series       = {Lecture Notes in Computer Science},
  volume       = {12228},
  pages        = {387--399},
  publisher    = {Springer},
  year         = {2020},
  url          = {https://doi.org/10.1007/978-3-030-63486-5\_40},
  doi          = {10.1007/978-3-030-63486-5\_40},
  timestamp    = {Wed, 08 Jan 2025 15:39:09 +0100},
  biburl       = {https://dblp.org/rec/conf/taros/FerrandoC0AFM20.bib},
  bibsource    = {dblp computer science bibliography, https://dblp.org}
}
```

If you use the service-monitoring or ordered-topic features, please also cite
the extension paper:

```bibtex
@inproceedings{DBLP:journals/corr/abs-2411-14367,
  author       = {Maryam Ghaffari Saadat and
                  Angelo Ferrando and
                  Louise A. Dennis and
                  Michael Fisher},
  editor       = {Matt Luckcuck and
                  Mengwei Xu},
  title        = {ROSMonitoring 2.0: Extending {ROS} Runtime Verification to Services
                  and Ordered Topics},
  booktitle    = {Proceedings Sixth International Workshop on Formal Methods for Autonomous
                  Systems, FMAS@iFM 2024, Manchester, UK, 11th and 12th of November
                  2024},
  series       = {{EPTCS}},
  volume       = {411},
  pages        = {38--55},
  year         = {2024},
  url          = {https://doi.org/10.4204/EPTCS.411.3},
  doi          = {10.4204/EPTCS.411.3},
  timestamp    = {Sun, 21 Jun 2026 17:12:58 +0200},
  biburl       = {https://dblp.org/rec/journals/corr/abs-2411-14367.bib},
  bibsource    = {dblp computer science bibliography, https://dblp.org}
}
```

## Development Checklist

Before merging changes:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider

source /opt/ros/humble/setup.bash
unset GTK_PATH GIO_EXTRA_MODULES
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src:$PYTHONPATH \
  python3 -m pytest -q tests/test_ros2_integration.py -s

tests/ros1_docker_smoke.sh

for yaml in examples/tutorials/*.yaml; do
  if [ "$(basename "$yaml")" = "ros1_legacy_compatible.yaml" ]; then
    python3 -m rosmonitoring.cli validate "$yaml" --ros-version ros1
  else
    python3 -m rosmonitoring.cli validate "$yaml" --ros-version ros2
  fi
done
```

To rebuild every tutorial package in isolated temporary workspaces:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/src:${PYTHONPATH:-}"
base=/tmp/rosmonitoring_tutorial_builds
rm -rf "$base"
mkdir -p "$base"
for yaml in examples/tutorials/*.yaml; do
  name="$(basename "$yaml" .yaml)"
  if [ "$name" = "ros1_legacy_compatible" ]; then
    python3 -m rosmonitoring.cli generate "$yaml" --ros-version ros1 --output "$base/$name/src"
  else
    python3 -m rosmonitoring.cli generate "$yaml" --ros-version ros2 --output "$base/$name/src"
    (cd "$base/$name" && colcon build)
  fi
done
```

For generated ROS2 packages, also verify the generated package itself:

```bash
colcon build
. install/setup.bash
ros2 pkg list | grep '^monitor$'
```

When changing dashboard code, check the embedded JavaScript:

```bash
PYTHONPATH=src python3 - <<'PY'
from pathlib import Path
from rosmonitoring.status import HTML
Path('/tmp/rosmonitoring_dashboard.js').write_text(
    HTML.split('<script>', 1)[1].split('</script>', 1)[0],
    encoding='utf-8',
)
PY
node --check /tmp/rosmonitoring_dashboard.js
```
