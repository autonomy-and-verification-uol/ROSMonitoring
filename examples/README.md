# ROSMonitoring Examples

This folder contains small ROS systems, monitor configurations, oracle traces,
and terminal-by-terminal tutorials. The goal is to make each ROSMonitoring
feature concrete:

- topic filtering;
- service filtering;
- passive logging;
- source-time ordering on one topic and across multiple topics;
- browser dashboard inspection;
- trusted legacy oracle integration.
- a larger turtlesim + TL/Reelay case study.

All commands below assume the repository root is stored in
`ROSMONITORING_HOME`.

```bash
export ROSMONITORING_HOME=/path/to/ROSMonitoring
cd "$ROSMONITORING_HOME"
python3 -m pip install -e .[dev]
```

If you are already somewhere inside this repository, including inside a
generated workspace such as `examples_ws`, you can recover the variable with:

```bash
export ROSMONITORING_HOME="$(
python3 - <<'PY'
from pathlib import Path
for candidate in [Path.cwd(), *Path.cwd().parents]:
    if (candidate / "src" / "rosmonitoring").is_dir() and (candidate / "examples" / "tutorials").is_dir():
        print(candidate)
        raise SystemExit(0)
raise SystemExit("Could not find ROSMonitoring root. Set ROSMONITORING_HOME manually.")
PY
)"
cd "$ROSMONITORING_HOME"
```

Check it before running tutorial commands:

```bash
test -f "$ROSMONITORING_HOME/examples/tutorials/topic_filter_rml.yaml" && echo "$ROSMONITORING_HOME"
```

For ROS2 terminals:

```bash
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
cd "$ROSMONITORING_HOME"
```

Change `/opt/ros/humble/setup.bash` if you use another ROS2 distribution.
Run that reset before starting a new generated workspace. If an old workspace is
still sourced, `colcon` can record a stale overlay and `ros2 run` may report
`No executable found`.

For a fresh dashboard/log view on each new run, export a session id before
starting generated monitors:

```bash
export ROSMONITORING_FRESH_SESSION=1
export ROSMONITORING_SESSION_ID="$(date +%Y%m%d-%H%M%S)"
```

`ROSMONITORING_FRESH_SESSION=1` clears the generated monitor JSONL logs at
startup. `ROSMONITORING_SESSION_ID` lets multiple monitors launched together
share the same fresh session without repeatedly clearing the shared status log.
You can also pass the same behavior directly:

```bash
ros2 run monitor tl_chatter_guard -- --dashboard --fresh-session --session-id demo-1
```

## Folder Layout

```text
examples/
├── ros2_system/        # tiny ROS2 app nodes used by the tutorials
├── oracles/            # small tutorial-only WebSocket oracles
├── oracle_runs/        # JSONL traces for offline oracle experiments
├── case_studies/       # larger reproducible ROS2 case studies
└── tutorials/          # monitor YAML files for individual features/oracles
```

The trusted legacy oracles live outside this folder in `oracle/` and are run
as-is. Files in `examples/oracles/` are only small tutorial helpers.

## Verify The Tutorial Catalog

From the repository root, validate every tutorial YAML before trying the
terminal-by-terminal examples:

```bash
cd "$ROSMONITORING_HOME"
for yaml in examples/tutorials/*.yaml; do
  if [ "$(basename "$yaml")" = "ros1_legacy_compatible.yaml" ]; then
    python3 -m rosmonitoring.cli validate "$yaml" --ros-version ros1
  else
    python3 -m rosmonitoring.cli validate "$yaml" --ros-version ros2
  fi
done
```

For ROS2 tutorials, a stronger check is to generate each workspace and run
`colcon build`. The project test suite performs this style of check on temporary
workspaces during release verification.

## Larger Case Study: Turtlesim + TL/Reelay

The folder
[`examples/case_studies/turtlesim_reelay`](case_studies/turtlesim_reelay)
contains a reproducible ROS2 case study using the standard `turtlesim`
application and the trusted TL/Reelay oracle.

It demonstrates, in one scenario:

- topic filtering for `/turtle1/cmd_vel`;
- passive pose logging for `/turtle1/pose`;
- ordered stamped pose logging through a small bridge node;
- service filtering for `/turtle1/teleport_absolute`;
- browser dashboard inspection;
- application-facing verdicts on
  `/turtlesim_safety_monitor/monitor_verdict`.

Start with:

```bash
cd "$ROSMONITORING_HOME"
sed -n '1,240p' examples/case_studies/turtlesim_reelay/README.md
```

This is the recommended end-to-end manual case study before moving to heavier
Gazebo/Jackal/TurtleBot simulations.

## Example 1: TL/Reelay Oracle + Topic Filtering

This example uses the trusted `oracle/TLOracle/oracle.py` with
`oracle/TLOracle/chatter_property.py`. The property expects `chatter` messages
whose `data` is `hello`.

The actual TL property is:

```text
historically[0:3]{hello}
```

This is stronger than "reject only the current `drop` message". Once a `drop`
is observed, the current 3-time-unit history is not all `hello`, so the oracle
can keep returning `currently_false` for later `hello` messages until the
history window is clean again. With a repeating sequence such as
`hello hello drop hello`, drops happen often enough that the dashboard will show
many violations after the first drop. That is expected.

Extra dependencies:

```bash
python3 -m pip install websocket-server reelay
```

Terminal A, start the real TL oracle:

```bash
cd "$ROSMONITORING_HOME/oracle/TLOracle"
python3 oracle.py --online --discrete --property chatter_property --port 8080
```

Terminal B, generate and run the monitor:

```bash
cd "$ROSMONITORING_HOME"
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
python3 -m rosmonitoring.cli validate "$ROSMONITORING_HOME/examples/tutorials/topic_filter_tl.yaml"
python3 -m rosmonitoring.cli generate "$ROSMONITORING_HOME/examples/tutorials/topic_filter_tl.yaml" --ros-version ros2 --output "$ROSMONITORING_HOME/examples_ws/src"

cd "$ROSMONITORING_HOME/examples_ws"
rm -rf build install log
colcon build
. install/setup.bash
ros2 run monitor tl_chatter_guard -- --dashboard
```

Terminal C, observe the final application topic:

```bash
cd "$ROSMONITORING_HOME/examples_ws"
. install/setup.bash
python3 ../examples/ros2_system/chatter_listener.py --topic /chatter
```

Terminal D, choose one finite trace to publish.

Clean trace:

```bash
cd "$ROSMONITORING_HOME/examples_ws"
. install/setup.bash
python3 ../examples/ros2_system/chatter_talker.py --topic /chatter_mon --once --messages hello hello hello
```

The dashboard should show `currently_true` verdicts and the listener should
receive all three messages.

Violating trace:

```bash
python3 ../examples/ros2_system/chatter_talker.py --topic /chatter_mon --once --messages hello hello drop hello
```

The `drop` message makes the property `currently_false`. Because the monitor is
configured with `action: filter`, messages with a negative current verdict are
not propagated to `/chatter`.

If you want to compare the clean and violating traces independently, restart
Terminal A and Terminal B between the two runs. The TL oracle is stateful, so
running both traces in the same oracle session means the second trace starts
from the history produced by the first.

Open:

```text
http://127.0.0.1:8765
```

Inspect logs:

```bash
cd "$ROSMONITORING_HOME/examples_ws"
cat logs/tl_chatter_guard.jsonl
cat logs/status.jsonl
```

## Example 2: RML/Prolog Oracle + Topic Filtering

This example uses the trusted RML oracle:

- runner: `oracle/RMLOracle/prolog/online_monitor.sh`;
- specification: `oracle/RMLOracle/rml/test.pl`;
- monitor config: `examples/tutorials/topic_filter_rml.yaml`.

Extra dependency: SWI-Prolog with HTTP/WebSocket libraries.

Terminal A, start the real RML oracle:

```bash
cd "$ROSMONITORING_HOME"
./oracle/RMLOracle/prolog/online_monitor.sh ./oracle/RMLOracle/rml/test.pl 8080
```

Terminal B, generate and run the monitor:

```bash
cd "$ROSMONITORING_HOME"
test -f "$ROSMONITORING_HOME/examples/tutorials/topic_filter_rml.yaml" || { echo "Set ROSMONITORING_HOME to the ROSMonitoring repository root"; exit 1; }
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
python3 -m rosmonitoring.cli validate "$ROSMONITORING_HOME/examples/tutorials/topic_filter_rml.yaml"
python3 -m rosmonitoring.cli generate "$ROSMONITORING_HOME/examples/tutorials/topic_filter_rml.yaml" --ros-version ros2 --output "$ROSMONITORING_HOME/examples_rml_ws/src"

cd "$ROSMONITORING_HOME/examples_rml_ws"
rm -rf build install log
colcon build
. install/setup.bash
ros2 run monitor rml_chatter_guard -- --dashboard
```

Terminal C:

```bash
cd "$ROSMONITORING_HOME/examples_rml_ws"
. install/setup.bash
python3 ../examples/ros2_system/chatter_listener.py --topic /chatter
```

Terminal D:

```bash
cd "$ROSMONITORING_HOME/examples_rml_ws"
. install/setup.bash
python3 ../examples/ros2_system/chatter_talker.py --topic /chatter_mon --messages hello hello drop hello
```

The RML oracle can return definitive `false`. The generated monitor records the
terminal verdict but keeps running because it is in the communication path.

## Example 3: LamaConv Oracle + Topic Filtering

This example uses the trusted `oracle/LamaConvOracle/oracle.py` and
`oracle/LamaConvOracle/property.py`.

The bundled LamaConv property is:

```text
G hello
```

This is a future-time LTL safety property. On a finite prefix containing only
`hello`, LamaConv reports `unknown`, not `true`, because the future could still
contain a violating event. Once a non-`hello` event is observed, the verdict is
definitively `false`. Seeing `unknown` for clean prefixes is therefore expected
for this example.

Extra dependencies:

- Java;
- LamaConv `rltlconv.jar`, not included in this repository;
- `websocket-server`.

The trusted LamaConv oracle dynamically calls the external LamaConv jar to
generate a monitor. This example will not run until you obtain that jar from the
LamaConv distribution and place it in `oracle/LamaConvOracle/`.

Preflight check:

```bash
test -f "$ROSMONITORING_HOME/oracle/LamaConvOracle/rltlconv.jar" || {
  echo "Missing rltlconv.jar. Download LamaConv and put rltlconv.jar in oracle/LamaConvOracle/."
  exit 1
}
```

Then run:

```bash
cd "$ROSMONITORING_HOME/oracle/LamaConvOracle"
python3 oracle.py --online --property property --port 8080 --tense future --lamaconvloc . --lamaconvfn rltlconv.jar
```

The LamaConv oracle imports the legacy-compatible `websocket_server.py` bundled
in `oracle/TLOracle/`; this avoids API differences in newer installed
`websocket-server` packages.

You can check the oracle behavior without ROS:

```bash
cd "$ROSMONITORING_HOME/oracle/LamaConvOracle"
python3 oracle.py --offline --property property --trace ../../examples/oracle_runs/chatter_good.jsonl --tense future --lamaconvloc . --lamaconvfn rltlconv.jar
python3 oracle.py --offline --property property --trace ../../examples/oracle_runs/chatter_bad.jsonl --tense future --lamaconvloc . --lamaconvfn rltlconv.jar
```

In another terminal:

```bash
cd "$ROSMONITORING_HOME"
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
python3 -m rosmonitoring.cli validate "$ROSMONITORING_HOME/examples/tutorials/topic_filter_lamaconv.yaml"
python3 -m rosmonitoring.cli generate "$ROSMONITORING_HOME/examples/tutorials/topic_filter_lamaconv.yaml" --ros-version ros2 --output "$ROSMONITORING_HOME/examples_lamaconv_ws/src"

cd "$ROSMONITORING_HOME/examples_lamaconv_ws"
rm -rf build install log
colcon build
. install/setup.bash
ros2 run monitor lamaconv_chatter_guard -- --dashboard
```

Then use the same listener and talker commands from Example 1, changing the
workspace path to `examples_lamaconv_ws`.

## Example 4: Service Filtering

The trusted legacy oracle properties in this repository are topic-oriented, so
this service example uses `examples/oracles/tutorial_oracle.py`. It still
exercises the same WebSocket protocol and generated ROS2 service proxy code.

Terminal A:

```bash
cd "$ROSMONITORING_HOME"
python3 examples/oracles/tutorial_oracle.py --port 8080 --log ./examples_service_ws/logs/oracle.jsonl
```

Terminal B, start the real service:

```bash
cd "$ROSMONITORING_HOME"
python3 examples/ros2_system/add_two_ints_server.py
```

Terminal C, run the monitor:

```bash
cd "$ROSMONITORING_HOME"
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
python3 -m rosmonitoring.cli validate "$ROSMONITORING_HOME/examples/tutorials/service_filter.yaml"
python3 -m rosmonitoring.cli generate "$ROSMONITORING_HOME/examples/tutorials/service_filter.yaml" --ros-version ros2 --output "$ROSMONITORING_HOME/examples_service_ws/src"

cd "$ROSMONITORING_HOME/examples_service_ws"
rm -rf build install log
colcon build
. install/setup.bash
ros2 run monitor add_two_ints_guard -- --dashboard --fresh-session
```

Terminal D, call accepted and rejected requests:

```bash
cd "$ROSMONITORING_HOME/examples_service_ws"
. install/setup.bash
python3 ../examples/ros2_system/add_two_ints_client.py 2 3
python3 ../examples/ros2_system/add_two_ints_client.py -1 7
```

The first call reaches `/add_two_ints`. The second is rejected by the oracle and
the real service is not called.

In the dashboard, open **Observed Events** to inspect the service payloads. The
accepted call should show a `request` payload like `{"a": 2, "b": 3}` and a
`response` payload like `{"sum": 5}`. The rejected call should show only the
`request` payload because the proxy does not forward it to the real service.
The **Status Timeline** also includes the same payload beside the oracle verdict.

## Example 5: Ordered Topic Logging

This example has no oracle. It shows source-time ordering with
`geometry_msgs/PoseStamped`.

Terminal A:

```bash
cd "$ROSMONITORING_HOME"
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
python3 -m rosmonitoring.cli validate "$ROSMONITORING_HOME/examples/tutorials/ordered_pose_log.yaml"
python3 -m rosmonitoring.cli generate "$ROSMONITORING_HOME/examples/tutorials/ordered_pose_log.yaml" --ros-version ros2 --output "$ROSMONITORING_HOME/examples_ordered_ws/src"

cd "$ROSMONITORING_HOME/examples_ordered_ws"
rm -rf build install log
colcon build
. install/setup.bash
ros2 run monitor ordered_pose_log
```

Terminal B:

```bash
cd "$ROSMONITORING_HOME/examples_ordered_ws"
. install/setup.bash
python3 ../examples/ros2_system/ordered_pose_publisher.py
```

After stopping the monitor:

```bash
cat "$ROSMONITORING_HOME/examples_ordered_ws/logs/ordered_pose_log.jsonl"
```

The event with source stamp `10` should appear before source stamp `20`, even
though it was published second.

## Example 6: Global Ordered Topics

This example has one monitor with two ordered topics. The monitor merges both
topics into one source-time stream before writing the log or talking to an
oracle.

Terminal A:

```bash
cd "$ROSMONITORING_HOME"
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
python3 -m rosmonitoring.cli validate "$ROSMONITORING_HOME/examples/tutorials/global_ordered_topics.yaml"
python3 -m rosmonitoring.cli generate "$ROSMONITORING_HOME/examples/tutorials/global_ordered_topics.yaml" --ros-version ros2 --output "$ROSMONITORING_HOME/examples_global_ordered_ws/src"

cd "$ROSMONITORING_HOME/examples_global_ordered_ws"
rm -rf build install log
colcon build
. install/setup.bash
ros2 run monitor global_ordered_pose_log
```

Terminal B:

```bash
cd "$ROSMONITORING_HOME/examples_global_ordered_ws"
. install/setup.bash
python3 ../examples/ros2_system/ordered_multi_publisher.py
```

After stopping the monitor:

```bash
cat "$ROSMONITORING_HOME/examples_global_ordered_ws/logs/global_ordered_pose_log.jsonl"
```

The first two rows should be `ordered_beta` at time `10.0`, then
`ordered_alpha` at time `20.0`, even though `ordered_alpha` was published first.

## Example 7: Mixed Dashboard

This example runs two generated monitors from one package:

- filtered `/chatter`;
- ordered `/ordered_pose`.

Use the small tutorial oracle:

```bash
cd "$ROSMONITORING_HOME"
python3 examples/oracles/tutorial_oracle.py --port 8080 --log ./examples_dashboard_ws/logs/oracle.jsonl
```

Generate and launch both monitors with one dashboard:

```bash
cd "$ROSMONITORING_HOME"
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/humble/setup.bash
python3 -m rosmonitoring.cli generate "$ROSMONITORING_HOME/examples/tutorials/dashboard_mixed.yaml" --ros-version ros2 --output "$ROSMONITORING_HOME/examples_dashboard_ws/src"

cd "$ROSMONITORING_HOME/examples_dashboard_ws"
rm -rf build install log
colcon build
. install/setup.bash
ros2 launch monitor monitor.launch.py dashboard:=true fresh_session:=true
```

Open:

```text
http://127.0.0.1:8765
```

The dashboard is served by the first monitor, but it reads the shared status log
and shows both monitors. Use the monitor selector to switch between `All
monitors`, `dashboard_chatter_guard`, and `dashboard_ordered_pose_log`.
The **Statistics** tab plots the selected events, and the **Sequence** tab shows
one dynamically updating sequence diagram. With `All monitors` selected, it
combines both monitors; selecting a single monitor focuses the diagram on that
monitor only. Arrows point from the observed ROS endpoint to the monitor, then
from the oracle back to the monitor when an oracle verdict is available. For
accepted filters/proxy calls, the diagram also shows the forwarding arrow from
the monitor to the original topic or service.

Only `dashboard_chatter_guard` has an oracle in this example. It should show
`currently_true` and `currently_false` verdicts. `dashboard_ordered_pose_log` is
intentionally log-only, so its `verdict_raw` is `unknown`: the events were
observed and ordered, but no oracle property was queried.

The same verdicts are also published in the ROS graph, as in the old
ROSMonitoring implementation. In another terminal you can inspect the
application-facing verdict topic:

```bash
cd "$ROSMONITORING_HOME/examples_dashboard_ws"
. install/setup.bash
ros2 topic echo /dashboard_chatter_guard/monitor_verdict std_msgs/msg/String
```

Then run:

```bash
python3 ../examples/ros2_system/chatter_talker.py --topic /chatter_mon --once
python3 ../examples/ros2_system/ordered_pose_publisher.py
```

`chatter_talker.py` runs forever unless `--once` is passed. The chatter monitor
observes `/chatter_mon`; the ordered-pose monitor observes `/ordered_pose`.

## Offline Oracle Experiments

Before starting ROS, you can test trusted oracles on JSONL traces.

TL/Reelay:

```bash
cd "$ROSMONITORING_HOME/oracle/TLOracle"
python3 oracle.py --offline --discrete --property chatter_property --trace ../../examples/oracle_runs/chatter_bad.jsonl
```

LamaConv:

```bash
cd "$ROSMONITORING_HOME/oracle/LamaConvOracle"
python3 oracle.py --offline --property property --trace ../../examples/oracle_runs/chatter_bad.jsonl --tense future --lamaconvloc . --lamaconvfn rltlconv.jar
```

RML:

```bash
cd "$ROSMONITORING_HOME"
./oracle/RMLOracle/prolog/offline_monitor.sh ./oracle/RMLOracle/rml/test.pl ./examples/oracle_runs/rml_mixed.jsonl
```

## Notes

- The example YAML files are intentionally small. They are meant to be copied
  and modified.
- The trusted oracle implementations are not edited by these examples.
- If a trusted oracle returns `true` or `false`, ROSMonitoring records a
  terminal verdict. Passive monitors may stop; communication-path monitors keep
  running.
- If the oracle dependencies are unavailable, the generated monitors still run
  in log-only mode, but the oracle-specific examples will not produce verdicts.
