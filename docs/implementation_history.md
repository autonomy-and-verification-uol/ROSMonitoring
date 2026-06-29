# ROSMonitoring 3 Implementation History and Handoff

This document records the migration from the older ROSMonitoring branches to
the integrated version in this repository. It is intended both for maintainers
and for future Codex sessions that need enough context to continue the work
without rediscovering the design decisions.

## Objective

This version is intended to become the new master version of ROSMonitoring. It
integrates and modernizes the historical feature lines:

- ROS1 topic monitoring from the original master branch.
- ROS1 service monitoring.
- ROS2 topic monitoring.
- ROS2 service monitoring.
- Ordered monitoring for source/publication-time traces.
- Legacy oracle compatibility with the trusted `oracle/` tree.
- New status logging and browser dashboard support.
- Regression and integration tests for the generator and real ROS runtimes.

The implementation is generator-based. Files under `src/rosmonitoring/`
generate complete ROS packages into user-selected workspaces. Generated
workspaces are build artifacts and should not be committed.

## Repository Layout

Important source directories:

- `src/rosmonitoring/`: Python package containing the CLI, config parser,
  event utilities, dashboard server, and monitor generator.
- `oracle/`: trusted legacy oracle bundle. Treat this as compatibility material
  and avoid unnecessary refactors.
- `examples/tutorials/`: YAML configurations covering topics, services,
  ordering, dashboards, ROS1 compatibility, and different oracle styles.
- `examples/ros2_system/`: small ROS2 nodes used by tutorials and manual tests.
- `examples/oracles/`: tutorial WebSocket oracle.
- `examples/oracle_runs/`: JSONL traces for offline examples.
- `tests/`: unit tests, regression tests, ROS2 integration, and ROS1 Docker
  smoke test.
- `docs/architecture.md`: short architecture reference.
- `docs/implementation_history.md`: this handoff.
- `docs/test_plan.md`: coverage matrix and release checklist.

Generated/local directories should stay out of Git:

- `generated/`, `generated_ros2/`
- `tutorial_ws/`, `tutorial_service_ws/`, `tutorial_ordered_ws/`
- `examples_ws/`, `examples_tl_ws/`, `examples_rml_ws/`
- `examples_lamaconv_ws/`, `examples_service_ws/`
- `examples_ordered_ws/`, `examples_dashboard_ws/`
- `build/`, `install/`, `log/`, `logs/`
- Python caches, package build outputs, coverage outputs.

## Packaging and CLI

The project is now a normal Python package named `rosmonitoring`, configured in
`pyproject.toml`.

Important points:

- Build backend: Hatchling.
- Package root: `src/rosmonitoring`.
- Console script: `rosmonitoring = rosmonitoring.cli:main`.
- Required dependency: `PyYAML`.
- Optional online/dev dependencies: `websocket-client`, `websockets`, `pytest`.

Common commands:

```bash
python3 -m pip install -e '.[dev]'
python3 -m rosmonitoring.cli validate examples/tutorials/tutorial_topic.yaml --ros-version ros2
python3 -m rosmonitoring.cli generate examples/tutorials/tutorial_topic.yaml --ros-version ros2 --output tutorial_ws/src
python3 -m rosmonitoring.cli status --status-log ./logs/status.jsonl
```

For generated ROS packages, use ROS tooling:

- ROS2: `colcon build`
- ROS1: `catkin_make`

## Configuration Model

Configuration parsing lives in `src/rosmonitoring/config.py`.

The parser accepts legacy and new YAML forms:

- `monitors: [{monitor: ...}]`
- legacy `nodes`
- `topics`
- `services`
- topic `publishers` and `subscribers`
- legacy `ordered: true`
- structured `ordering`
- structured `status`
- `oracle.url`, `oracle.port`, `oracle.action`, and timeout
- top-level or command-line `ros_version`

The normalized model uses dataclasses:

- `ProjectConfig`
- `MonitorSpec`
- `InterfaceSpec`
- `OracleSpec`
- `OrderingSpec`
- `StatusSpec`

Future generator changes should depend on this normalized model rather than raw
YAML shape.

## Generated ROS Packages

Generation lives in `src/rosmonitoring/generator.py`.

For ROS2, the generator writes an `ament_python` package:

```text
monitor/
  package.xml
  setup.py
  setup.cfg
  resource/monitor
  monitor/__init__.py
  monitor/<monitor_id>.py
  launch/monitor.launch.py
```

For ROS1, it writes a Catkin package:

```text
monitor/
  package.xml
  CMakeLists.txt
  launch/<monitor_id>.launch
  src/<monitor_id>.py
```

Stale generated monitor files are removed from the output package when a config
is regenerated, so old executables do not remain after a monitor is deleted or
renamed.

Message and service package dependencies are inferred from interface types.
`std_msgs` is always included because generated monitors publish the legacy
verdict topic.

## Event Shape and Oracle Compatibility

The old oracle protocol is preserved. Generated monitors send flat JSON events,
not a new wrapper schema.

Topic event:

```json
{
  "topic": "chatter",
  "time": 1782458599.1,
  "data": "hello"
}
```

Service request event:

```json
{
  "service": "add_two_ints",
  "time": 1782458599.1,
  "request": {"a": 2, "b": 3}
}
```

Service response event:

```json
{
  "service": "add_two_ints",
  "time": 1782458599.2,
  "response": {"sum": 5}
}
```

Internal fields are kept under `__...` keys during processing and removed by
`_wire_event` before events are sent to legacy oracles or written to event logs.

Do not break this flat protocol unless a versioned compatibility mode is added.

## Verdict Handling

The old verdict semantics are preserved.

Accepted verdict forms:

- Boolean `true` and `false`.
- String `true` and `false`.
- Legacy current verdicts: `currently_true`, `currently_false`.
- Other negative strings: `violation`, `violated`.
- `unknown`, used for log-only monitors or oracle errors.

Negative verdicts are:

- `false`
- `currently_false`
- `violation`
- `violated`

Filtering behavior:

- `action: filter` blocks propagation on negative verdicts.
- `action: log` records verdicts but never blocks communication.

Warning behavior:

- `warning: 0`: no warnings.
- `warning: 1`: warns on `currently_false` and final negative verdicts.
- `warning: 2`: warns only on final-style negative verdicts such as `false`,
  `violation`, and `violated`.

Terminal-verdict behavior:

- `true` and `false` are definitive terminal verdicts.
- Passive monitors may stop after a terminal verdict to save resources.
- Filter/service monitors must keep running after terminal verdicts because
  stopping them would break the ROS communication path.

Keep `currently_false` distinct from final `false` in logs, status rows, and the
dashboard.

## Legacy ROS Verdict Topic

Generated monitors publish each raw normalized verdict into the ROS graph on the
legacy topic:

```text
/<monitor_id>/monitor_verdict
```

Type: `std_msgs/String`.

ROS1 behavior:

- Latched publisher.
- Queue size `1000`.

ROS2 behavior:

- Transient-local durability.
- Depth `1000`.

This topic is the application-facing integration point for nodes that need to
react to monitor verdicts. JSONL logs and the dashboard are diagnostic views;
they are not the only monitor outputs.

Manual checks:

```bash
ros2 topic echo /dashboard_chatter_guard/monitor_verdict std_msgs/msg/String
rostopic echo /ros1_smoke_monitor/monitor_verdict
```

Tests cover this behavior in generator tests, ROS2 integration, and the ROS1
Docker smoke test.

## Topic Monitoring

Passive topic log:

- The monitor subscribes to the original topic.
- It writes events and optionally asks the oracle.
- It does not change application communication.

Filtering topic:

- Application publishers publish to `<topic>_mon`.
- The monitor subscribes to `<topic>_mon`.
- Accepted events are republished to the original `<topic>`.
- Negative verdicts are not republished.

The event sent to the oracle uses the original legacy topic name, not the
`_mon` name.

## Service Monitoring

Services are monitored in two steps, matching the older behavior.

For `/add_two_ints`:

- Monitor serves `/add_two_ints_mon`.
- Original server serves `/add_two_ints`.
- Client calls `/add_two_ints_mon`.
- Monitor sends/logs the request event.
- If accepted, monitor calls `/add_two_ints`.
- Monitor sends/logs the response event.
- Monitor returns the original response to the caller.
- If rejected, original service is not called and the generated default response
  object is returned.

ROS2 service monitors use `ReentrantCallbackGroup` and
`MultiThreadedExecutor` to avoid deadlocks when a service callback forwards to a
service client.

## Ordered Monitoring

Ordering utilities live in `src/rosmonitoring/events.py`.

The implementation preserves source/publication order across all ordered topics
and services in the same monitor. It also reorders stamped events on a single
topic, which is useful when multiple publishers publish to the same topic.

Source time selection:

1. `header.stamp`
2. `stamp`
3. local observation time

All ordered interfaces in one monitor share one `OrderedEventBuffer`. Ordered
mode is therefore global inside a monitor, not one isolated buffer per topic.

Flush behavior:

- New events update a source-time watermark.
- Events older than `watermark - max_delay_ms` become ready.
- `max_delay_ms: 0` flushes immediately among already buffered events.
- Shutdown flushes all remaining buffered events.

Inline filtering behavior:

- Ordered filter topics and service requests wait for their own ordered oracle
  verdict before forwarding.
- The wait is bounded by the monitor-level ordered window.
- The monitor-level window is the largest `max_delay_ms` among ordered
  interfaces in that monitor.

## Dashboard and Status Logs

Status support lives in `src/rosmonitoring/status.py` and generated monitor
code.

Status rows include:

- monitor id
- status kind
- ROS version
- session id
- interface
- direction
- payload
- raw oracle verdict
- boolean propagation decision
- terminal verdict marker
- whether a passive monitor would stop
- ordered flag
- oracle response

Dashboard modes:

```bash
python3 -m rosmonitoring.cli status --status-log ./logs/status.jsonl
ros2 run monitor chatter_guard -- --dashboard
ros2 launch monitor monitor.launch.py dashboard:=true fresh_session:=true
```

Dashboard features:

- All-monitors and per-monitor views.
- Event and violation counts.
- Recent observed events and payload introspection.
- Status timeline.
- Verdict timeline.
- Text filtering.
- Statistics tab and browser-side plots.
- Dynamic sequence diagram.

Sequence diagram directions:

- ROS endpoint to monitor for observed events.
- Oracle to monitor for verdict responses.
- Monitor to ROS endpoint for accepted forwarding.

Fresh session behavior:

- `--fresh-session` clears the monitor event log.
- Shared status logs are cleared once per `ROSMONITORING_SESSION_ID`.
- Launch files can pass `fresh_session:=true`.

## Examples

Tutorial YAML files:

- `examples/tutorials/tutorial_topic.yaml`
- `examples/tutorials/tutorial_service.yaml`
- `examples/tutorials/topic_filter_tl.yaml`
- `examples/tutorials/topic_filter_rml.yaml`
- `examples/tutorials/topic_filter_lamaconv.yaml`
- `examples/tutorials/service_filter.yaml`
- `examples/tutorials/ordered_pose_log.yaml`
- `examples/tutorials/global_ordered_topics.yaml`
- `examples/tutorials/ros2_topics_services_ordered.yaml`
- `examples/tutorials/dashboard_mixed.yaml`
- `examples/tutorials/ros1_legacy_compatible.yaml`

ROS2 example nodes:

- `examples/ros2_system/chatter_talker.py`
- `examples/ros2_system/chatter_listener.py`
- `examples/ros2_system/add_two_ints_server.py`
- `examples/ros2_system/add_two_ints_client.py`
- `examples/ros2_system/ordered_pose_publisher.py`
- `examples/ros2_system/ordered_multi_publisher.py`

Tutorial oracle:

- `examples/oracles/tutorial_oracle.py`
- Returns `currently_true` for accepted demo events.
- Returns `currently_false` for documented violations such as topic data
  `drop` or service requests with negative `a`.

Legacy oracle notes:

- TL/Reelay example uses `oracle/TLOracle/oracle.py`.
- RML example uses `oracle/RMLOracle/prolog/online_monitor.sh`.
- LamaConv example uses `oracle/LamaConvOracle/oracle.py`.
- LamaConv needs `rltlconv.jar`; if unavailable or misconfigured, verdicts can
  be `unknown`.

## Tests

No-ROS tests:

- `tests/test_config.py`
- `tests/test_events.py`
- `tests/test_generator.py`
- `tests/test_cli.py`
- `tests/test_status.py`
- `tests/test_examples_catalog.py`
- `tests/test_legacy_oracle_tree.py`

ROS2 integration:

- `tests/test_ros2_integration.py`
- Generates a fresh workspace.
- Builds with `colcon`.
- Starts real ROS2 monitor processes, a WebSocket oracle, and a ROS2 service
  server.
- Checks topic filter propagation/blocking.
- Checks service proxy request/response behavior.
- Checks ordered single-topic and global multi-topic behavior.
- Checks status JSONL and dashboard API.
- Checks `/online_topic_guard/monitor_verdict`.

ROS1 smoke:

- `tests/ros1_docker_smoke.sh`
- Uses `ros:noetic-ros-base`.
- Generates a ROS1 Catkin package.
- Starts `roscore`, service server, and monitor.
- Checks topic forwarding, service forwarding, logs, status, and
  `/ros1_smoke_monitor/monitor_verdict`.

Useful test commands:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider

source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider tests/test_ros2_integration.py

tests/ros1_docker_smoke.sh
```

Recent verified results before this handoff:

- Unsourced unit/regression suite: `59 passed, 1 skipped`.
- Full pytest suite with ROS Humble sourced: `60 passed`.
- Focused generator/status/config/events hardening suite: `49 passed`.
- Standalone ROS2 integration: `1 passed`.
- ROS1 Docker smoke: `ROS1 smoke ok`.
- Tutorial matrix: all tutorial YAML files validated, 10 ROS2 packages built,
  and the ROS1 tutorial package generated.
- Dashboard JavaScript syntax check: passed with `node --check`.

## Documentation Updates

`README.md` now includes:

- Project overview.
- Installation and ROS dependency notes.
- Step-by-step topic tutorial.
- Step-by-step service tutorial.
- Dashboard usage.
- Fresh-session guidance.
- Configuration reference.
- Event shape and oracle protocol.
- Runtime verdict topic.
- Testing commands.
- Migration notes.
- Citation section for the original paper and service/ordered-topic extension.

`examples/README.md` is a command-heavy tutorial catalog.

`docs/architecture.md` summarizes generator, monitors, oracles, ordering,
status, dashboard, and verdict-topic behavior.

`docs/test_plan.md` documents the release-critical compatibility requirements,
the test layers that protect them, and the commands to run before publishing.

## Compatibility Contracts

Do not break these without a deliberate compatibility plan:

1. Flat legacy oracle event schema.
2. Topic events with `topic`, `time`, and payload fields.
3. Service request events with `service`, `time`, and `request`.
4. Service response events with `service`, `time`, and `response`.
5. Trusted `oracle/` tree remains present.
6. `currently_false` remains distinct from final `false`.
7. Passive monitors may stop on terminal verdicts.
8. Filter/service monitors keep running after terminal verdicts.
9. Verdict topic name remains `/<monitor_id>/monitor_verdict`.
10. Verdict topic type remains `std_msgs/String`.
11. ROS1 verdict publisher remains latched.
12. ROS2 verdict publisher remains transient-local.
13. Services remain monitored as request and response events.
14. Ordered mode remains global across ordered interfaces inside one monitor.
15. Generated workspaces remain out of Git.

## Git and Push Notes

This repository should be used from:

```bash
cd /media/angelo/WorkData/git/NewROSMonitoring/ROSMonitoring
```

Do not commit from `/media/angelo/WorkData/git` or
`/media/angelo/WorkData/git/NewROSMonitoring` unless those are intentionally
initialized as separate repositories. The parent `/media/angelo/WorkData/git`
has another remote and may report many unrelated sibling projects.

Before committing, check:

```bash
git status --short
find . \( -name '__pycache__' -o -name '.pytest_cache' -o -name '*.pyc' -o -name '*.pyo' \) -print
find . -maxdepth 2 -type d \( -name '*_ws' -o -name 'generated*' \) -print
```

The cleanup commands should print no generated artifacts unless the user has
intentionally recreated local workspaces.

## Future Codex Instructions

When continuing work:

1. Read `README.md`, `examples/README.md`, `docs/architecture.md`,
   `docs/test_plan.md`, and this file before broad changes.
2. Preserve the legacy oracle protocol and verdict topic behavior.
3. Change `src/rosmonitoring/generator.py`, not generated workspaces.
4. Regenerate temporary workspaces only for testing; do not commit them.
5. Run `tests/test_generator.py` after generator changes.
6. Run `tests/test_ros2_integration.py` after ROS2 runtime, service, ordering,
   dashboard, or verdict-topic changes.
7. Run `tests/ros1_docker_smoke.sh` after ROS1 runtime/package-generation
   changes.
8. Keep documentation commands portable with `ROSMONITORING_HOME`; avoid local
   absolute paths.
9. If dashboard JavaScript changes, extract it and run `node --check` as
   documented in the README.
10. Avoid destructive Git commands. Preserve user changes and keep generated
    artifacts out of commits.
