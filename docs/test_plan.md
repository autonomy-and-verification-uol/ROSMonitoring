# ROSMonitoring Test Plan

This document explains what is tested, why those tests exist, and which commands
should be run before publishing a release or replacing the upstream master
branch.

The goal is not only line coverage. ROSMonitoring is a compatibility framework:
the tests must protect the old behavior that existing ROS applications and
oracles rely on.

## Compatibility Requirements Under Test

The following contracts are considered release-critical:

1. Legacy flat JSON events are preserved for old oracles.
2. Topic events contain `topic`, `time`, and flattened payload fields.
3. Service request events contain `service`, `time`, and `request`.
4. Service response events contain `service`, `time`, and `response`.
5. ROS1 and ROS2 generation both support topics and services.
6. `action: log` never blocks communication.
7. `action: filter` blocks negative topic and service-request verdicts.
8. `currently_false` and final `false` remain distinguishable.
9. Warning levels distinguish current and final false verdicts.
10. Passive monitors may stop after terminal `true` or `false`.
11. Filter/service monitors keep running after terminal verdicts.
12. Offline monitors publish/report `unknown`, not definitive `true`.
13. Oracle failures are reported and degrade to `unknown` rather than crashing.
14. The legacy ROS verdict topic exists as `/<monitor_id>/monitor_verdict`.
15. ROS1 verdict topics are latched.
16. ROS2 verdict topics use transient-local durability.
17. Services are monitored in two steps: request then response.
18. Ordered mode uses source/publication time.
19. Ordered mode merges all ordered topics/services in one monitor.
20. Ordered mode also handles stamped out-of-order events on one topic.
21. Generated packages declare required ROS dependencies.
22. Generated packages remove stale monitor files when regenerated.
23. Dashboard/status logs support multiple monitors.
24. Fresh sessions clear logs predictably.
25. Example YAML files remain valid and renderable.

## Test Layers

### Pure Python Regression Tests

Run:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider
```

These tests do not require ROS to be installed. They cover:

- configuration parsing and legacy YAML compatibility;
- all ROS/interface/action/oracle/status/ordering render combinations;
- message-to-dictionary conversion;
- source timestamp extraction;
- deterministic ordered-buffer behavior;
- ordered-buffer stress cases;
- generator output for ROS1 and ROS2;
- generated monitor Python source behavior without ROS imports;
- oracle verdict parsing;
- oracle failure fallback;
- terminal verdict behavior;
- warning-level behavior;
- verdict-topic payload normalization;
- status-disabled monitors;
- stale generated-file cleanup;
- status aggregation;
- dashboard payload aggregation, limits, and multi-monitor event ordering;
- example source compilation;
- trusted legacy oracle tree presence.

### ROS2 Integration Test

Run:

```bash
source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider tests/test_ros2_integration.py
```

This test builds and runs a real ROS2 workspace. It checks:

- `colcon` build of generated package;
- real monitor executables;
- online WebSocket oracle connection;
- topic filter propagation for accepted verdicts;
- topic filter blocking for negative verdicts;
- offline log-only topic monitoring;
- service proxy creation;
- accepted service request forwarding;
- rejected service request blocking;
- service response event logging;
- single-topic ordered `PoseStamped` reordering;
- global ordered reordering across two topics;
- status JSONL rows;
- dashboard API payload;
- ROS2 transient-local verdict topics for online, offline, and service
  monitors.

### ROS1 Docker Smoke Test

Run:

```bash
tests/ros1_docker_smoke.sh
```

This test uses `ros:noetic-ros-base` and checks:

- ROS1 package generation;
- Catkin build;
- `roscore` startup;
- ROS1 topic monitor forwarding;
- ROS1 service monitor forwarding;
- event JSONL rows;
- status JSONL rows;
- legacy latched verdict topic `/ros1_smoke_monitor/monitor_verdict`.

### Tutorial Generation and Build Matrix

Run:

```bash
source /opt/ros/humble/setup.bash
export PYTHONPATH="$PWD/src:${PYTHONPATH:-}"
base=/tmp/rosmonitoring_tutorial_builds
rm -rf "$base"
mkdir -p "$base"
for yaml in examples/tutorials/*.yaml; do
  name="$(basename "$yaml" .yaml)"
  if [ "$name" = "ros1_legacy_compatible" ]; then
    python3 -m rosmonitoring.cli validate "$yaml" --ros-version ros1
    python3 -m rosmonitoring.cli generate "$yaml" --ros-version ros1 --output "$base/$name/src"
  else
    python3 -m rosmonitoring.cli validate "$yaml" --ros-version ros2
    python3 -m rosmonitoring.cli generate "$yaml" --ros-version ros2 --output "$base/$name/src"
    (cd "$base/$name" && colcon build)
  fi
done
```

This verifies that every tutorial YAML can generate, and every ROS2 tutorial
package can build in an isolated workspace.

## Stress Tests Added During Hardening

The hardening pass added or strengthened tests for:

- large out-of-order ordered-buffer traces;
- dense delayed-flush ordered-buffer traces;
- mixed nested ROS-like payload conversion;
- all supported YAML/generator combinations across ROS1/ROS2;
- oracle exceptions;
- raw non-JSON string verdicts;
- status-disabled monitors;
- multi-monitor ROS1/ROS2 generation;
- dashboard aggregation over hundreds of events from multiple monitors;
- status aggregation over hundreds of events and multiple interfaces;
- offline and service verdict topics in real ROS2 integration.

## Latest Verified Results

The hardening pass on this checkout produced:

- Unsourced unit/regression suite: `59 passed, 1 skipped`.
- Full pytest suite with ROS Humble sourced: `60 passed`.
- Focused generator/status/config/events hardening suite: `49 passed`.
- Standalone ROS2 integration: `1 passed`.
- ROS1 Docker smoke: `ROS1 smoke ok`.
- Tutorial matrix: all tutorial YAML files validated, 10 ROS2 packages built,
  and the ROS1 tutorial package generated.
- Dashboard JavaScript syntax check: passed with `node --check`.

## Manual Release Checklist

Before pushing a release branch:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider

source /opt/ros/humble/setup.bash
PYTHONDONTWRITEBYTECODE=1 python3 -m pytest -q -p no:cacheprovider tests/test_ros2_integration.py

tests/ros1_docker_smoke.sh
```

Then run the tutorial generation/build matrix above.

Finally check for generated artifacts:

```bash
find . -path ./.git -prune -o \( -name '__pycache__' -o -name '.pytest_cache' -o -name '*.pyc' -o -name '*.pyo' -o -name 'build' -o -name 'install' -o -name 'log' -o -name 'logs' -o -name '*.egg-info' \) -print
find . -maxdepth 2 -type d \( -name '*_ws' -o -name 'generated*' \) -print
```

The first command should print nothing. The second should print nothing unless a
developer intentionally recreated ignored local workspaces.

## Known Limits

No test suite can prove every possible ROS deployment, QoS configuration,
network layout, or oracle formalism. The current suite is designed to protect
the framework contracts and the highest-risk combinations:

- ROS1 and ROS2 generation;
- real ROS2 runtime behavior;
- real ROS1 runtime smoke behavior;
- topic and service monitoring;
- log and filter actions;
- online, offline, and oracle-error cases;
- ordered single-topic and multi-interface traces;
- old oracle protocol;
- old verdict topic behavior;
- dashboard/status introspection.

Future work should add performance benchmarks for high-frequency topics and a
CI job that runs the ROS2 and ROS1 tests in containers.
