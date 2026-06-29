# ROSMonitoring Architecture

ROSMonitoring keeps the original framework split:

- instrumentation: a YAML-driven generator creates monitor nodes and launch files;
- monitors: generated ROS1 or ROS2 Python nodes observe topics and services;
- oracle: an optional external WebSocket service receives JSON events and returns verdicts.

The new generator is a normal Python package. Configuration parsing, validation,
event ordering, status aggregation, and file rendering are separated so they can
be tested without ROS installed.

## Event Shape

Generated monitors write and send one legacy-compatible JSON object per event.
Topic message fields are flattened next to `topic` and `time`:

```json
{
  "topic": "battery_status",
  "time": 1782469999.9,
  "data": "critical"
}
```

For services, requests and responses are explicit top-level keys:

```json
{
  "service": "set_led",
  "time": 1782469999.9,
  "request": {"data": true}
}
```

The generator keeps monitor id, interface name, direction, ordered flag, and
observed time internally for buffering and status reporting, but those wrapper
fields are not sent to legacy oracles.

## Ordered Topics and Services

Legacy `ordered: true` is still accepted. New configurations can use:

```yaml
ordering:
  enabled: true
  source: header_stamp
  max_delay_ms: 50
```

The generated monitor uses `header.stamp` when available, then `stamp`, then the
local observation time. All ordered topics and services in one monitor share a
single source-time buffer, so the oracle/log sees one merged stream rather than
one stream per interface. This also reorders stamped events on a single topic,
which is useful when several publishers publish to the same topic.

`max_delay_ms` is the reorder window used before flushing events to the
oracle/log. A monitor with multiple ordered interfaces uses the largest
configured `max_delay_ms` as its global ordered window. A value of `0` flushes
immediately in source-time order among events already buffered.

For inline topic filters and service requests, propagation waits until the event
has been flushed and its oracle verdict is known. The wait is bounded by the
global ordered window; if no later source timestamp arrives, the monitor flushes
known buffered events through the waiting event when that deadline expires.

## Browser Status

Generated monitors append status events to a JSONL file. Start the dashboard
with:

```bash
python3 -m rosmonitoring.cli status --status-log ./logs/rosmonitoring-status.jsonl
```

Then open `http://127.0.0.1:8765`.

## ROS Verdict Topic

Generated monitors also publish verdicts back into the ROS graph. The topic name
matches the legacy ROSMonitoring contract:

```text
/<monitor_id>/monitor_verdict
```

The topic carries `std_msgs/String` and contains the raw normalized oracle
verdict, for example `currently_true`, `currently_false`, `true`, `false`, or
`unknown`. ROS1 uses a latched publisher. ROS2 uses transient-local QoS so a
subscriber can still see the latest verdict after it starts.

Status JSONL and the browser dashboard are diagnostic views over monitor
activity; the verdict topic is the online ROS interface that application nodes
can subscribe to when they need to react to the monitor.
