import ast
import importlib.util
import json
from pathlib import Path

from rosmonitoring.config import parse_config
from rosmonitoring.generator import generate_project, render_monitor_source


def _config():
    return parse_config(
        {
            "monitors": [
                {
                    "monitor": {
                        "id": "battery_guard",
                        "log": "./logs/battery_guard.jsonl",
                        "status": {"enabled": True, "log": "./logs/status.jsonl"},
                        "oracle": {"url": "127.0.0.1", "port": 8080, "action": "nothing"},
                        "topics": [
                            {
                                "name": "battery_status",
                                "type": "std_msgs.msg.String",
                                "action": "filter",
                                "publishers": ["battery_supervisor"],
                                "ordering": {"enabled": True, "max_delay_ms": 25},
                            }
                        ],
                        "services": [
                            {
                                "name": "set_led",
                                "type": "example_interfaces.srv.SetBool",
                                "action": "filter",
                            }
                        ],
                    }
                }
            ]
        }
    )


def _passive_config():
    return parse_config(
        {
            "monitors": [
                {
                    "monitor": {
                        "id": "passive_guard",
                        "log": "./logs/passive_guard.jsonl",
                        "status": {"enabled": True, "log": "./logs/status.jsonl"},
                        "oracle": {"url": "127.0.0.1", "port": 8080, "action": "nothing"},
                        "topics": [
                            {
                                "name": "passive_text",
                                "type": "std_msgs.msg.String",
                                "action": "log",
                            }
                        ],
                    }
                }
            ]
        }
    )


def _offline_passive_config():
    return parse_config(
        {
            "monitors": [
                {
                    "monitor": {
                        "id": "offline_passive_log",
                        "log": "./logs/offline_passive_log.jsonl",
                        "status": {"enabled": True, "log": "./logs/status.jsonl"},
                        "topics": [
                            {
                                "name": "offline_text",
                                "type": "std_msgs.msg.String",
                                "action": "log",
                            }
                        ],
                    }
                }
            ]
        }
    )


def _ordered_multi_config():
    return parse_config(
        {
            "monitors": [
                {
                    "monitor": {
                        "id": "ordered_multi_guard",
                        "log": "./logs/ordered_multi_guard.jsonl",
                        "status": {"enabled": True, "log": "./logs/status.jsonl"},
                        "oracle": {"url": "127.0.0.1", "port": 8080, "action": "nothing"},
                        "topics": [
                            {
                                "name": "alpha",
                                "type": "std_msgs.msg.String",
                                "action": "log",
                                "ordering": {"enabled": True, "max_delay_ms": 10000},
                            },
                            {
                                "name": "beta",
                                "type": "std_msgs.msg.String",
                                "action": "log",
                                "ordering": {"enabled": True, "max_delay_ms": 10000},
                            },
                        ],
                        "services": [
                            {
                                "name": "set_led",
                                "type": "example_interfaces.srv.SetBool",
                                "action": "log",
                                "ordering": {"enabled": True, "max_delay_ms": 10000},
                            }
                        ],
                    }
                }
            ]
        }
    )


def _ordered_filter_config():
    return parse_config(
        {
            "monitors": [
                {
                    "monitor": {
                        "id": "ordered_filter_guard",
                        "log": "./logs/ordered_filter_guard.jsonl",
                        "status": {"enabled": True, "log": "./logs/status.jsonl"},
                        "oracle": {"url": "127.0.0.1", "port": 8080, "action": "nothing"},
                        "topics": [
                            {
                                "name": "guarded",
                                "type": "std_msgs.msg.String",
                                "action": "filter",
                                "publishers": ["guarded_pub"],
                                "ordering": {"enabled": True, "max_delay_ms": 1},
                            }
                        ],
                    }
                }
            ]
        }
    )


def _ordered_service_filter_config():
    return parse_config(
        {
            "monitors": [
                {
                    "monitor": {
                        "id": "ordered_service_filter_guard",
                        "log": "./logs/ordered_service_filter_guard.jsonl",
                        "status": {"enabled": True, "log": "./logs/status.jsonl"},
                        "oracle": {"url": "127.0.0.1", "port": 8080, "action": "nothing"},
                        "services": [
                            {
                                "name": "set_led",
                                "type": "example_interfaces.srv.SetBool",
                                "action": "filter",
                                "ordering": {"enabled": True, "max_delay_ms": 1},
                            }
                        ],
                    }
                }
            ]
        }
    )


def _same_name_topic_service_config():
    return parse_config(
        {
            "monitors": [
                {
                    "monitor": {
                        "id": "same_name_guard",
                        "log": "./logs/same_name_guard.jsonl",
                        "status": {"enabled": True, "log": "./logs/status.jsonl"},
                        "topics": [
                            {
                                "name": "shared",
                                "type": "std_msgs.msg.String",
                                "action": "log",
                                "ordering": {"enabled": True, "max_delay_ms": 10000},
                            }
                        ],
                        "services": [
                            {
                                "name": "shared",
                                "type": "example_interfaces.srv.SetBool",
                                "action": "log",
                                "ordering": {"enabled": True, "max_delay_ms": 10000},
                            }
                        ],
                    }
                }
            ]
        }
    )


def _status_disabled_config():
    return parse_config(
        {
            "monitors": [
                {
                    "monitor": {
                        "id": "status_disabled_guard",
                        "log": "./logs/status_disabled_guard.jsonl",
                        "status": {"enabled": False, "log": "./logs/status.jsonl"},
                        "topics": [
                            {
                                "name": "statusless",
                                "type": "std_msgs.msg.String",
                                "action": "log",
                            }
                        ],
                    }
                }
            ]
        }
    )


def _two_monitor_config():
    return parse_config(
        {
            "monitors": [
                {
                    "monitor": {
                        "id": "first_guard",
                        "log": "./logs/first_guard.jsonl",
                        "topics": [{"name": "first", "type": "std_msgs.msg.String", "action": "log"}],
                    }
                },
                {
                    "monitor": {
                        "id": "second_guard",
                        "log": "./logs/second_guard.jsonl",
                        "services": [{"name": "second", "type": "std_srvs.srv.SetBool", "action": "filter"}],
                    }
                },
            ]
        }
    )


def _load_rendered_monitor(tmp_path: Path, ros_version: str = "ros2", config=None):
    config = config or _config()
    source = render_monitor_source(config.monitors[0], ros_version)
    path = tmp_path / f"rendered_{ros_version}.py"
    path.write_text(source, encoding="utf-8")
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _prepare_rendered_monitor(module, monitor, tmp_path: Path, label: str):
    monitor.lock = module.threading.RLock()
    monitor.order_lock = module.threading.RLock()
    ordered_delays = [item["max_delay_ms"] for item in module.INTERFACES if item["ordered"]]
    monitor.ordered_buffer = module.OrderedEventBuffer(max(ordered_delays) if ordered_delays else 0)
    monitor.interfaces_by_key = {item["key"]: item for item in module.INTERFACES}
    monitor._log_warning = lambda message: None
    monitor._log_info = lambda message: None
    monitor.status_file = (tmp_path / f"status_{label}.jsonl").open("w+", encoding="utf-8")
    monitor.log_file = (tmp_path / f"events_{label}.jsonl").open("w+", encoding="utf-8")
    return tmp_path / f"events_{label}.jsonl", tmp_path / f"status_{label}.jsonl"


def test_rendered_ros2_monitor_is_valid_python_and_mentions_features():
    monitor = _config().monitors[0]
    source = render_monitor_source(monitor, "ros2")
    ast.parse(source)
    assert "create_subscription" in source
    assert "create_service" in source
    assert "monitor_verdict" in source
    assert "ROSMonitoringVerdictString" in source
    assert "TRANSIENT_LOCAL" in source
    assert "OrderedEventBuffer" in source
    assert "STATUS_LOG_PATH" in source
    assert "--dashboard" in source
    assert "ThreadingHTTPServer" in source
    assert "/api/dashboard" in source
    assert "recent_events" in source
    assert "dashboard_started" in source
    assert 'event["topic"] = interface["legacy_name"]' in source
    assert 'event["service"] = interface["legacy_name"]' in source
    assert 'event[direction] = payload_dict' in source
    assert "'key':" in source
    assert 'self.interfaces_by_key' in source
    assert '"ordered": true' not in source


def test_rendered_monitor_builds_legacy_topic_service_wire_events(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path)
    monitor = module.ROSMonitor_battery_guard.__new__(module.ROSMonitor_battery_guard)
    topic = next(item for item in module.INTERFACES if item["kind"] == "topic")
    service = next(item for item in module.INTERFACES if item["kind"] == "service")

    class Stamp:
        sec = 12
        nanosec = 300

    class Header:
        stamp = Stamp()

    class TopicMessage:
        __slots__ = ("header", "data")

        def __init__(self):
            self.header = Header()
            self.data = "low"

    class ServiceRequest:
        __slots__ = ("data",)

        def __init__(self):
            self.data = False

    topic_event = monitor._event(topic, TopicMessage())
    assert topic_event["topic"] == "battery_status"
    assert topic_event["data"] == "low"
    assert topic_event["time"] == 12.0000003

    service_event = monitor._event(service, ServiceRequest(), "request")
    assert service_event["service"] == "set_led"
    assert service_event["request"] == {"data": False}
    assert monitor._event_payload_for_status(monitor._wire_event(service_event)) == {"data": False}
    service_response_event = monitor._event(service, ServiceRequest(), "response")
    assert service_response_event["service"] == "set_led"
    assert service_response_event["response"] == {"data": False}
    assert monitor._event_payload_for_status(monitor._wire_event(service_response_event)) == {"data": False}

    wire_event = monitor._wire_event(topic_event)
    assert wire_event["topic"] == "battery_status"
    assert not any(key.startswith("__") for key in wire_event)


def test_rendered_monitor_oracle_verdicts_and_filtering_are_legacy_compatible(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path)
    monitor = module.ROSMonitor_battery_guard.__new__(module.ROSMonitor_battery_guard)
    monitor.lock = module.threading.RLock()
    monitor._log_warning = lambda message: None
    monitor.status_file = (tmp_path / "status.jsonl").open("w+", encoding="utf-8")
    monitor.log_file = (tmp_path / "events.jsonl").open("w+", encoding="utf-8")
    published_verdicts = []

    class VerdictPublisher:
        def publish(self, message):
            published_verdicts.append(message.data)

    monitor.verdict_publisher = VerdictPublisher()

    class Oracle:
        def __init__(self):
            self.sent = []

        def send(self, message):
            self.sent.append(json.loads(message))

        def recv(self):
            return json.dumps({"verdict": "currently_false"})

    monitor.oracle = Oracle()
    interface = next(item for item in module.INTERFACES if item["kind"] == "topic")
    event = {
        "__monitor": module.MONITOR_ID,
        "__interface": interface["name"],
        "__kind": "topic",
        "__action": "filter",
        "topic": "battery_status",
        "time": 1.0,
        "data": "low",
    }

    try:
        assert monitor._handle_event(interface, event, object()) is False
    finally:
        monitor.log_file.close()
        monitor.status_file.close()

    sent = monitor.oracle.sent[-1]
    assert sent == {"topic": "battery_status", "time": 1.0, "data": "low"}
    events = [json.loads(line) for line in (tmp_path / "events.jsonl").read_text(encoding="utf-8").splitlines()]
    statuses = [json.loads(line) for line in (tmp_path / "status.jsonl").read_text(encoding="utf-8").splitlines()]
    assert events == [sent]
    assert statuses[-1]["verdict"] is False
    assert statuses[-1]["verdict_raw"] == "currently_false"
    assert statuses[-1]["action"] == "filter"
    assert statuses[-1]["blocked"] is True
    assert statuses[-1]["communication_allowed"] is False
    assert statuses[-1]["decision"] == "blocked"
    assert statuses[-1]["oracle_response"] == {"verdict": "currently_false"}
    assert statuses[-1]["interface"] == "/battery_status"
    assert statuses[-1]["direction"] == "message"
    assert statuses[-1]["payload"] == "low"
    assert statuses[-1]["event"] == sent
    assert published_verdicts == ["currently_false"]


def test_rendered_monitor_publishes_legacy_verdict_topic_payloads(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path)
    monitor = module.ROSMonitor_battery_guard.__new__(module.ROSMonitor_battery_guard)
    published = []

    class Publisher:
        def publish(self, message):
            published.append(message.data)

    monitor.verdict_publisher = Publisher()

    monitor._publish_verdict("currently_true")
    monitor._publish_verdict(False)
    monitor.verdict_publisher = None
    monitor._publish_verdict("false")

    assert published == ["currently_true", "false"]


def test_rendered_monitor_allows_and_reports_unknown_when_oracle_fails(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path)
    monitor = module.ROSMonitor_battery_guard.__new__(module.ROSMonitor_battery_guard)
    monitor.lock = module.threading.RLock()
    monitor._log_warning = lambda message: None
    monitor.status_file = (tmp_path / "status_oracle_error.jsonl").open("w+", encoding="utf-8")
    monitor.log_file = (tmp_path / "events_oracle_error.jsonl").open("w+", encoding="utf-8")
    published = []

    class VerdictPublisher:
        def publish(self, message):
            published.append(message.data)

    class BrokenOracle:
        def send(self, message):
            raise RuntimeError("oracle down")

    monitor.verdict_publisher = VerdictPublisher()
    monitor.oracle = BrokenOracle()
    interface = next(item for item in module.INTERFACES if item["kind"] == "topic")
    event = {
        "__monitor": module.MONITOR_ID,
        "__interface": interface["name"],
        "__kind": "topic",
        "__action": "filter",
        "topic": "battery_status",
        "time": 1.0,
        "data": "should-pass-on-unknown",
    }

    try:
        assert monitor._handle_event(interface, event, object()) is True
    finally:
        monitor.log_file.close()
        monitor.status_file.close()

    statuses = [
        json.loads(line)
        for line in (tmp_path / "status_oracle_error.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert statuses[-2]["status"] == "oracle_error"
    assert statuses[-1]["verdict_raw"] == "unknown"
    assert statuses[-1]["verdict"] is True
    assert statuses[-1]["blocked"] is False
    assert statuses[-1]["communication_allowed"] is True
    assert statuses[-1]["decision"] == "forwarded"
    assert statuses[-1]["terminal"] is False
    assert published == ["unknown"]


def test_rendered_monitor_accepts_raw_string_oracle_responses(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path)
    monitor = module.ROSMonitor_battery_guard.__new__(module.ROSMonitor_battery_guard)
    monitor.lock = module.threading.RLock()
    monitor._log_warning = lambda message: None
    monitor.status_file = (tmp_path / "status_raw_string.jsonl").open("w+", encoding="utf-8")
    monitor.log_file = (tmp_path / "events_raw_string.jsonl").open("w+", encoding="utf-8")

    class Oracle:
        def send(self, message):
            return None

        def recv(self):
            return "violated"

    monitor.verdict_publisher = type("Publisher", (), {"publish": lambda self, message: None})()
    monitor.oracle = Oracle()
    interface = next(item for item in module.INTERFACES if item["kind"] == "topic")
    event = {
        "__monitor": module.MONITOR_ID,
        "__interface": interface["name"],
        "__kind": "topic",
        "__action": "filter",
        "topic": "battery_status",
        "time": 1.0,
        "data": "bad",
    }

    try:
        assert monitor._handle_event(interface, event, object()) is False
    finally:
        monitor.log_file.close()
        monitor.status_file.close()

    statuses = [
        json.loads(line)
        for line in (tmp_path / "status_raw_string.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert statuses[-1]["verdict_raw"] == "violated"
    assert statuses[-1]["verdict"] is False
    assert statuses[-1]["blocked"] is True
    assert statuses[-1]["communication_allowed"] is False
    assert statuses[-1]["decision"] == "blocked"
    assert statuses[-1]["terminal"] is False
    assert statuses[-1]["will_stop"] is False


def test_rendered_monitor_warning_level_distinguishes_current_and_final_false(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path)
    monitor = module.ROSMonitor_battery_guard.__new__(module.ROSMonitor_battery_guard)

    warnings = []
    monitor._log_warning = warnings.append

    module.WARNING = 1
    assert monitor._should_report_violation("currently_false") is True
    assert monitor._should_report_violation("false") is True

    module.WARNING = 2
    assert monitor._should_report_violation("currently_false") is False
    assert monitor._should_report_violation("false") is True

    module.WARNING = 0
    assert monitor._should_report_violation("false") is False

    assert monitor._is_negative_verdict("currently_false") is True
    assert monitor._is_negative_verdict("false") is True
    assert monitor._is_negative_verdict("currently_true") is False
    assert monitor._is_negative_verdict("unknown") is False


def test_passive_rendered_monitor_stops_only_on_definitive_verdicts(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path, config=_passive_config())
    interface = module.INTERFACES[0]

    class Oracle:
        def __init__(self, verdict):
            self.verdict = verdict

        def send(self, message):
            return None

        def recv(self):
            return json.dumps({"verdict": self.verdict})

    def run(verdict):
        monitor = module.ROSMonitor_passive_guard.__new__(module.ROSMonitor_passive_guard)
        monitor.lock = module.threading.RLock()
        monitor._log_warning = lambda message: None
        monitor._log_info = lambda message: None
        stopped = []
        monitor._shutdown_runtime = lambda: stopped.append(verdict)
        monitor.status_file = (tmp_path / f"status_{verdict}.jsonl").open("w+", encoding="utf-8")
        monitor.log_file = (tmp_path / f"events_{verdict}.jsonl").open("w+", encoding="utf-8")
        monitor.oracle = Oracle(verdict)
        event = {
            "__monitor": module.MONITOR_ID,
            "__interface": interface["name"],
            "__kind": "topic",
            "__action": "log",
            "topic": "passive_text",
            "time": 1.0,
            "data": verdict,
        }
        try:
            allowed = monitor._handle_event(interface, event, object())
        finally:
            monitor.log_file.close()
            monitor.status_file.close()
        statuses = [
            json.loads(line)
            for line in (tmp_path / f"status_{verdict}.jsonl").read_text(encoding="utf-8").splitlines()
        ]
        return allowed, stopped, statuses

    for verdict in ("currently_true", "currently_false"):
        allowed, stopped, statuses = run(verdict)
        assert allowed is True
        assert stopped == []
        assert statuses[-1]["status"] == "event"
        assert statuses[-1]["action"] == "log"
        assert statuses[-1]["blocked"] is False
        assert statuses[-1]["communication_allowed"] is True
        assert statuses[-1]["decision"] == "logged"
        assert statuses[-1]["terminal"] is False

    for verdict in ("true", "false"):
        allowed, stopped, statuses = run(verdict)
        assert allowed is True
        assert stopped == [verdict]
        assert statuses[-1]["status"] == "event"
        assert statuses[-1]["blocked"] is False
        assert statuses[-1]["communication_allowed"] is True
        assert statuses[-1]["decision"] == "logged"
        assert statuses[-1]["terminal"] is True
        assert statuses[-1]["will_stop"] is True


def test_offline_passive_rendered_monitor_does_not_treat_no_oracle_as_definitive_true(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path, config=_offline_passive_config())
    monitor = module.ROSMonitor_offline_passive_log.__new__(module.ROSMonitor_offline_passive_log)
    interface = module.INTERFACES[0]

    stopped = []
    monitor.lock = module.threading.RLock()
    monitor._log_warning = lambda message: None
    monitor._log_info = lambda message: None
    monitor._shutdown_runtime = lambda: stopped.append("stopped")
    monitor.status_file = (tmp_path / "offline_status.jsonl").open("w+", encoding="utf-8")
    monitor.log_file = (tmp_path / "offline_events.jsonl").open("w+", encoding="utf-8")
    monitor.oracle = None
    event = {
        "__monitor": module.MONITOR_ID,
        "__interface": interface["name"],
        "__kind": "topic",
        "__action": "log",
        "topic": "offline_text",
        "time": 1.0,
        "data": "observed",
    }

    try:
        assert monitor._handle_event(interface, event, object()) is True
    finally:
        monitor.log_file.close()
        monitor.status_file.close()

    statuses = [
        json.loads(line)
        for line in (tmp_path / "offline_status.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert stopped == []
    assert statuses[-1]["status"] == "event"
    assert statuses[-1]["verdict_raw"] == "unknown"
    assert statuses[-1]["terminal"] is False


def test_status_disabled_rendered_monitor_runs_without_status_file(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path, config=_status_disabled_config())
    monitor = module.ROSMonitor_status_disabled_guard.__new__(module.ROSMonitor_status_disabled_guard)
    interface = module.INTERFACES[0]
    monitor.lock = module.threading.RLock()
    monitor._log_warning = lambda message: None
    monitor._log_info = lambda message: None
    monitor.status_file = None
    monitor.log_file = (tmp_path / "statusless_events.jsonl").open("w+", encoding="utf-8")
    monitor.oracle = None
    event = {
        "__monitor": module.MONITOR_ID,
        "__interface": interface["name"],
        "__kind": "topic",
        "__action": "log",
        "topic": "statusless",
        "time": 1.0,
        "data": "ok",
    }

    try:
        assert monitor._handle_event(interface, event, object()) is True
    finally:
        monitor.log_file.close()

    rows = [
        json.loads(line)
        for line in (tmp_path / "statusless_events.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert rows == [{"data": "ok", "time": 1.0, "topic": "statusless"}]


def test_intercepting_rendered_monitor_records_terminal_verdict_without_stopping(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path)
    monitor = module.ROSMonitor_battery_guard.__new__(module.ROSMonitor_battery_guard)
    interface = next(item for item in module.INTERFACES if item["kind"] == "topic")

    class Oracle:
        def send(self, message):
            return None

        def recv(self):
            return json.dumps({"verdict": "false"})

    stopped = []
    monitor.lock = module.threading.RLock()
    monitor._log_warning = lambda message: None
    monitor._log_info = lambda message: None
    monitor._shutdown_runtime = lambda: stopped.append("stopped")
    monitor.status_file = (tmp_path / "status_intercepting.jsonl").open("w+", encoding="utf-8")
    monitor.log_file = (tmp_path / "events_intercepting.jsonl").open("w+", encoding="utf-8")
    monitor.oracle = Oracle()
    event = {
        "__monitor": module.MONITOR_ID,
        "__interface": interface["name"],
        "__kind": "topic",
        "__action": "filter",
        "topic": "battery_status",
        "time": 1.0,
        "data": "bad",
    }

    try:
        assert monitor._handle_event(interface, event, object()) is False
    finally:
        monitor.log_file.close()
        monitor.status_file.close()

    statuses = [
        json.loads(line)
        for line in (tmp_path / "status_intercepting.jsonl").read_text(encoding="utf-8").splitlines()
    ]
    assert stopped == []
    assert len(statuses) == 1
    assert statuses[-1]["terminal"] is True
    assert statuses[-1]["will_stop"] is False


def test_rendered_monitor_ordered_buffer_and_dashboard_payload(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path)
    module.LOG_PATH = str(tmp_path / "events.jsonl")
    module.STATUS_LOG_PATH = str(tmp_path / "status.jsonl")
    Path(module.LOG_PATH).write_text(
        "\n".join(
            [
                json.dumps({"topic": "battery_status", "time": 1.0, "data": "first"}),
                json.dumps({"topic": "battery_status", "time": 2.0, "data": "second"}),
            ]
        ),
        encoding="utf-8",
    )
    Path(module.STATUS_LOG_PATH).write_text(
        json.dumps({"monitor": module.MONITOR_ID, "status": "event", "interface": "/battery_status", "verdict": True})
        + "\n",
        encoding="utf-8",
    )

    buffer = module.OrderedEventBuffer(max_delay_ms=5000)
    later = module.OrderedDecision(False)
    earlier = module.OrderedDecision(False)
    buffer.push({"__source_time": 10.0, "value": "later"}, "later-payload", later)
    buffer.push({"__source_time": 1.0, "value": "earlier"}, "earlier-payload", earlier)
    assert buffer.flush_ready() == [({"__source_time": 1.0, "value": "earlier"}, "earlier-payload", earlier)]
    assert buffer.flush_all() == [({"__source_time": 10.0, "value": "later"}, "later-payload", later)]

    args, ros_args = module.parse_monitor_args(["--dashboard", "--dashboard-port", "9999", "--ros-args"])
    assert args.dashboard is True
    assert args.dashboard_port == 9999
    assert args.fresh_session is False
    assert ros_args == ["--ros-args"]

    payload = module.dashboard_payload()
    assert payload["monitor_config"]["id"] == "battery_guard"
    assert payload["monitor_config"]["interfaces"][0]["ordered"] is True
    assert payload["recent_events"][-1]["data"] == "second"
    assert payload["status"]["events"] == 1


def test_rendered_monitor_globally_orders_topics_and_services(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path, config=_ordered_multi_config())
    monitor = module.ROSMonitor_ordered_multi_guard.__new__(module.ROSMonitor_ordered_multi_guard)
    events_path, _ = _prepare_rendered_monitor(module, monitor, tmp_path, "global")
    monitor.oracle = None

    alpha = next(item for item in module.INTERFACES if item["legacy_name"] == "alpha")
    beta = next(item for item in module.INTERFACES if item["legacy_name"] == "beta")
    service = next(item for item in module.INTERFACES if item["kind"] == "service")

    try:
        assert monitor._handle_or_buffer(alpha, {"stamp": {"sec": 20}, "data": "topic-later"}) is True
        assert monitor._handle_or_buffer(service, {"stamp": {"sec": 15}, "data": True}, "request") is True
        assert monitor._handle_or_buffer(beta, {"stamp": {"sec": 10}, "data": "topic-earlier"}) is True
        monitor.close()
    finally:
        monitor.log_file.close()
        monitor.status_file.close()

    rows = [json.loads(line) for line in events_path.read_text(encoding="utf-8").splitlines()]
    assert [(row.get("topic"), row.get("service"), row["time"]) for row in rows] == [
        ("beta", None, 10.0),
        (None, "set_led", 15.0),
        ("alpha", None, 20.0),
    ]


def test_rendered_monitor_ordered_filter_waits_for_own_verdict_before_allowing(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path, config=_ordered_filter_config())
    monitor = module.ROSMonitor_ordered_filter_guard.__new__(module.ROSMonitor_ordered_filter_guard)
    events_path, status_path = _prepare_rendered_monitor(module, monitor, tmp_path, "filter")

    class Oracle:
        def send(self, message):
            return None

        def recv(self):
            return json.dumps({"verdict": "currently_false"})

    monitor.oracle = Oracle()
    guarded = next(item for item in module.INTERFACES if item["legacy_name"] == "guarded")

    try:
        assert monitor._handle_or_buffer(guarded, {"stamp": {"sec": 1}, "data": "bad"}) is False
    finally:
        monitor.log_file.close()
        monitor.status_file.close()

    rows = [json.loads(line) for line in events_path.read_text(encoding="utf-8").splitlines()]
    statuses = [json.loads(line) for line in status_path.read_text(encoding="utf-8").splitlines()]
    assert rows == [{"data": "bad", "stamp": {"sec": 1}, "time": 1.0, "topic": "guarded"}]
    assert statuses[-1]["verdict"] is False
    assert statuses[-1]["verdict_raw"] == "currently_false"


def test_rendered_monitor_ordered_service_filter_waits_for_request_verdict(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path, config=_ordered_service_filter_config())
    monitor = module.ROSMonitor_ordered_service_filter_guard.__new__(module.ROSMonitor_ordered_service_filter_guard)
    events_path, status_path = _prepare_rendered_monitor(module, monitor, tmp_path, "service_filter")

    class Oracle:
        def send(self, message):
            return None

        def recv(self):
            return json.dumps({"verdict": "false"})

    monitor.oracle = Oracle()
    service = module.INTERFACES[0]

    try:
        assert monitor._handle_or_buffer(service, {"stamp": {"sec": 4}, "data": False}, "request") is False
    finally:
        monitor.log_file.close()
        monitor.status_file.close()

    rows = [json.loads(line) for line in events_path.read_text(encoding="utf-8").splitlines()]
    statuses = [json.loads(line) for line in status_path.read_text(encoding="utf-8").splitlines()]
    assert rows == [{"request": {"data": False, "stamp": {"sec": 4}}, "service": "set_led", "stamp": {"sec": 4}, "time": 4.0}]
    assert statuses[-1]["kind"] == "service"
    assert statuses[-1]["direction"] == "request"
    assert statuses[-1]["verdict"] is False
    assert statuses[-1]["terminal"] is True
    assert statuses[-1]["will_stop"] is False


def test_rendered_monitor_ordered_topic_and_service_can_share_ros_name(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path, config=_same_name_topic_service_config())
    monitor = module.ROSMonitor_same_name_guard.__new__(module.ROSMonitor_same_name_guard)
    events_path, _ = _prepare_rendered_monitor(module, monitor, tmp_path, "same_name")
    monitor.oracle = None

    topic = next(item for item in module.INTERFACES if item["kind"] == "topic")
    service = next(item for item in module.INTERFACES if item["kind"] == "service")

    assert topic["name"] == service["name"] == "/shared"
    assert topic["key"] == "topic:/shared"
    assert service["key"] == "service:/shared"

    try:
        assert monitor._handle_or_buffer(topic, {"stamp": {"sec": 20}, "data": "topic"}) is True
        assert monitor._handle_or_buffer(service, {"stamp": {"sec": 10}, "data": True}, "request") is True
        monitor.close()
    finally:
        monitor.log_file.close()
        monitor.status_file.close()

    rows = [json.loads(line) for line in events_path.read_text(encoding="utf-8").splitlines()]
    assert [(row.get("topic"), row.get("service"), row["time"]) for row in rows] == [
        (None, "shared", 10.0),
        ("shared", None, 20.0),
    ]


def test_rendered_monitor_fresh_session_clears_logs_and_records_session(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path)
    module.LOG_PATH = str(tmp_path / "events.jsonl")
    module.STATUS_LOG_PATH = str(tmp_path / "status.jsonl")
    Path(module.LOG_PATH).write_text("old-event\n", encoding="utf-8")
    Path(module.STATUS_LOG_PATH).write_text("old-status\n", encoding="utf-8")

    monitor = module.ROSMonitor_battery_guard.__new__(module.ROSMonitor_battery_guard)
    monitor.session_id = "session-a"
    monitor._open_files(fresh_session=True)
    try:
        monitor._publish_status("started")
    finally:
        monitor.log_file.close()
        monitor.status_file.close()

    assert Path(module.LOG_PATH).read_text(encoding="utf-8") == ""
    status_rows = [json.loads(line) for line in Path(module.STATUS_LOG_PATH).read_text(encoding="utf-8").splitlines()]
    assert status_rows == [
        {"monitor": "battery_guard", "ros_version": "ros2", "session_id": "session-a", "status": "started", "time": status_rows[0]["time"]}
    ]

    Path(module.STATUS_LOG_PATH).write_text("kept-status\n", encoding="utf-8")
    second = module.ROSMonitor_battery_guard.__new__(module.ROSMonitor_battery_guard)
    second.session_id = "session-a"
    second._open_files(fresh_session=True)
    second.log_file.close()
    second.status_file.close()
    assert Path(module.STATUS_LOG_PATH).read_text(encoding="utf-8") == "kept-status\n"


def test_rendered_monitor_fresh_session_args_can_come_from_env(tmp_path: Path, monkeypatch):
    module = _load_rendered_monitor(tmp_path)
    monkeypatch.setenv("ROSMONITORING_FRESH_SESSION", "1")
    monkeypatch.setenv("ROSMONITORING_SESSION_ID", "example-session")

    args, _ = module.parse_monitor_args([])

    assert args.fresh_session is True
    assert args.session_id == "example-session"


def test_rendered_ros1_monitor_is_valid_python_and_mentions_services():
    monitor = _config().monitors[0]
    source = render_monitor_source(monitor, "ros1")
    ast.parse(source)
    assert "rospy.Service" in source
    assert "rospy.Publisher" in source
    assert "--dashboard" in source
    assert "dashboard_payload" in source


def test_rendered_ros1_monitor_imports_without_rospy_for_static_regression(tmp_path: Path):
    module = _load_rendered_monitor(tmp_path, ros_version="ros1")
    monitor = module.ROSMonitor_battery_guard.__new__(module.ROSMonitor_battery_guard)
    topic = next(item for item in module.INTERFACES if item["kind"] == "topic")

    event = monitor._event(topic, {"data": "hello", "stamp": {"secs": 4, "nsecs": 5}})

    assert event["topic"] == "battery_status"
    assert event["data"] == "hello"
    assert event["time"] == 4.000000005
    assert module.parse_monitor_args(["--dashboard"])[0].dashboard is True


def test_generate_ros2_project_files(tmp_path: Path):
    written = generate_project(_config(), tmp_path, ros_version="ros2")
    rel = {path.relative_to(tmp_path).as_posix() for path in written}
    assert "monitor/package.xml" in rel
    assert "monitor/monitor/battery_guard.py" in rel
    assert "monitor/launch/monitor.launch.py" in rel
    package_xml = (tmp_path / "monitor" / "package.xml").read_text(encoding="utf-8")
    launch = (tmp_path / "monitor" / "launch" / "monitor.launch.py").read_text(encoding="utf-8")
    assert "<build_type>ament_python</build_type>" in package_xml
    assert "<exec_depend>std_msgs</exec_depend>" in package_xml
    assert "<exec_depend>example_interfaces</exec_depend>" in package_xml
    assert "dashboard_monitor" in launch
    assert "--dashboard" in launch
    assert "OpaqueFunction" in launch


def test_generate_ros2_project_removes_stale_monitor_sources(tmp_path: Path):
    generate_project(_config(), tmp_path, ros_version="ros2")
    stale = tmp_path / "monitor" / "monitor" / "old_guard.py"
    stale.write_text("stale", encoding="utf-8")

    generate_project(_config(), tmp_path, ros_version="ros2")

    assert not stale.exists()
    assert (tmp_path / "monitor" / "monitor" / "battery_guard.py").is_file()
    assert (tmp_path / "monitor" / "monitor" / "__init__.py").is_file()


def test_generate_ros2_project_updates_multi_monitor_entry_points_and_stale_sources(tmp_path: Path):
    generate_project(_two_monitor_config(), tmp_path, ros_version="ros2")
    setup = (tmp_path / "monitor" / "setup.py").read_text(encoding="utf-8")
    assert "first_guard = monitor.first_guard:main" in setup
    assert "second_guard = monitor.second_guard:main" in setup

    stale = tmp_path / "monitor" / "monitor" / "third_guard.py"
    stale.write_text("stale", encoding="utf-8")
    generate_project(_two_monitor_config(), tmp_path, ros_version="ros2")

    assert not stale.exists()
    assert (tmp_path / "monitor" / "monitor" / "first_guard.py").is_file()
    assert (tmp_path / "monitor" / "monitor" / "second_guard.py").is_file()


def test_generate_ros1_project_files(tmp_path: Path):
    written = generate_project(_config(), tmp_path, ros_version="ros1")
    rel = {path.relative_to(tmp_path).as_posix() for path in written}
    assert "monitor/package.xml" in rel
    assert "monitor/src/battery_guard.py" in rel
    assert "monitor/run_battery_guard.launch" in rel
    assert (tmp_path / "monitor" / "src" / "battery_guard.py").stat().st_mode & 0o111
    cmake = (tmp_path / "monitor" / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "catkin_install_python" in cmake
    assert "src/battery_guard.py" in cmake
    assert "rospy std_msgs" in cmake
    package_xml = (tmp_path / "monitor" / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>std_msgs</exec_depend>" in package_xml


def test_generate_ros1_project_removes_stale_monitor_sources(tmp_path: Path):
    generate_project(_config(), tmp_path, ros_version="ros1")
    stale = tmp_path / "monitor" / "src" / "old_guard.py"
    stale.write_text("stale", encoding="utf-8")

    generate_project(_config(), tmp_path, ros_version="ros1")

    assert not stale.exists()
    assert (tmp_path / "monitor" / "src" / "battery_guard.py").is_file()


def test_generate_ros1_project_updates_multi_monitor_install_and_launch_files(tmp_path: Path):
    generate_project(_two_monitor_config(), tmp_path, ros_version="ros1")
    cmake = (tmp_path / "monitor" / "CMakeLists.txt").read_text(encoding="utf-8")
    assert "src/first_guard.py" in cmake
    assert "src/second_guard.py" in cmake
    assert (tmp_path / "monitor" / "run_first_guard.launch").is_file()
    assert (tmp_path / "monitor" / "run_second_guard.launch").is_file()

    stale = tmp_path / "monitor" / "src" / "third_guard.py"
    stale.write_text("stale", encoding="utf-8")
    generate_project(_two_monitor_config(), tmp_path, ros_version="ros1")

    assert not stale.exists()
