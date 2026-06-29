from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import pprint
import textwrap
from typing import Iterable

from .config import InterfaceSpec, MonitorSpec, ProjectConfig
from .status import HTML as DASHBOARD_HTML


@dataclass(frozen=True)
class GeneratedFile:
    path: Path
    content: str


def generate_project(config: ProjectConfig, output_dir: str | Path, ros_version: str | None = None) -> list[Path]:
    selected_ros_version = (ros_version or config.ros_version or "ros2").lower()
    if selected_ros_version not in {"ros1", "ros2"}:
        raise ValueError("ros_version must be ros1 or ros2")
    output_path = Path(output_dir)
    files = render_project(config, selected_ros_version)
    _remove_stale_generated_monitors(output_path, files, selected_ros_version)
    written: list[Path] = []
    for generated in files:
        target = output_path / generated.path
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(generated.content, encoding="utf-8")
        if selected_ros_version == "ros1" and target.suffix == ".py" and "monitor/src" in target.as_posix():
            target.chmod(0o755)
        written.append(target)
    return written


def _remove_stale_generated_monitors(output_path: Path, files: list[GeneratedFile], ros_version: str) -> None:
    expected = {generated.path for generated in files}
    if ros_version == "ros2":
        monitor_dir = output_path / "monitor" / "monitor"
        pattern = "*.py"
        keep = {Path("monitor/monitor/__init__.py")}
    else:
        monitor_dir = output_path / "monitor" / "src"
        pattern = "*.py"
        keep = set()
    if not monitor_dir.exists():
        return
    for path in monitor_dir.glob(pattern):
        relative = path.relative_to(output_path)
        if relative in keep:
            continue
        if relative not in expected:
            path.unlink()


def render_project(config: ProjectConfig, ros_version: str) -> list[GeneratedFile]:
    monitors = [_monitor_file(monitor, ros_version) for monitor in config.monitors]
    if ros_version == "ros2":
        return [
            GeneratedFile(Path("monitor/package.xml"), _ros2_package_xml(config.monitors)),
            GeneratedFile(Path("monitor/setup.py"), _ros2_setup(config.monitors)),
            GeneratedFile(Path("monitor/setup.cfg"), "[develop]\nscript_dir=$base/lib/monitor\n[install]\ninstall_scripts=$base/lib/monitor\n"),
            GeneratedFile(Path("monitor/resource/monitor"), ""),
            GeneratedFile(Path("monitor/monitor/__init__.py"), ""),
            GeneratedFile(Path("monitor/launch/monitor.launch.py"), _ros2_launch(config.monitors)),
            *monitors,
        ]
    return [
        GeneratedFile(Path("monitor/package.xml"), _ros1_package_xml()),
        GeneratedFile(Path("monitor/CMakeLists.txt"), _ros1_cmakelists(config.monitors)),
        *[_ros1_launch_file(monitor) for monitor in config.monitors],
        *monitors,
    ]


def _monitor_file(monitor: MonitorSpec, ros_version: str) -> GeneratedFile:
    if ros_version == "ros2":
        return GeneratedFile(Path(f"monitor/monitor/{monitor.id}.py"), render_monitor_source(monitor, ros_version))
    return GeneratedFile(Path(f"monitor/src/{monitor.id}.py"), render_monitor_source(monitor, ros_version))


def render_monitor_source(monitor: MonitorSpec, ros_version: str) -> str:
    imports = _message_imports(monitor.interfaces)
    interfaces_literal = pprint.pformat([_interface_dict(interface) for interface in monitor.interfaces], width=100)
    oracle_literal = "None"
    if monitor.oracle:
        oracle_literal = repr(
            {
                "url": monitor.oracle.websocket_url,
                "action": monitor.oracle.action,
                "timeout": monitor.oracle.timeout,
            }
        )
    class_name = f"ROSMonitor_{monitor.id}"
    if ros_version == "ros2":
        runtime = _ros2_runtime(class_name)
    else:
        runtime = _ros1_runtime(class_name)
    source = f'''#!/usr/bin/env python3
"""Generated ROSMonitoring monitor: {monitor.id}.

This file is generated. Edit the YAML configuration and regenerate instead of
editing this monitor by hand.
"""

from __future__ import annotations

import argparse
try:
    import fcntl
except Exception:
    fcntl = None
import heapq
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import itertools
import json
import os
from pathlib import Path
import threading
import time
from typing import Any
from urllib.parse import urlparse

try:
    import websocket
except Exception:
    websocket = None

{imports}
try:
    from std_msgs.msg import String as ROSMonitoringVerdictString
except Exception:
    class ROSMonitoringVerdictString:
        def __init__(self):
            self.data = ""

ROS_VERSION = {ros_version!r}
MONITOR_ID = {monitor.id!r}
LOG_PATH = {monitor.log!r}
STATUS_LOG_PATH = {monitor.status.log!r}
STATUS_ENABLED = {monitor.status.enabled!r}
SILENT = {monitor.silent!r}
WARNING = {monitor.warning!r}
ORACLE = {oracle_literal}
INTERFACES = {interfaces_literal}
DASHBOARD_HTML = {DASHBOARD_HTML!r}


def message_to_dict(message: Any) -> Any:
    if message is None or isinstance(message, (str, int, float, bool)):
        return message
    if isinstance(message, bytes):
        return message.decode("utf-8", errors="replace")
    if isinstance(message, (list, tuple)):
        return [message_to_dict(item) for item in message]
    if isinstance(message, dict):
        return {{str(key): message_to_dict(value) for key, value in message.items()}}
    slots = getattr(message, "__slots__", None)
    if slots:
        return {{slot.lstrip("_"): message_to_dict(getattr(message, slot)) for slot in slots if hasattr(message, slot)}}
    if hasattr(message, "__dict__"):
        return {{str(key): message_to_dict(value) for key, value in vars(message).items() if not key.startswith("_")}}
    return str(message)


def stamp_to_float(stamp: Any) -> float | None:
    if stamp is None:
        return None
    if isinstance(stamp, (int, float)):
        return float(stamp)
    sec = getattr(stamp, "sec", getattr(stamp, "secs", None))
    nanosec = getattr(stamp, "nanosec", getattr(stamp, "nsecs", None))
    if sec is not None:
        return float(sec) + (float(nanosec or 0) / 1000000000.0)
    if isinstance(stamp, dict):
        sec = stamp.get("sec", stamp.get("secs"))
        nanosec = stamp.get("nanosec", stamp.get("nsecs", 0))
        if sec is not None:
            return float(sec) + (float(nanosec or 0) / 1000000000.0)
    return None


def source_time(payload: Any, fallback: float | None = None) -> float:
    fallback = time.time() if fallback is None else fallback
    header = getattr(payload, "header", None)
    if header is not None:
        value = stamp_to_float(getattr(header, "stamp", None))
        if value is not None:
            return value
    direct = stamp_to_float(getattr(payload, "stamp", None))
    if direct is not None:
        return direct
    if isinstance(payload, dict):
        header = payload.get("header")
        if isinstance(header, dict):
            value = stamp_to_float(header.get("stamp"))
            if value is not None:
                return value
        value = stamp_to_float(payload.get("stamp"))
        if value is not None:
            return value
    return fallback


class OrderedDecision:
    def __init__(self, requires_decision: bool):
        self.requires_decision = requires_decision
        self.condition = threading.Condition()
        self.done = False
        self.allowed = True

    def set_result(self, allowed: bool):
        with self.condition:
            self.allowed = allowed
            self.done = True
            self.condition.notify_all()

    def wait(self, timeout: float | None = None) -> bool:
        with self.condition:
            if not self.done:
                self.condition.wait(timeout=timeout)
            return self.done


class OrderedEventBuffer:
    def __init__(self, max_delay_ms: int = 0):
        self.max_delay = max(0, max_delay_ms) / 1000.0
        self.sequence = itertools.count()
        self.heap = []
        self.watermark = None

    def push(self, event: dict[str, Any], payload: Any, decision: OrderedDecision) -> OrderedDecision:
        event_time = float(event["__source_time"])
        if self.watermark is None or event_time > self.watermark:
            self.watermark = event_time
        heapq.heappush(self.heap, (event_time, next(self.sequence), event, payload, decision))
        return decision

    def flush_ready(self) -> list[tuple[dict[str, Any], Any, OrderedDecision]]:
        if self.max_delay == 0:
            return self.flush_all()
        if self.watermark is None:
            return []
        threshold = self.watermark - self.max_delay
        return self._pop_while(lambda event_time, event, decision: event_time <= threshold)

    def flush_expired(self) -> list[tuple[dict[str, Any], Any, OrderedDecision]]:
        if self.max_delay == 0:
            return self.flush_all()
        now = time.monotonic()
        return self._pop_while(
            lambda event_time, event, decision: now - float(event.get("__buffered_at", now)) >= self.max_delay
        )

    def flush_through(self, decision: OrderedDecision) -> list[tuple[dict[str, Any], Any, OrderedDecision]]:
        target_time = None
        for event_time, _, _, _, item_decision in self.heap:
            if item_decision is decision:
                target_time = event_time
                break
        if target_time is None:
            return []
        return self._pop_while(lambda event_time, event, item_decision: event_time <= target_time)

    def flush_all(self) -> list[tuple[dict[str, Any], Any, OrderedDecision]]:
        return self._pop_while(lambda event_time, event, decision: True)

    def _pop_while(self, predicate):
        ready = []
        while self.heap and predicate(self.heap[0][0], self.heap[0][2], self.heap[0][4]):
            _, _, event, payload, decision = heapq.heappop(self.heap)
            ready.append((event, payload, decision))
        return ready


def read_status_log(log_path: str) -> dict[str, Any]:
    path = Path(log_path)
    monitors: dict[str, dict[str, Any]] = {{}}
    events = 0
    violations = 0
    if not path.exists():
        return {{"monitors": monitors, "events": events, "violations": violations}}
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            monitor_id = item.get("monitor", "unknown")
            monitor = monitors.setdefault(
                monitor_id,
                {{"status": "unknown", "last_seen": None, "events": 0, "violations": 0, "interfaces": {{}}}},
            )
            monitor["status"] = item.get("status", monitor["status"])
            monitor["last_seen"] = item.get("time", monitor["last_seen"])
            if item.get("status") == "event":
                events += 1
                monitor["events"] += 1
                interface = item.get("interface", "unknown")
                monitor["interfaces"][interface] = monitor["interfaces"].get(interface, 0) + 1
                if item.get("verdict") is False:
                    violations += 1
                    monitor["violations"] += 1
    return {{"monitors": monitors, "events": events, "violations": violations}}


def read_jsonl_tail(log_path: str, limit: int = 200) -> list[dict[str, Any]]:
    path = Path(log_path)
    if not path.exists():
        return []
    rows = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows[-limit:]


def monitor_event_log_paths(status_rows: list[dict[str, Any]]) -> dict[str, str]:
    paths = {{}}
    for item in status_rows:
        monitor = item.get("monitor")
        log_path = item.get("log_path")
        if monitor and log_path:
            paths[str(monitor)] = str(log_path)
    return paths


def read_monitor_events(status_rows: list[dict[str, Any]], default_monitor: str, default_log_path: str) -> dict[str, list[dict[str, Any]]]:
    paths = monitor_event_log_paths(status_rows)
    paths.setdefault(default_monitor, default_log_path)
    events = {{}}
    for monitor, path in sorted(paths.items()):
        rows = []
        for event in read_jsonl_tail(path):
            annotated = dict(event)
            annotated["_monitor"] = monitor
            rows.append(annotated)
        events[monitor] = rows
    return events


def monitor_configs_from_status_rows(status_rows: list[dict[str, Any]], current_config: dict[str, Any]) -> dict[str, dict[str, Any]]:
    configs = {{str(current_config["id"]): dict(current_config)}}
    for item in status_rows:
        monitor = item.get("monitor")
        if not monitor:
            continue
        config = configs.setdefault(str(monitor), {{"id": str(monitor)}})
        if item.get("ros_version"):
            config["ros_version"] = item["ros_version"]
        if item.get("log_path"):
            config["log_path"] = item["log_path"]
        if item.get("interfaces"):
            config["interfaces"] = item["interfaces"]
    return configs


def dashboard_payload() -> dict[str, Any]:
    recent_status = read_jsonl_tail(STATUS_LOG_PATH)
    events_by_monitor = read_monitor_events(recent_status, MONITOR_ID, LOG_PATH)
    recent_events = []
    for rows in events_by_monitor.values():
        recent_events.extend(rows)
    recent_events.sort(key=lambda item: item.get("time", 0))
    current_config = {{
        "id": MONITOR_ID,
        "ros_version": ROS_VERSION,
        "status_enabled": STATUS_ENABLED,
        "oracle": ORACLE,
        "interfaces": INTERFACES,
        "log_path": LOG_PATH,
        "status_log_path": STATUS_LOG_PATH,
    }}
    return {{
        "status": read_status_log(STATUS_LOG_PATH),
        "status_log": STATUS_LOG_PATH,
        "event_log": LOG_PATH,
        "event_logs": monitor_event_log_paths(recent_status),
        "recent_status": recent_status,
        "recent_events": recent_events[-200:],
        "recent_events_by_monitor": events_by_monitor,
        "monitor_config": current_config,
        "monitor_configs": monitor_configs_from_status_rows(recent_status, current_config),
    }}


def parse_monitor_args(args=None):
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--dashboard",
        action="store_true",
        default=os.environ.get("ROSMONITORING_DASHBOARD", "").lower() in {{"1", "true", "yes", "on"}},
    )
    parser.add_argument("--dashboard-host", default=os.environ.get("ROSMONITORING_DASHBOARD_HOST", "127.0.0.1"))
    parser.add_argument(
        "--dashboard-port",
        type=int,
        default=int(os.environ.get("ROSMONITORING_DASHBOARD_PORT", "8765")),
    )
    parser.add_argument(
        "--fresh-session",
        action="store_true",
        default=os.environ.get("ROSMONITORING_FRESH_SESSION", "").lower() in {{"1", "true", "yes", "on"}},
    )
    parser.add_argument("--session-id", default=os.environ.get("ROSMONITORING_SESSION_ID", str(os.getppid())))
    return parser.parse_known_args(args)


{runtime}

'''
    return source


def _message_imports(interfaces: Iterable[InterfaceSpec]) -> str:
    imports = []
    for package, name in sorted({_split_type(interface.type) for interface in interfaces}):
        imports.append(f"    from {package} import {name}")
    body = "\n".join(imports) if imports else "    pass"
    names = ", ".join(sorted({name for _, name in (_split_type(interface.type) for interface in interfaces)}))
    fallback = "\n".join(f"    {name} = object" for _, name in sorted({_split_type(interface.type) for interface in interfaces}))
    return f"""try:\n{body}\nexcept Exception:\n{fallback or '    pass'}"""


def _split_type(type_name: str) -> tuple[str, str]:
    index = type_name.rfind(".")
    if index < 1:
        raise ValueError(f"invalid ROS type {type_name!r}; expected package.module.Type")
    return type_name[:index], type_name[index + 1 :]


def _interface_dict(interface: InterfaceSpec) -> dict[str, object]:
    _, type_name = _split_type(interface.type)
    return {
        "key": f"{interface.kind}:{interface.normalized_name}",
        "name": interface.normalized_name,
        "legacy_name": interface.name,
        "remapped_name": interface.remapped_name,
        "python_name": interface.python_name,
        "type": type_name,
        "type_fqn": interface.type,
        "kind": interface.kind,
        "action": interface.action,
        "publishers": list(interface.publishers),
        "subscribers": list(interface.subscribers),
        "ordered": interface.ordering.enabled,
        "max_delay_ms": interface.ordering.max_delay_ms,
        "intercepting": interface.is_intercepting,
    }


def _common_monitor_methods(class_name: str, ros2: bool) -> str:
    log_line = "self.get_logger().info(message)" if ros2 else "rospy.loginfo(message)"
    warn_line = "self.get_logger().warning(message)" if ros2 else "rospy.logwarn(message)"
    shutdown_block = (
        '''if rclpy is not None and rclpy.ok():
            rclpy.shutdown()'''
        if ros2
        else '''if rospy is not None:
            rospy.signal_shutdown("terminal oracle verdict")'''
    )
    return f'''
class {class_name}(Node):
    def __init__(
        self,
        dashboard: bool = False,
        dashboard_host: str = "127.0.0.1",
        dashboard_port: int = 8765,
        fresh_session: bool = False,
        session_id: str = "",
    ):
        super().__init__(MONITOR_ID)
        self.lock = threading.RLock()
        self.order_lock = threading.RLock()
        self.oracle = self._connect_oracle()
        ordered_delays = [item["max_delay_ms"] for item in INTERFACES if item["ordered"]]
        self.ordered_buffer = OrderedEventBuffer(max(ordered_delays) if ordered_delays else 0)
        self.interfaces_by_key = {{item["key"]: item for item in INTERFACES}}
        self.publishers_by_interface = {{}}
        self.service_clients_by_interface = {{}}
        self._stop_requested = False
        self.session_id = str(session_id or os.getppid())
        self._open_files(fresh_session)
        self._create_ros_interfaces()
        self._publish_status("started", {{"interfaces": INTERFACES, "log_path": LOG_PATH}})
        if dashboard:
            self._start_dashboard(dashboard_host, dashboard_port)

    def _clear_shared_log_once(self, log_path: str, session_id: str):
        path = Path(log_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        marker = Path(str(path) + ".session")
        lock = Path(str(path) + ".lock")
        lock.parent.mkdir(parents=True, exist_ok=True)
        with lock.open("a", encoding="utf-8") as handle:
            if fcntl is not None:
                fcntl.flock(handle, fcntl.LOCK_EX)
            try:
                previous = marker.read_text(encoding="utf-8") if marker.exists() else None
                if previous != session_id:
                    path.write_text("", encoding="utf-8")
                    marker.write_text(session_id, encoding="utf-8")
            finally:
                if fcntl is not None:
                    fcntl.flock(handle, fcntl.LOCK_UN)

    def _open_files(self, fresh_session: bool = False):
        Path(LOG_PATH).parent.mkdir(parents=True, exist_ok=True)
        Path(STATUS_LOG_PATH).parent.mkdir(parents=True, exist_ok=True)
        if fresh_session:
            Path(LOG_PATH).write_text("", encoding="utf-8")
            if STATUS_ENABLED:
                self._clear_shared_log_once(STATUS_LOG_PATH, self.session_id)
        self.log_file = open(LOG_PATH, "a", encoding="utf-8")
        self.status_file = open(STATUS_LOG_PATH, "a", encoding="utf-8") if STATUS_ENABLED else None

    def _log_info(self, message: str):
        if not SILENT:
            {log_line}

    def _log_warning(self, message: str):
        if WARNING:
            {warn_line}

    def _shutdown_runtime(self):
        {shutdown_block}

    def _connect_oracle(self):
        if ORACLE is None:
            return None
        if websocket is None:
            self._log_warning("websocket-client is not installed; monitor will run in log-only mode")
            return None
        try:
            return websocket.create_connection(ORACLE["url"], timeout=ORACLE["timeout"])
        except Exception as exc:
            self._log_warning("could not connect to oracle: " + str(exc))
            return None

    def _event(self, interface: dict[str, Any], payload: Any, direction: str = "message") -> dict[str, Any]:
        observed = time.time()
        payload_dict = message_to_dict(payload)
        event_time = source_time(payload, observed)
        event = {{
            "__monitor": MONITOR_ID,
            "__kind": interface["kind"],
            "__interface": interface["name"],
            "__interface_key": interface["key"],
            "__direction": direction,
            "__action": interface["action"],
            "__ordered": interface["ordered"],
            "__observed_time": observed,
            "__source_time": event_time,
        }}
        if interface["kind"] == "topic":
            if isinstance(payload_dict, dict):
                event.update(payload_dict)
            else:
                event["data"] = payload_dict
            event["topic"] = interface["legacy_name"]
            event["time"] = event_time
            return event

        event["service"] = interface["legacy_name"]
        event["time"] = event_time
        event[direction] = payload_dict
        if isinstance(payload_dict, dict) and "stamp" in payload_dict:
            event["stamp"] = payload_dict["stamp"]
        return event

    def _wire_event(self, event: dict[str, Any]) -> dict[str, Any]:
        return {{key: value for key, value in event.items() if not key.startswith("__")}}

    def _event_payload_for_status(self, wire_event: dict[str, Any]) -> Any:
        if "request" in wire_event:
            return wire_event["request"]
        if "response" in wire_event:
            return wire_event["response"]
        if "data" in wire_event:
            return wire_event["data"]
        return {{
            key: value
            for key, value in wire_event.items()
            if key not in {{"time", "topic", "service"}}
        }}

    def _write_jsonl(self, file_handle, event: dict[str, Any]):
        file_handle.write(json.dumps(event, sort_keys=True) + "\\n")
        file_handle.flush()

    def _publish_status(self, status: str, extra: dict[str, Any] | None = None):
        if not self.status_file:
            return
        event = {{
            "monitor": MONITOR_ID,
            "status": status,
            "time": time.time(),
            "ros_version": ROS_VERSION,
            "session_id": getattr(self, "session_id", str(os.getppid())),
        }}
        if extra:
            event.update(extra)
        self._write_jsonl(self.status_file, event)

    def _start_dashboard(self, host: str, port: int):
        if not STATUS_ENABLED:
            self._log_warning("status dashboard requested, but status logging is disabled")
            return

        class Handler(BaseHTTPRequestHandler):
            def do_GET(handler_self):
                path = urlparse(handler_self.path).path
                if path == "/api/status":
                    body = json.dumps(read_status_log(STATUS_LOG_PATH)).encode("utf-8")
                    handler_self.send_response(200)
                    handler_self.send_header("Content-Type", "application/json")
                    handler_self.send_header("Content-Length", str(len(body)))
                    handler_self.end_headers()
                    handler_self.wfile.write(body)
                    return
                if path == "/api/dashboard":
                    body = json.dumps(dashboard_payload()).encode("utf-8")
                    handler_self.send_response(200)
                    handler_self.send_header("Content-Type", "application/json")
                    handler_self.send_header("Content-Length", str(len(body)))
                    handler_self.end_headers()
                    handler_self.wfile.write(body)
                    return
                body = DASHBOARD_HTML.encode("utf-8")
                handler_self.send_response(200)
                handler_self.send_header("Content-Type", "text/html; charset=utf-8")
                handler_self.send_header("Content-Length", str(len(body)))
                handler_self.end_headers()
                handler_self.wfile.write(body)

            def log_message(handler_self, format, *args):
                return

        try:
            server = ThreadingHTTPServer((host, port), Handler)
        except OSError as exc:
            self._log_warning("could not start status dashboard: " + str(exc))
            self._publish_status("dashboard_error", {{"error": str(exc), "host": host, "port": port}})
            return
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        self._dashboard_server = server
        self._publish_status("dashboard_started", {{"host": host, "port": port}})
        self._log_info("ROSMonitoring status dashboard: http://" + host + ":" + str(port))

    def _normalize_verdict(self, verdict: Any) -> str:
        if isinstance(verdict, bool):
            return "true" if verdict else "false"
        return str(verdict).lower()

    def _publish_verdict(self, verdict: Any):
        publisher = getattr(self, "verdict_publisher", None)
        if publisher is None:
            return
        message = ROSMonitoringVerdictString()
        message.data = self._normalize_verdict(verdict)
        publisher.publish(message)

    def _is_negative_verdict(self, verdict: Any) -> bool:
        return self._normalize_verdict(verdict) in {{"false", "currently_false", "violation", "violated"}}

    def _is_terminal_verdict(self, verdict: Any) -> bool:
        return self._normalize_verdict(verdict) in {{"true", "false"}}

    def _can_stop_after_terminal_verdict(self) -> bool:
        return not any(interface["intercepting"] for interface in INTERFACES)

    def _should_report_violation(self, verdict: Any) -> bool:
        if WARNING <= 0:
            return False
        normalized = self._normalize_verdict(verdict)
        if normalized in {{"false", "violation", "violated"}}:
            return True
        return normalized == "currently_false" and WARNING == 1

    def _request_stop_after_terminal_verdict(self, verdict: Any):
        if not self._is_terminal_verdict(verdict) or getattr(self, "_stop_requested", False):
            return
        if not self._can_stop_after_terminal_verdict():
            return
        self._stop_requested = True
        self._log_info("terminal oracle verdict " + self._normalize_verdict(verdict) + "; stopping passive monitor")
        self._shutdown_runtime()

    def _oracle_verdict(self, event: dict[str, Any]) -> tuple[bool, Any, Any]:
        if self.oracle is None:
            return True, "unknown", None
        try:
            self.oracle.send(json.dumps(self._wire_event(event)))
            response = self.oracle.recv()
        except Exception as exc:
            self._publish_status("oracle_error", {{"error": str(exc)}})
            return True, "unknown", None
        try:
            decoded = json.loads(response)
        except Exception:
            decoded = response
        if isinstance(decoded, dict):
            verdict = decoded.get("verdict", decoded.get("ok", True))
            return not self._is_negative_verdict(verdict), verdict, decoded
        return not self._is_negative_verdict(decoded), decoded, decoded

    def _handle_event(self, interface: dict[str, Any], event: dict[str, Any], payload: Any) -> bool:
        with self.lock:
            wire_event = self._wire_event(event)
            self._write_jsonl(self.log_file, wire_event)
            verdict, oracle_verdict, oracle_response = self._oracle_verdict(event)
            direction = event.get("__direction", "message")
            blocked = (
                interface["action"] == "filter"
                and not verdict
                and (interface["kind"] == "topic" or direction == "request")
            )
            communication_allowed = not blocked
            if blocked:
                decision = "blocked"
            elif interface["action"] == "filter" and (interface["kind"] == "topic" or direction == "request"):
                decision = "forwarded"
            elif interface["kind"] == "service" and direction == "response":
                decision = "response_logged"
            else:
                decision = "logged"
            self._publish_status(
                "event",
                {{
                    "interface": interface["name"],
                    "kind": interface["kind"],
                    "action": interface["action"],
                    "direction": direction,
                    "event": wire_event,
                    "payload": self._event_payload_for_status(wire_event),
                    "verdict": verdict,
                    "verdict_raw": oracle_verdict,
                    "blocked": blocked,
                    "communication_allowed": communication_allowed,
                    "decision": decision,
                    "terminal": self._is_terminal_verdict(oracle_verdict),
                    "will_stop": self._is_terminal_verdict(oracle_verdict) and self._can_stop_after_terminal_verdict(),
                    "ordered": interface["ordered"],
                    "oracle_response": oracle_response,
                }},
            )
            self._publish_verdict(oracle_verdict)
            if self._should_report_violation(oracle_verdict):
                self._log_warning("property violation on " + interface["name"])
            self._request_stop_after_terminal_verdict(oracle_verdict)
            return communication_allowed

    def _requires_inline_decision(self, interface: dict[str, Any], direction: str) -> bool:
        return interface["action"] == "filter" and (
            interface["kind"] == "topic" or direction == "request"
        )

    def _process_ordered_ready(self, ready: list[tuple[dict[str, Any], Any, OrderedDecision]]):
        for ready_event, ready_payload, decision in ready:
            interface = self.interfaces_by_key[ready_event["__interface_key"]]
            allowed = self._handle_event(interface, ready_event, ready_payload)
            decision.set_result(allowed)

    def _drain_ordered_ready(self):
        with self.order_lock:
            ready = self.ordered_buffer.flush_ready()
        self._process_ordered_ready(ready)

    def _drain_ordered_expired(self):
        with self.order_lock:
            ready = self.ordered_buffer.flush_expired()
        self._process_ordered_ready(ready)

    def _flush_ordered_through(self, decision: OrderedDecision):
        with self.order_lock:
            ready = self.ordered_buffer.flush_through(decision)
        self._process_ordered_ready(ready)

    def _wait_for_ordered_decision(self, decision: OrderedDecision) -> bool:
        deadline = time.monotonic() + self.ordered_buffer.max_delay
        while not decision.done:
            self._drain_ordered_ready()
            if decision.done:
                break
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self._drain_ordered_expired()
                if not decision.done:
                    self._flush_ordered_through(decision)
                break
            decision.wait(min(remaining, 0.05))
        return decision.allowed

    def _handle_or_buffer(self, interface: dict[str, Any], payload: Any, direction: str = "message") -> bool:
        event = self._event(interface, payload, direction)
        if not interface["ordered"]:
            return self._handle_event(interface, event, payload)
        event["__buffered_at"] = time.monotonic()
        decision = OrderedDecision(self._requires_inline_decision(interface, direction))
        with self.order_lock:
            self.ordered_buffer.push(event, payload, decision)
            ready = self.ordered_buffer.flush_ready()
        self._process_ordered_ready(ready)
        if decision.requires_decision:
            return self._wait_for_ordered_decision(decision)
        return True

    def close(self):
        with self.order_lock:
            ready = self.ordered_buffer.flush_all()
        self._process_ordered_ready(ready)
        self._publish_status("stopped")
        dashboard_server = getattr(self, "_dashboard_server", None)
        if dashboard_server is not None:
            dashboard_server.shutdown()
            dashboard_server.server_close()
        self.log_file.close()
        if self.status_file:
            self.status_file.close()
'''


def _ros2_runtime(class_name: str) -> str:
    runtime = f'''try:
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile
except Exception:
    rclpy = None
    Node = object
    ReentrantCallbackGroup = None
    ExternalShutdownException = KeyboardInterrupt
    MultiThreadedExecutor = None
    DurabilityPolicy = None
    QoSProfile = None

{_common_monitor_methods(class_name, ros2=True)}

    def _create_ros_interfaces(self):
        self.callback_group = ReentrantCallbackGroup()
        verdict_qos = QoSProfile(depth=1000, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.verdict_publisher = self.create_publisher(
            ROSMonitoringVerdictString,
            MONITOR_ID + "/monitor_verdict",
            verdict_qos,
            callback_group=self.callback_group,
        )
        for interface in INTERFACES:
            msg_type = globals()[interface["type"]]
            if interface["kind"] == "topic":
                subscribe_name = interface["remapped_name"] if interface["intercepting"] else interface["name"]
                self.create_subscription(
                    msg_type,
                    subscribe_name,
                    self._topic_callback(interface),
                    1000,
                    callback_group=self.callback_group,
                )
                if interface["intercepting"]:
                    publish_name = interface["name"] if interface["publishers"] else interface["remapped_name"]
                    self.publishers_by_interface[interface["name"]] = self.create_publisher(msg_type, publish_name, 1000)
            else:
                self.create_service(
                    msg_type,
                    interface["remapped_name"],
                    self._service_callback(interface),
                    callback_group=self.callback_group,
                )
                self.service_clients_by_interface[interface["name"]] = self.create_client(
                    msg_type,
                    interface["name"],
                    callback_group=self.callback_group,
                )

    def _topic_callback(self, interface):
        def callback(message):
            allowed = self._handle_or_buffer(interface, message)
            publisher = self.publishers_by_interface.get(interface["name"])
            if allowed and publisher is not None:
                publisher.publish(message)
        return callback

    def _service_callback(self, interface):
        def callback(request, response):
            allowed = self._handle_or_buffer(interface, request, "request")
            if not allowed:
                return response
            client = self.service_clients_by_interface[interface["name"]]
            if not client.wait_for_service(timeout_sec=10.0):
                self._publish_status("service_unavailable", {{"interface": interface["name"]}})
                return response
            future = client.call_async(request)
            done = threading.Event()
            future.add_done_callback(lambda _: done.set())
            if not done.wait(timeout=10.0):
                self._publish_status("service_timeout", {{"interface": interface["name"]}})
                return response
            result = future.result()
            self._handle_or_buffer(interface, result, "response")
            return result
        return callback


def main(args=None):
    if rclpy is None:
        raise RuntimeError("ROS2 rclpy is not available")
    monitor_args, ros_args = parse_monitor_args(args)
    rclpy.init(args=ros_args)
    monitor = {class_name}(
        dashboard=monitor_args.dashboard,
        dashboard_host=monitor_args.dashboard_host,
        dashboard_port=monitor_args.dashboard_port,
        fresh_session=monitor_args.fresh_session,
        session_id=monitor_args.session_id,
    )
    executor = MultiThreadedExecutor()
    executor.add_node(monitor)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        monitor.close()
        monitor.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
'''
    return runtime


def _ros1_runtime(class_name: str) -> str:
    runtime = f'''try:
    import rospy
except Exception:
    rospy = None

class Node:
    def __init__(self, name):
        if rospy is not None:
            rospy.init_node(name, anonymous=False)

{_common_monitor_methods(class_name, ros2=False)}

    def _create_ros_interfaces(self):
        self.verdict_publisher = rospy.Publisher(
            name=MONITOR_ID + "/monitor_verdict",
            data_class=ROSMonitoringVerdictString,
            latch=True,
            queue_size=1000,
        )
        for interface in INTERFACES:
            msg_type = globals()[interface["type"]]
            if interface["kind"] == "topic":
                subscribe_name = interface["remapped_name"] if interface["intercepting"] else interface["name"]
                rospy.Subscriber(subscribe_name, msg_type, self._topic_callback(interface), queue_size=1000)
                if interface["intercepting"]:
                    publish_name = interface["name"] if interface["publishers"] else interface["remapped_name"]
                    self.publishers_by_interface[interface["name"]] = rospy.Publisher(publish_name, msg_type, queue_size=1000)
            else:
                rospy.Service(interface["remapped_name"], msg_type, self._service_callback(interface))
                self.service_clients_by_interface[interface["name"]] = rospy.ServiceProxy(interface["name"], msg_type)

    def _topic_callback(self, interface):
        def callback(message):
            allowed = self._handle_or_buffer(interface, message)
            publisher = self.publishers_by_interface.get(interface["name"])
            if allowed and publisher is not None:
                publisher.publish(message)
        return callback

    def _service_callback(self, interface):
        def callback(request):
            allowed = self._handle_or_buffer(interface, request, "request")
            if not allowed:
                response_cls = getattr(globals()[interface["type"]], "_response_class", None)
                return response_cls() if response_cls is not None else None
            rospy.wait_for_service(interface["name"])
            result = self.service_clients_by_interface[interface["name"]](request)
            self._handle_or_buffer(interface, result, "response")
            return result
        return callback


def main():
    if rospy is None:
        raise RuntimeError("ROS1 rospy is not available")
    monitor_args, _ = parse_monitor_args()
    monitor = {class_name}(
        dashboard=monitor_args.dashboard,
        dashboard_host=monitor_args.dashboard_host,
        dashboard_port=monitor_args.dashboard_port,
        fresh_session=monitor_args.fresh_session,
        session_id=monitor_args.session_id,
    )
    rospy.on_shutdown(monitor.close)
    rospy.spin()


if __name__ == "__main__":
    main()
'''
    return runtime


def _ros2_package_xml(monitors: tuple[MonitorSpec, ...]) -> str:
    dependencies = sorted(
        {
            interface.type.split(".", 1)[0]
            for monitor in monitors
            for interface in monitor.interfaces
            if "." in interface.type
        }
        | {"std_msgs"}
    )
    dependency_xml = "\n".join(f"  <exec_depend>{dependency}</exec_depend>" for dependency in dependencies)
    return f"""<?xml version="1.0"?>
<package format="3">
  <name>monitor</name>
  <version>3.0.0</version>
  <description>Generated ROSMonitoring monitors.</description>
  <maintainer email="maintainer@example.com">ROSMonitoring</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_python</buildtool_depend>
  <exec_depend>rclpy</exec_depend>
{dependency_xml}
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"""


def _ros2_setup(monitors: tuple[MonitorSpec, ...]) -> str:
    entry_points = "\n".join(f"            '{monitor.id} = monitor.{monitor.id}:main'," for monitor in monitors)
    return f"""from setuptools import setup

package_name = 'monitor'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/monitor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROSMonitoring',
    maintainer_email='maintainer@example.com',
    description='Generated ROSMonitoring monitors',
    license='MIT',
    entry_points={{
        'console_scripts': [
{entry_points}
        ],
    }},
)
"""


def _ros2_launch(monitors: tuple[MonitorSpec, ...]) -> str:
    monitor_ids = [monitor.id for monitor in monitors]
    monitor_ids_repr = repr(monitor_ids)
    return f"""from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


MONITORS = {monitor_ids_repr}


def _truthy(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _launch_setup(context, *args, **kwargs):
    dashboard = _truthy(LaunchConfiguration('dashboard').perform(context))
    dashboard_monitor = LaunchConfiguration('dashboard_monitor').perform(context)
    dashboard_host = LaunchConfiguration('dashboard_host').perform(context)
    dashboard_port = LaunchConfiguration('dashboard_port').perform(context)
    fresh_session = _truthy(LaunchConfiguration('fresh_session').perform(context))
    session_id = LaunchConfiguration('session_id').perform(context)
    nodes = []
    for index, monitor_id in enumerate(MONITORS):
        arguments = []
        wants_dashboard = dashboard and (
            dashboard_monitor == monitor_id
            or (dashboard_monitor == 'first' and index == 0)
        )
        if wants_dashboard:
            arguments.extend(['--dashboard', '--dashboard-host', dashboard_host, '--dashboard-port', dashboard_port])
            if fresh_session:
                arguments.append('--fresh-session')
            if session_id:
                arguments.extend(['--session-id', session_id])
        nodes.append(Node(package='monitor', executable=monitor_id, name=monitor_id, output='screen', arguments=arguments))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('dashboard', default_value='false'),
        DeclareLaunchArgument('dashboard_monitor', default_value='first'),
        DeclareLaunchArgument('dashboard_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('dashboard_port', default_value='8765'),
        DeclareLaunchArgument('fresh_session', default_value='false'),
        DeclareLaunchArgument('session_id', default_value=''),
        OpaqueFunction(function=_launch_setup),
    ])
"""


def _ros1_package_xml() -> str:
    return """<?xml version="1.0"?>
<package format="2">
  <name>monitor</name>
  <version>3.0.0</version>
  <description>Generated ROSMonitoring monitors.</description>
  <maintainer email="maintainer@example.com">ROSMonitoring</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
</package>
"""


def _ros1_cmakelists(monitors: tuple[MonitorSpec, ...]) -> str:
    programs = "\n".join(f"  src/{monitor.id}.py" for monitor in monitors)
    return f"""cmake_minimum_required(VERSION 3.0.2)
project(monitor)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs)
catkin_package()
catkin_install_python(PROGRAMS
{programs}
  DESTINATION ${{CATKIN_PACKAGE_BIN_DESTINATION}}
)
"""


def _ros1_launch_file(monitor: MonitorSpec) -> GeneratedFile:
    return GeneratedFile(
        Path(f"monitor/run_{monitor.id}.launch"),
        textwrap.dedent(
            f"""\
            <launch>
              <node pkg="monitor" type="{monitor.id}.py" name="{monitor.id}" output="screen"/>
            </launch>
            """
        ),
    )
