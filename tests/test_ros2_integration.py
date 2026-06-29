from __future__ import annotations

import json
import os
from pathlib import Path
import random
import shutil
import signal
import socket
import subprocess
import sys
import textwrap
import time
import urllib.request

import pytest


pytestmark = pytest.mark.skipif(
    shutil.which("ros2") is None,
    reason="ROS2 is not available in PATH",
)


def _run(command: str, *, cwd: Path, env: dict[str, str], timeout: float = 20.0) -> subprocess.CompletedProcess:
    result = subprocess.run(
        ["bash", "-lc", command],
        cwd=cwd,
        env=env,
        text=True,
        capture_output=True,
        timeout=timeout,
    )
    if result.returncode != 0:
        raise AssertionError(
            f"command failed ({result.returncode}): {command}\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        )
    return result


def _start(command: str, *, cwd: Path, env: dict[str, str]) -> subprocess.Popen:
    return subprocess.Popen(
        ["bash", "-lc", command],
        cwd=cwd,
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def _stop(process: subprocess.Popen, timeout: float = 12.0) -> str:
    if process.poll() is None:
        os.killpg(process.pid, signal.SIGINT)
    try:
        output, _ = process.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        output, _ = process.communicate(timeout=timeout)
    return output or ""


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def _wait_for(predicate, *, timeout: float = 10.0, interval: float = 0.2, detail: str = "condition") -> None:
    deadline = time.time() + timeout
    last_error = None
    while time.time() < deadline:
        try:
            if predicate():
                return
        except Exception as exc:  # pragma: no cover - only used for diagnostics
            last_error = exc
        time.sleep(interval)
    raise AssertionError(f"timed out waiting for {detail}; last error: {last_error}")


def _wait_for_file_lines(path: Path, minimum: int, *, timeout: float = 10.0) -> list[dict]:
    def has_lines() -> bool:
        return path.exists() and len([line for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]) >= minimum

    _wait_for(has_lines, timeout=timeout, detail=f"{path} to contain {minimum} JSONL lines")
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]


def _write_oracle(path: Path, log_path: Path, port: int) -> None:
    path.write_text(
        textwrap.dedent(
            f"""
            import asyncio
            import json
            from pathlib import Path
            import websockets

            LOG = Path({str(log_path)!r})
            LOG.parent.mkdir(parents=True, exist_ok=True)

            async def handler(websocket):
                async for message in websocket:
                    event = json.loads(message)
                    verdict = True
                    if event.get("topic") in {{"guarded_text", "/guarded_text"}} and event.get("data") == "drop":
                        verdict = False
                    if (
                        event.get("service") in {{"add_two_ints", "/add_two_ints"}}
                        and "request" in event
                        and (event.get("request") or {{}}).get("a") == -1
                    ):
                        verdict = False
                    with LOG.open("a", encoding="utf-8") as handle:
                        handle.write(json.dumps({{"event": event, "verdict": verdict}}, sort_keys=True) + "\\n")
                    await websocket.send(json.dumps({{"verdict": verdict}}))

            async def main():
                async with websockets.serve(handler, "127.0.0.1", {port}):
                    await asyncio.Future()

            asyncio.run(main())
            """
        ),
        encoding="utf-8",
    )


def _write_service_server(path: Path, log_path: Path) -> None:
    path.write_text(
        textwrap.dedent(
            f"""
            import json
            from pathlib import Path
            import rclpy
            from rclpy.node import Node
            from example_interfaces.srv import AddTwoInts

            LOG = Path({str(log_path)!r})
            LOG.parent.mkdir(parents=True, exist_ok=True)

            class Server(Node):
                def __init__(self):
                    super().__init__("rosmonitoring_test_add_two_ints_server")
                    self.create_service(AddTwoInts, "/add_two_ints", self.callback)

                def callback(self, request, response):
                    response.sum = request.a + request.b
                    with LOG.open("a", encoding="utf-8") as handle:
                        handle.write(json.dumps({{"a": request.a, "b": request.b, "sum": response.sum}}) + "\\n")
                    return response

            rclpy.init()
            node = Server()
            try:
                rclpy.spin(node)
            except KeyboardInterrupt:
                pass
            finally:
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            """
        ),
        encoding="utf-8",
    )


def _echo_once(
    workspace: Path, topic: str, msg_type: str, *, env: dict[str, str], transient_local: bool = False
) -> subprocess.Popen:
    qos = "--qos-durability transient_local " if transient_local else ""
    return _start(
        f". install/setup.bash && exec ros2 topic echo --once {qos}{topic} {msg_type}",
        cwd=workspace,
        env=env,
    )


def _publish_once(workspace: Path, topic: str, msg_type: str, payload: str, *, env: dict[str, str]) -> None:
    _run(
        f". install/setup.bash && ros2 topic pub --once {topic} {msg_type} '{payload}'",
        cwd=workspace,
        env=env,
        timeout=12,
    )


def _wait_for_string_topic_value(workspace: Path, topic: str, expected: str, *, env: dict[str, str]) -> list[str]:
    result = _run(
        f"""export ROSMONITORING_TOPIC={topic!r}
export ROSMONITORING_EXPECTED={expected!r}
. install/setup.bash
python3 - <<'PY'
import os
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

topic = os.environ["ROSMONITORING_TOPIC"]
expected = os.environ["ROSMONITORING_EXPECTED"]
received = []

rclpy.init()
node = rclpy.create_node("rosmonitoring_verdict_probe")
qos = QoSProfile(depth=10)
qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
qos.reliability = ReliabilityPolicy.RELIABLE
node.create_subscription(String, topic, lambda message: received.append(message.data), qos)
deadline = time.time() + 8.0
try:
    while expected not in received and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
if expected not in received:
    raise SystemExit("timed out waiting for " + expected + " on " + topic + "; received=" + repr(received))
for item in received:
    print(item)
PY""",
        cwd=workspace,
        env=env,
        timeout=12,
    )
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def _wait_for_service(workspace: Path, service: str, *, env: dict[str, str]) -> None:
    def exists() -> bool:
        result = _run(". install/setup.bash && ros2 service list", cwd=workspace, env=env, timeout=5)
        return service in result.stdout.splitlines()

    _wait_for(exists, timeout=15, detail=f"service {service}")


def _wait_for_subscriber(workspace: Path, topic: str, *, env: dict[str, str]) -> None:
    def exists() -> bool:
        result = _run(f". install/setup.bash && ros2 topic info {topic}", cwd=workspace, env=env, timeout=5)
        return "Subscription count: 1" in result.stdout or "Subscription count: 2" in result.stdout

    _wait_for(exists, timeout=15, detail=f"subscriber on {topic}")


def _read_json_url(url: str) -> dict:
    with urllib.request.urlopen(url, timeout=2) as response:
        return json.loads(response.read().decode("utf-8"))


def test_ros2_online_offline_services_ordering_and_status(tmp_path: Path):
    try:
        import rclpy  # noqa: F401
    except Exception as exc:  # pragma: no cover - collection skip handles common case
        pytest.skip(f"rclpy is unavailable: {exc}")

    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = str(random.randint(120, 220))
    env["PYTHONUNBUFFERED"] = "1"
    env.pop("GTK_PATH", None)
    env.pop("GIO_EXTRA_MODULES", None)

    workspace = tmp_path / "ws"
    source_dir = workspace / "src"
    source_dir.mkdir(parents=True)
    logs = tmp_path / "logs"
    oracle_port = _free_port()
    dashboard_port = _free_port()

    config_path = tmp_path / "config.yaml"
    config_path.write_text(
        textwrap.dedent(
            f"""
            ros_version: ros2
            monitors:
              - monitor:
                  id: online_topic_guard
                  log: {str(logs / "online_topic_guard.jsonl")!r}
                  status: {{enabled: true, log: {str(logs / "status.jsonl")!r}}}
                  oracle: {{url: 127.0.0.1, port: {oracle_port}, action: nothing}}
                  topics:
                    - name: guarded_text
                      type: std_msgs.msg.String
                      action: filter
                      publishers: [source]
              - monitor:
                  id: offline_topic_log
                  log: {str(logs / "offline_topic_log.jsonl")!r}
                  status: {{enabled: true, log: {str(logs / "status.jsonl")!r}}}
                  topics:
                    - name: offline_text
                      type: std_msgs.msg.String
                      action: log
              - monitor:
                  id: service_guard
                  log: {str(logs / "service_guard.jsonl")!r}
                  status: {{enabled: true, log: {str(logs / "status.jsonl")!r}}}
                  oracle: {{url: 127.0.0.1, port: {oracle_port}, action: nothing}}
                  services:
                    - name: add_two_ints
                      type: example_interfaces.srv.AddTwoInts
                      action: filter
              - monitor:
                  id: ordered_pose_log
                  log: {str(logs / "ordered_pose_log.jsonl")!r}
                  status: {{enabled: true, log: {str(logs / "status.jsonl")!r}}}
                  topics:
                    - name: ordered_pose
                      type: geometry_msgs.msg.PoseStamped
                      action: log
                      ordering:
                        enabled: true
                        max_delay_ms: 10000
              - monitor:
                  id: ordered_global_log
                  log: {str(logs / "ordered_global_log.jsonl")!r}
                  status: {{enabled: true, log: {str(logs / "status.jsonl")!r}}}
                  topics:
                    - name: ordered_alpha
                      type: geometry_msgs.msg.PoseStamped
                      action: log
                      ordering:
                        enabled: true
                        max_delay_ms: 10000
                    - name: ordered_beta
                      type: geometry_msgs.msg.PoseStamped
                      action: log
                      ordering:
                        enabled: true
                        max_delay_ms: 10000
            """
        ),
        encoding="utf-8",
    )

    source_pythonpath = str(Path.cwd() / "src")
    env["PYTHONPATH"] = (
        f"{source_pythonpath}{os.pathsep}{env['PYTHONPATH']}" if env.get("PYTHONPATH") else source_pythonpath
    )
    _run(
        f"{sys.executable} -m rosmonitoring.cli generate {config_path} --ros-version ros2 --output {source_dir}",
        cwd=Path.cwd(),
        env=env,
    )
    _run("colcon build --event-handlers console_direct+", cwd=workspace, env=env, timeout=60)

    oracle_log = logs / "oracle.jsonl"
    oracle_script = tmp_path / "oracle.py"
    _write_oracle(oracle_script, oracle_log, oracle_port)

    service_log = logs / "service_server.jsonl"
    service_script = tmp_path / "service_server.py"
    _write_service_server(service_script, service_log)

    oracle = _start(f"exec {sys.executable} {oracle_script}", cwd=tmp_path, env=env)
    server = _start(f"exec {sys.executable} {service_script}", cwd=tmp_path, env=env)
    monitors = [
        _start(
            f". install/setup.bash && exec ros2 run monitor online_topic_guard -- --dashboard --dashboard-port {dashboard_port}",
            cwd=workspace,
            env=env,
        ),
        _start(". install/setup.bash && exec ros2 run monitor offline_topic_log", cwd=workspace, env=env),
        _start(". install/setup.bash && exec ros2 run monitor service_guard", cwd=workspace, env=env),
        _start(". install/setup.bash && exec ros2 run monitor ordered_pose_log", cwd=workspace, env=env),
        _start(". install/setup.bash && exec ros2 run monitor ordered_global_log", cwd=workspace, env=env),
    ]

    try:
        _wait_for_service(workspace, "/add_two_ints", env=env)
        _wait_for_service(workspace, "/add_two_ints_mon", env=env)
        _wait_for_subscriber(workspace, "/guarded_text_mon", env=env)
        _wait_for_subscriber(workspace, "/offline_text", env=env)
        _wait_for_subscriber(workspace, "/ordered_pose", env=env)
        _wait_for_subscriber(workspace, "/ordered_alpha", env=env)
        _wait_for_subscriber(workspace, "/ordered_beta", env=env)
        _wait_for(
            lambda: _read_json_url(f"http://127.0.0.1:{dashboard_port}/api/dashboard")["monitor_config"]["id"]
            == "online_topic_guard",
            timeout=10,
            detail="online monitor dashboard",
        )

        echo = _echo_once(workspace, "/guarded_text", "std_msgs/msg/String", env=env)
        _wait_for_subscriber(workspace, "/guarded_text", env=env)
        _publish_once(workspace, "/guarded_text_mon", "std_msgs/msg/String", "{data: pass}", env=env)
        delivered, _ = echo.communicate(timeout=8)
        verdict_output = _wait_for_string_topic_value(
            workspace, "/online_topic_guard/monitor_verdict", "true", env=env
        )
        assert "data: pass" in delivered
        assert "true" in verdict_output

        blocked_echo = _echo_once(workspace, "/guarded_text", "std_msgs/msg/String", env=env)
        _wait_for_subscriber(workspace, "/guarded_text", env=env)
        _publish_once(workspace, "/guarded_text_mon", "std_msgs/msg/String", "{data: drop}", env=env)
        blocked_verdict_output = _wait_for_string_topic_value(
            workspace, "/online_topic_guard/monitor_verdict", "false", env=env
        )
        assert "false" in blocked_verdict_output
        with pytest.raises(subprocess.TimeoutExpired):
            blocked_echo.communicate(timeout=3)
        _stop(blocked_echo)

        _publish_once(workspace, "/offline_text", "std_msgs/msg/String", "{data: offline}", env=env)
        assert "unknown" in _wait_for_string_topic_value(
            workspace, "/offline_topic_log/monitor_verdict", "unknown", env=env
        )
        offline_rows = _wait_for_file_lines(logs / "offline_topic_log.jsonl", 1)
        assert offline_rows[-1]["topic"] == "offline_text"
        assert offline_rows[-1]["data"] == "offline"

        direct_service = _run(
            ". install/setup.bash && ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts '{a: 1, b: 1}'",
            cwd=workspace,
            env=env,
            timeout=25,
        )
        assert "sum=2" in direct_service.stdout or "sum: 2" in direct_service.stdout

        allowed_service = _run(
            ". install/setup.bash && ros2 service call /add_two_ints_mon example_interfaces/srv/AddTwoInts '{a: 2, b: 3}'",
            cwd=workspace,
            env=env,
            timeout=25,
        )
        assert "sum=5" in allowed_service.stdout or "sum: 5" in allowed_service.stdout
        assert "true" in _wait_for_string_topic_value(workspace, "/service_guard/monitor_verdict", "true", env=env)

        blocked_service = _run(
            ". install/setup.bash && ros2 service call /add_two_ints_mon example_interfaces/srv/AddTwoInts '{a: -1, b: 7}'",
            cwd=workspace,
            env=env,
            timeout=12,
        )
        assert "sum=0" in blocked_service.stdout or "sum: 0" in blocked_service.stdout
        assert "false" in _wait_for_string_topic_value(workspace, "/service_guard/monitor_verdict", "false", env=env)

        _publish_once(
            workspace,
            "/ordered_pose",
            "geometry_msgs/msg/PoseStamped",
            "{header: {stamp: {sec: 20, nanosec: 0}, frame_id: later}, pose: {orientation: {w: 1.0}}}",
            env=env,
        )
        _publish_once(
            workspace,
            "/ordered_pose",
            "geometry_msgs/msg/PoseStamped",
            "{header: {stamp: {sec: 10, nanosec: 0}, frame_id: earlier}, pose: {orientation: {w: 1.0}}}",
            env=env,
        )
        _publish_once(
            workspace,
            "/ordered_pose",
            "geometry_msgs/msg/PoseStamped",
            "{header: {stamp: {sec: 30, nanosec: 0}, frame_id: watermark}, pose: {orientation: {w: 1.0}}}",
            env=env,
        )

        ordered_rows = _wait_for_file_lines(logs / "ordered_pose_log.jsonl", 2)
        assert [row["time"] for row in ordered_rows[-2:]] == [10.0, 20.0]

        _publish_once(
            workspace,
            "/ordered_alpha",
            "geometry_msgs/msg/PoseStamped",
            "{header: {stamp: {sec: 20, nanosec: 0}, frame_id: alpha-later}, pose: {orientation: {w: 1.0}}}",
            env=env,
        )
        _publish_once(
            workspace,
            "/ordered_beta",
            "geometry_msgs/msg/PoseStamped",
            "{header: {stamp: {sec: 10, nanosec: 0}, frame_id: beta-earlier}, pose: {orientation: {w: 1.0}}}",
            env=env,
        )
        _publish_once(
            workspace,
            "/ordered_alpha",
            "geometry_msgs/msg/PoseStamped",
            "{header: {stamp: {sec: 30, nanosec: 0}, frame_id: alpha-watermark}, pose: {orientation: {w: 1.0}}}",
            env=env,
        )

        global_rows = _wait_for_file_lines(logs / "ordered_global_log.jsonl", 2)
        assert [(row["topic"], row["time"]) for row in global_rows[-2:]] == [
            ("ordered_beta", 10.0),
            ("ordered_alpha", 20.0),
        ]

        oracle_rows = _wait_for_file_lines(oracle_log, 5)
        assert any(row["event"].get("topic") == "guarded_text" and row["verdict"] is False for row in oracle_rows)
        assert any(row["event"].get("service") == "add_two_ints" and row["verdict"] is False for row in oracle_rows)

        service_rows = _wait_for_file_lines(service_log, 1)
        assert {"a": 2, "b": 3, "sum": 5} in service_rows
        assert {"a": -1, "b": 7, "sum": 6} not in service_rows

        monitor_service_rows = _wait_for_file_lines(logs / "service_guard.jsonl", 3)
        assert any(row.get("service") == "add_two_ints" and row.get("request", {}).get("a") == 2 for row in monitor_service_rows)
        assert any(row.get("service") == "add_two_ints" and row.get("response", {}).get("sum") == 5 for row in monitor_service_rows)
        assert any(row.get("service") == "add_two_ints" and row.get("request", {}).get("a") == -1 for row in monitor_service_rows)
        assert not any(row.get("service") == "add_two_ints" and row.get("response", {}).get("sum") == 6 for row in monitor_service_rows)

        status_rows = _wait_for_file_lines(logs / "status.jsonl", 8)
        assert any(row.get("monitor") == "online_topic_guard" and row.get("verdict") is False for row in status_rows)
        assert any(row.get("monitor") == "service_guard" and row.get("verdict") is False for row in status_rows)
        assert any(row.get("monitor") == "ordered_global_log" and row.get("ordered") is True for row in status_rows)
        assert not any(row.get("status") == "terminal_verdict" for row in status_rows)

        dashboard_payload = _read_json_url(f"http://127.0.0.1:{dashboard_port}/api/dashboard")
        assert dashboard_payload["monitor_config"]["id"] == "online_topic_guard"
        assert dashboard_payload["monitor_config"]["interfaces"][0]["kind"] == "topic"
        assert dashboard_payload["status"]["violations"] >= 2
        assert "online_topic_guard" in dashboard_payload["recent_events_by_monitor"]
        assert any(row.get("data") == "pass" for row in dashboard_payload["recent_events"])
        assert any(row.get("data") == "drop" for row in dashboard_payload["recent_events"])
    finally:
        for process in monitors:
            _stop(process)
        _stop(server)
        _stop(oracle)
