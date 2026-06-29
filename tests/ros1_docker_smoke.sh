#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${ROSMONITORING_ROS1_IMAGE:-ros:noetic-ros-base}"

docker run --rm -i -v "$REPO_ROOT":/repo -w /repo "$IMAGE" bash <<'BASH'
set -euo pipefail
source /opt/ros/noetic/setup.bash
export PYTHONPATH=/repo/src:${PYTHONPATH:-}

rm -rf /tmp/ros1_ws
mkdir -p /tmp/ros1_ws/src /tmp/ros1_ws/logs

cat > /tmp/ros1_config.yaml <<'YAML'
ros_version: ros1
monitors:
  - monitor:
      id: ros1_smoke_monitor
      log: /tmp/ros1_ws/logs/ros1_smoke_monitor.jsonl
      silent: true
      warning: 1
      status:
        enabled: true
        log: /tmp/ros1_ws/logs/status.jsonl
      topics:
        - name: smoke_chatter
          type: std_msgs.msg.String
          action: filter
          publishers:
            - talker
      services:
        - name: set_bool
          type: std_srvs.srv.SetBool
          action: filter
YAML

python3 -m rosmonitoring.cli validate /tmp/ros1_config.yaml --ros-version ros1
python3 -m rosmonitoring.cli generate /tmp/ros1_config.yaml --ros-version ros1 --output /tmp/ros1_ws/src

cd /tmp/ros1_ws
catkin_make >/tmp/ros1_catkin_make.log
source devel/setup.bash

roscore >/tmp/ros1_core.log 2>&1 &
core_pid=$!
monitor_pid=
server_pid=
echo_pid=
verdict_echo_pid=
cleanup() {
  set +e
  kill "$monitor_pid" "$server_pid" "$echo_pid" "$verdict_echo_pid" "$core_pid" 2>/dev/null || true
  wait "$monitor_pid" "$server_pid" "$echo_pid" "$verdict_echo_pid" "$core_pid" 2>/dev/null || true
}
trap cleanup EXIT

python3 - <<'PY'
import subprocess
import time

for _ in range(60):
    result = subprocess.run(["rostopic", "list"], text=True, capture_output=True)
    if result.returncode == 0:
        raise SystemExit(0)
    time.sleep(0.25)
raise SystemExit("roscore did not start")
PY

cat > /tmp/ros1_service_server.py <<'PY'
#!/usr/bin/env python3
import json
from pathlib import Path

import rospy
from std_srvs.srv import SetBool, SetBoolResponse

log = Path("/tmp/ros1_ws/logs/service_server.jsonl")
log.parent.mkdir(parents=True, exist_ok=True)


def callback(request):
    with log.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps({"data": bool(request.data)}) + "\n")
    return SetBoolResponse(success=bool(request.data), message="accepted" if request.data else "rejected")


rospy.init_node("ros1_smoke_service_server")
rospy.Service("/set_bool", SetBool, callback)
rospy.spin()
PY

python3 /tmp/ros1_service_server.py >/tmp/ros1_service_server.log 2>&1 &
server_pid=$!
rosrun monitor ros1_smoke_monitor.py >/tmp/ros1_monitor.log 2>&1 &
monitor_pid=$!

python3 - <<'PY'
import subprocess
import time

for _ in range(80):
    topics = subprocess.run(["rostopic", "list"], text=True, capture_output=True).stdout.splitlines()
    services = subprocess.run(["rosservice", "list"], text=True, capture_output=True).stdout.splitlines()
    if (
        "/smoke_chatter_mon" in topics
        and "/smoke_chatter" in topics
        and "/ros1_smoke_monitor/monitor_verdict" in topics
        and "/set_bool_mon" in services
        and "/set_bool" in services
    ):
        raise SystemExit(0)
    time.sleep(0.25)
raise SystemExit("monitor interfaces not ready")
PY

rostopic echo -n 1 /smoke_chatter >/tmp/ros1_echo.log 2>&1 &
echo_pid=$!
sleep 2
rostopic pub -1 /smoke_chatter_mon std_msgs/String "data: hello" >/tmp/ros1_pub.log
wait "$echo_pid"
echo_pid=
rostopic echo -n 1 /ros1_smoke_monitor/monitor_verdict >/tmp/ros1_verdict_echo.log 2>&1 &
verdict_echo_pid=$!
wait "$verdict_echo_pid"
verdict_echo_pid=

rosservice call /set_bool_mon "data: true" >/tmp/ros1_service_call.log

python3 - <<'PY'
from pathlib import Path
import json
import time

monitor_log = Path("/tmp/ros1_ws/logs/ros1_smoke_monitor.jsonl")
server_log = Path("/tmp/ros1_ws/logs/service_server.jsonl")
status_log = Path("/tmp/ros1_ws/logs/status.jsonl")
verdict_echo = Path("/tmp/ros1_verdict_echo.log")

for _ in range(40):
    if monitor_log.exists() and server_log.exists() and status_log.exists():
        rows = [json.loads(line) for line in monitor_log.read_text().splitlines() if line.strip()]
        server = [json.loads(line) for line in server_log.read_text().splitlines() if line.strip()]
        status = [json.loads(line) for line in status_log.read_text().splitlines() if line.strip()]
        if len(rows) >= 3 and server:
            assert any(row.get("topic") == "smoke_chatter" and row.get("data") == "hello" for row in rows), rows
            assert any(row.get("service") == "set_bool" and row.get("request", {}).get("data") is True for row in rows), rows
            assert any(row.get("service") == "set_bool" and row.get("response", {}).get("success") is True for row in rows), rows
            assert server == [{"data": True}], server
            assert any(item.get("monitor") == "ros1_smoke_monitor" and item.get("status") == "event" for item in status), status
            assert "unknown" in verdict_echo.read_text(encoding="utf-8"), verdict_echo.read_text(encoding="utf-8")
            print("ROS1 smoke ok")
            raise SystemExit(0)
    time.sleep(0.25)
raise SystemExit("expected ROS1 logs were not written")
PY
BASH
