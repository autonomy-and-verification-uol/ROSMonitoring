from __future__ import annotations

import importlib.util
import py_compile
from pathlib import Path

from rosmonitoring.config import load_config
from rosmonitoring.generator import render_project


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "examples"
TURTLESIM_CASE = EXAMPLES / "case_studies" / "turtlesim_reelay"


def test_all_tutorial_yaml_files_validate_and_generate():
    for path in sorted((EXAMPLES / "tutorials").glob("*.yaml")):
        config = load_config(path)
        files = render_project(config, config.ros_version or "ros2")
        generated_paths = {item.path.as_posix() for item in files}
        assert "monitor/package.xml" in generated_paths
        assert any(item.startswith("monitor/monitor/") and item.endswith(".py") for item in generated_paths), path


def test_ros2_example_nodes_are_valid_python():
    for path in sorted((EXAMPLES / "ros2_system").glob("*.py")):
        py_compile.compile(str(path), doraise=True)


def test_case_study_python_files_are_valid_python():
    for path in sorted(TURTLESIM_CASE.glob("*.py")):
        py_compile.compile(str(path), doraise=True)


def test_turtlesim_case_study_yaml_validates_and_renders():
    config = load_config(TURTLESIM_CASE / "monitor.yaml")
    monitor = config.monitors[0]
    assert monitor.id == "turtlesim_safety_monitor"
    assert {topic.normalized_name for topic in monitor.topics} == {
        "/turtle1/cmd_vel",
        "/turtle1/pose",
        "/turtle1/pose_stamped",
    }
    assert monitor.services[0].normalized_name == "/turtle1/teleport_absolute"

    files = render_project(config, "ros2")
    package_xml = next(item.content for item in files if item.path.as_posix() == "monitor/package.xml")
    source = next(
        item.content
        for item in files
        if item.path.as_posix() == "monitor/monitor/turtlesim_safety_monitor.py"
    )

    assert "<exec_depend>turtlesim</exec_depend>" in package_xml
    assert "<exec_depend>geometry_msgs</exec_depend>" in package_xml
    assert "/turtle1/cmd_vel_mon" in source
    assert "/turtle1/teleport_absolute_mon" in source
    assert '"ordered": true' not in source
    assert "turtle1/pose_stamped" in source


def test_turtlesim_case_study_reelay_property_abstraction():
    spec = importlib.util.spec_from_file_location("turtlesim_property", TURTLESIM_CASE / "turtlesim_property.py")
    assert spec is not None
    assert spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    assert module.PROPERTY == "{safe}"
    assert module.abstract_message(
        {
            "topic": "turtle1/cmd_vel",
            "time": 1.0,
            "linear": {"x": 1.0},
            "angular": {"z": 0.5},
        }
    )["safe"] is True
    assert module.abstract_message(
        {
            "topic": "turtle1/cmd_vel",
            "time": 2.0,
            "linear": {"x": 4.0},
            "angular": {"z": 0.0},
        }
    )["safe"] is False
    assert module.abstract_message(
        {
            "service": "turtle1/teleport_absolute",
            "time": 3.0,
            "request": {"x": 5.5, "y": 5.5, "theta": 0.0},
        }
    )["safe"] is True
    assert module.abstract_message(
        {
            "service": "turtle1/teleport_absolute",
            "time": 4.0,
            "request": {"x": 11.0, "y": 11.0, "theta": 0.0},
        }
    )["safe"] is False
    assert module.abstract_message(
        {
            "topic": "turtle1/pose",
            "time": 5.0,
            "x": 11.0,
            "y": 11.0,
        }
    )["safe"] is True


def test_turtlesim_case_study_offline_trace_is_jsonl():
    trace = TURTLESIM_CASE / "unsafe_trace.jsonl"
    rows = [line for line in trace.read_text(encoding="utf-8").splitlines() if line.strip()]
    assert len(rows) == 4
    assert all(line.startswith("{") and line.endswith("}") for line in rows)


def test_example_oracles_are_valid_python():
    for path in sorted((EXAMPLES / "oracles").glob("*.py")):
        py_compile.compile(str(path), doraise=True)


def test_oracle_example_assets_point_to_trusted_oracles():
    readme = (EXAMPLES / "README.md").read_text(encoding="utf-8")
    assert "oracle/TLOracle/oracle.py" in readme
    assert "oracle/RMLOracle/prolog/online_monitor.sh" in readme
    assert "oracle/LamaConvOracle/oracle.py" in readme
    assert "rltlconv.jar" in readme
    assert "not included in this repository" in readme
    assert "Missing rltlconv.jar" in readme

    assert (ROOT / "oracle" / "TLOracle" / "chatter_property.py").is_file()
    assert (ROOT / "oracle" / "RMLOracle" / "rml" / "test.pl").is_file()
    assert (ROOT / "oracle" / "LamaConvOracle" / "property.py").is_file()


def test_oracle_trace_examples_are_jsonl():
    for path in sorted((EXAMPLES / "oracle_runs").glob("*.jsonl")):
        rows = [line for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
        assert rows, path
        assert all(line.startswith("{") and line.endswith("}") for line in rows), path


def test_examples_readme_points_to_turtlesim_case_study():
    readme = (EXAMPLES / "README.md").read_text(encoding="utf-8")
    case_readme = (TURTLESIM_CASE / "README.md").read_text(encoding="utf-8")

    assert "case_studies/turtlesim_reelay" in readme
    assert "TL/Reelay" in case_readme
    assert "turtlesim_safety_monitor/monitor_verdict" in case_readme
    assert "teleport_absolute_mon" in case_readme
