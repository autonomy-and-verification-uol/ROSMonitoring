from __future__ import annotations

import py_compile
from pathlib import Path

from rosmonitoring.config import load_config
from rosmonitoring.generator import render_project


ROOT = Path(__file__).resolve().parents[1]
EXAMPLES = ROOT / "examples"


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
