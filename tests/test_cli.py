from pathlib import Path

from rosmonitoring.cli import main


def test_cli_validate_and_generate(tmp_path: Path):
    config = tmp_path / "config.yaml"
    config.write_text(
        """
monitors:
  - monitor:
      id: m0
      log: ./m0.jsonl
      topics:
        - name: chatter
          type: std_msgs.msg.String
          action: log
""",
        encoding="utf-8",
    )
    assert main(["validate", str(config), "--ros-version", "ros2"]) == 0
    output = tmp_path / "out"
    assert main(["generate", str(config), "--ros-version", "ros2", "--output", str(output)]) == 0
    assert (output / "monitor" / "monitor" / "m0.py").exists()
