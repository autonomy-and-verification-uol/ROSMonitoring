import pytest

from rosmonitoring.config import ConfigError, parse_config
from rosmonitoring.generator import render_project


def test_parse_legacy_ordered_topic_and_service():
    config = parse_config(
        {
            "monitors": [
                {
                    "monitor": {
                        "id": "monitor_0",
                        "log": "./log.jsonl",
                        "oracle": {"url": "127.0.0.1", "port": 8080, "action": "nothing"},
                        "topics": [
                            {
                                "name": "chatter",
                                "type": "std_msgs.msg.String",
                                "action": "filter",
                                "publishers": ["talker"],
                                "ordered": True,
                            }
                        ],
                        "services": [
                            {
                                "name": "add_two_ints",
                                "type": "example_interfaces.srv.AddTwoInts",
                                "action": "log",
                            }
                        ],
                    }
                }
            ]
        },
        ros_version="ros1",
    )

    monitor = config.monitors[0]
    assert config.ros_version == "ros1"
    assert monitor.oracle.websocket_url == "ws://127.0.0.1:8080"
    assert monitor.topics[0].normalized_name == "/chatter"
    assert monitor.topics[0].remapped_name == "/chatter_mon"
    assert monitor.topics[0].ordering.enabled is True
    assert monitor.services[0].kind == "service"


def test_parse_status_disabled_and_ordering_dictionary():
    config = parse_config(
        {
            "ros_version": "ros2",
            "monitors": [
                {
                    "monitor": {
                        "id": "m0",
                        "log": "./m0.jsonl",
                        "status": {"enabled": False, "log": "./status.jsonl"},
                        "topics": [
                            {
                                "name": "/robot/pose",
                                "type": "geometry_msgs.msg.PoseStamped",
                                "ordering": {"enabled": True, "max_delay_ms": 50},
                            }
                        ],
                    }
                }
            ],
        }
    )

    monitor = config.monitors[0]
    assert monitor.status.enabled is False
    assert monitor.status.log == "./status.jsonl"
    assert monitor.topics[0].ordering.enabled is True
    assert monitor.topics[0].ordering.max_delay_ms == 50
    assert monitor.topics[0].python_name == "robot_pose"


def test_invalid_action_is_rejected():
    with pytest.raises(ConfigError):
        parse_config(
            {
                "monitors": [
                    {
                        "monitor": {
                            "id": "bad",
                            "log": "./log.jsonl",
                            "topics": [{"name": "x", "type": "std_msgs.msg.String", "action": "drop"}],
                        }
                    }
                ]
            }
        )


def test_invalid_ros_version_monitor_id_and_empty_monitor_are_rejected():
    with pytest.raises(ConfigError):
        parse_config({"ros_version": "ros3", "monitors": [{"monitor": {"id": "m", "log": "./m.jsonl"}}]})

    with pytest.raises(ConfigError):
        parse_config(
            {
                "monitors": [
                    {
                        "monitor": {
                            "id": "not-valid-name",
                            "log": "./m.jsonl",
                            "topics": [{"name": "x", "type": "pkg.msg.T"}],
                        }
                    }
                ]
            }
        )

    with pytest.raises(ConfigError):
        parse_config({"monitors": [{"monitor": {"id": "empty", "log": "./m.jsonl"}}]})


def test_configuration_matrix_renders_supported_ros_interface_combinations():
    rendered = 0
    ordering_variants = [
        {},
        {"ordered": True},
        {"ordering": {"enabled": True, "max_delay_ms": 25}},
    ]

    for ros_version in ("ros1", "ros2"):
        for kind in ("topic", "service"):
            for action in ("log", "filter"):
                for ordering in ordering_variants:
                    for has_oracle in (False, True):
                        for status_enabled in (False, True):
                            interface = {
                                "name": f"{kind}_{action}",
                                "type": "std_msgs.msg.String" if kind == "topic" else "std_srvs.srv.SetBool",
                                "action": action,
                                **ordering,
                            }
                            if kind == "topic" and action == "filter":
                                interface["publishers"] = ["source"]
                            monitor = {
                                "id": f"{ros_version}_{kind}_{action}_{rendered}",
                                "log": "./events.jsonl",
                                "status": {"enabled": status_enabled, "log": "./status.jsonl"},
                                f"{kind}s": [interface],
                            }
                            if has_oracle:
                                monitor["oracle"] = {"url": "127.0.0.1", "port": 8080, "action": "nothing"}
                            config = parse_config({"ros_version": ros_version, "monitors": [{"monitor": monitor}]})
                            files = render_project(config, ros_version)
                            rendered_paths = {item.path.as_posix() for item in files}
                            assert "monitor/package.xml" in rendered_paths
                            if ros_version == "ros2":
                                assert any(path.startswith("monitor/monitor/") for path in rendered_paths)
                            else:
                                assert any(path.startswith("monitor/src/") for path in rendered_paths)
                            rendered += 1

    assert rendered == 96
