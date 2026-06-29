from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import re
from typing import Any

import yaml


VALID_ACTIONS = {"log", "filter"}
VALID_ROS_VERSIONS = {"ros1", "ros2"}
VALID_ORACLE_ACTIONS = {"nothing", "modify"}


class ConfigError(ValueError):
    """Raised when a ROSMonitoring configuration is invalid."""


@dataclass(frozen=True)
class NodeSpec:
    name: str
    package: str | None = None
    path: str | None = None


@dataclass(frozen=True)
class OracleSpec:
    url: str = "127.0.0.1"
    port: int = 8080
    action: str = "nothing"
    timeout: float = 2.0

    @property
    def websocket_url(self) -> str:
        return f"ws://{self.url}:{self.port}"


@dataclass(frozen=True)
class OrderingSpec:
    enabled: bool = False
    source: str = "header_stamp"
    max_delay_ms: int = 0


@dataclass(frozen=True)
class StatusSpec:
    enabled: bool = True
    log: str = "rosmonitoring-status.jsonl"


@dataclass(frozen=True)
class InterfaceSpec:
    name: str
    type: str
    action: str = "log"
    kind: str = "topic"
    publishers: tuple[str, ...] = ()
    subscribers: tuple[str, ...] = ()
    ordering: OrderingSpec = field(default_factory=OrderingSpec)

    @property
    def normalized_name(self) -> str:
        return self.name if self.name.startswith("/") else f"/{self.name}"

    @property
    def remapped_name(self) -> str:
        name = self.normalized_name
        return f"{name}_mon"

    @property
    def python_name(self) -> str:
        value = self.normalized_name.strip("/").replace("/", "_")
        return re.sub(r"[^0-9A-Za-z_]", "_", value) or "root"

    @property
    def is_intercepting(self) -> bool:
        return bool(self.publishers or self.subscribers or self.kind == "service")


@dataclass(frozen=True)
class MonitorSpec:
    id: str
    log: str
    topics: tuple[InterfaceSpec, ...] = ()
    services: tuple[InterfaceSpec, ...] = ()
    oracle: OracleSpec | None = None
    silent: bool = False
    warning: int = 1
    status: StatusSpec = field(default_factory=StatusSpec)

    @property
    def interfaces(self) -> tuple[InterfaceSpec, ...]:
        return self.topics + self.services

    @property
    def offline(self) -> bool:
        return self.oracle is None


@dataclass(frozen=True)
class ProjectConfig:
    path: str | None
    ros_version: str | None
    nodes: tuple[NodeSpec, ...]
    monitors: tuple[MonitorSpec, ...]


def load_config(path: str | Path, ros_version: str | None = None) -> ProjectConfig:
    source_path = Path(path)
    with source_path.open("r", encoding="utf-8") as handle:
        raw = yaml.safe_load(handle) or {}
    config = parse_config(raw, ros_version=ros_version)
    return config


def parse_config(raw: dict[str, Any], ros_version: str | None = None) -> ProjectConfig:
    selected_ros_version = ros_version or raw.get("ros_version")
    if selected_ros_version is not None:
        selected_ros_version = str(selected_ros_version).lower()
        if selected_ros_version not in VALID_ROS_VERSIONS:
            raise ConfigError(f"ros_version must be one of {sorted(VALID_ROS_VERSIONS)}")

    nodes = tuple(_parse_node(item) for item in raw.get("nodes", []) or [])
    monitors = tuple(_parse_monitor(item) for item in raw.get("monitors", []) or [])
    if not monitors:
        raise ConfigError("configuration must define at least one monitor")
    return ProjectConfig(
        path=raw.get("path"),
        ros_version=selected_ros_version,
        nodes=nodes,
        monitors=monitors,
    )


def _unwrap(item: Any, key: str) -> dict[str, Any]:
    if not isinstance(item, dict):
        raise ConfigError(f"{key} entries must be dictionaries")
    if key in item:
        item = item[key]
    if not isinstance(item, dict):
        raise ConfigError(f"{key} entry must be a dictionary")
    return item


def _parse_node(item: Any) -> NodeSpec:
    data = _unwrap(item, "node")
    name = _required_str(data, "name", "node")
    return NodeSpec(name=name, package=_optional_str(data, "package"), path=_optional_str(data, "path"))


def _parse_monitor(item: Any) -> MonitorSpec:
    data = _unwrap(item, "monitor")
    monitor_id = _required_str(data, "id", "monitor")
    if not re.match(r"^[A-Za-z_][0-9A-Za-z_]*$", monitor_id):
        raise ConfigError(f"monitor id {monitor_id!r} is not a valid Python identifier")
    log = _required_str(data, "log", f"monitor {monitor_id}")
    oracle = _parse_oracle(data.get("oracle"))
    status = _parse_status(data.get("status"))
    topics = tuple(_parse_interface(item, "topic") for item in data.get("topics", []) or [])
    services = tuple(_parse_interface(item, "service") for item in data.get("services", []) or [])
    if not topics and not services:
        raise ConfigError(f"monitor {monitor_id} must define at least one topic or service")
    return MonitorSpec(
        id=monitor_id,
        log=log,
        topics=topics,
        services=services,
        oracle=oracle,
        silent=bool(data.get("silent", False)),
        warning=int(data.get("warning", 1)),
        status=status,
    )


def _parse_oracle(raw: Any) -> OracleSpec | None:
    if raw is None:
        return None
    if not isinstance(raw, dict):
        raise ConfigError("oracle must be a dictionary")
    action = str(raw.get("action", "nothing"))
    if action not in VALID_ORACLE_ACTIONS:
        raise ConfigError(f"oracle action must be one of {sorted(VALID_ORACLE_ACTIONS)}")
    return OracleSpec(
        url=str(raw.get("url", "127.0.0.1")),
        port=int(raw.get("port", 8080)),
        action=action,
        timeout=float(raw.get("timeout", 2.0)),
    )


def _parse_status(raw: Any) -> StatusSpec:
    if raw is None:
        return StatusSpec()
    if not isinstance(raw, dict):
        raise ConfigError("status must be a dictionary")
    return StatusSpec(enabled=bool(raw.get("enabled", True)), log=str(raw.get("log", "rosmonitoring-status.jsonl")))


def _parse_interface(item: Any, kind: str) -> InterfaceSpec:
    if not isinstance(item, dict):
        raise ConfigError(f"{kind} entries must be dictionaries")
    name = _required_str(item, "name", kind)
    msg_type = _required_str(item, "type", kind)
    action = str(item.get("action", "log"))
    if action not in VALID_ACTIONS:
        raise ConfigError(f"{kind} {name} action must be one of {sorted(VALID_ACTIONS)}")

    ordering = _parse_ordering(item)
    return InterfaceSpec(
        name=name,
        type=msg_type,
        action=action,
        kind=kind,
        publishers=tuple(str(v) for v in item.get("publishers", []) or []),
        subscribers=tuple(str(v) for v in item.get("subscribers", []) or []),
        ordering=ordering,
    )


def _parse_ordering(item: dict[str, Any]) -> OrderingSpec:
    raw = item.get("ordering")
    legacy_ordered = bool(item.get("ordered", False))
    if raw is None:
        return OrderingSpec(enabled=legacy_ordered)
    if isinstance(raw, bool):
        return OrderingSpec(enabled=raw)
    if not isinstance(raw, dict):
        raise ConfigError("ordering must be a boolean or dictionary")
    return OrderingSpec(
        enabled=bool(raw.get("enabled", legacy_ordered)),
        source=str(raw.get("source", raw.get("order_by", "header_stamp"))),
        max_delay_ms=int(raw.get("max_delay_ms", 0)),
    )


def _required_str(data: dict[str, Any], key: str, context: str) -> str:
    value = data.get(key)
    if value is None or str(value).strip() == "":
        raise ConfigError(f"{context} requires {key}")
    return str(value)


def _optional_str(data: dict[str, Any], key: str) -> str | None:
    value = data.get(key)
    if value is None:
        return None
    return str(value)
