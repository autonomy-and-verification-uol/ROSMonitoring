from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .config import ConfigError, load_config
from .generator import generate_project
from .status import serve_status


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="rosmonitoring")
    subparsers = parser.add_subparsers(dest="command", required=True)

    validate_parser = subparsers.add_parser("validate", help="validate a ROSMonitoring YAML file")
    validate_parser.add_argument("config")
    validate_parser.add_argument("--ros-version", choices=["ros1", "ros2"])

    generate_parser = subparsers.add_parser("generate", help="generate monitor package files")
    generate_parser.add_argument("config")
    generate_parser.add_argument("--ros-version", choices=["ros1", "ros2"])
    generate_parser.add_argument("--output", default="generated")

    status_parser = subparsers.add_parser("status", help="serve the browser status dashboard")
    status_parser.add_argument("--status-log", default="rosmonitoring-status.jsonl")
    status_parser.add_argument("--host", default="127.0.0.1")
    status_parser.add_argument("--port", type=int, default=8765)

    args = parser.parse_args(argv)
    try:
        if args.command == "validate":
            config = load_config(args.config, ros_version=args.ros_version)
            print(f"valid: {len(config.monitors)} monitor(s)")
            return 0
        if args.command == "generate":
            config = load_config(args.config, ros_version=args.ros_version)
            written = generate_project(config, Path(args.output), ros_version=args.ros_version)
            print(f"generated {len(written)} file(s) under {Path(args.output).resolve()}")
            return 0
        if args.command == "status":
            serve_status(args.status_log, host=args.host, port=args.port)
            return 0
    except ConfigError as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
