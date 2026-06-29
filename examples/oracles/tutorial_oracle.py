#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import json
from pathlib import Path

import websockets
from websockets.exceptions import ConnectionClosed


def verdict_for(event: dict) -> str:
    if event.get("topic") == "chatter" and event.get("data") == "drop":
        return "currently_false"
    if event.get("service") == "add_two_ints":
        request = event.get("request") or {}
        if request.get("a", 0) < 0:
            return "currently_false"
    return "currently_true"


async def serve(port: int, log_path: Path) -> None:
    log_path.parent.mkdir(parents=True, exist_ok=True)

    async def handler(websocket):
        try:
            async for message in websocket:
                event = json.loads(message)
                verdict = verdict_for(event)
                with log_path.open("a", encoding="utf-8") as handle:
                    handle.write(json.dumps({"event": event, "verdict": verdict}, sort_keys=True) + "\n")
                await websocket.send(json.dumps({"verdict": verdict}))
        except ConnectionClosed:
            return

    async with websockets.serve(handler, "127.0.0.1", port, ping_interval=None):
        await asyncio.Future()


def main() -> None:
    parser = argparse.ArgumentParser(description="Small tutorial oracle for ROSMonitoring examples.")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--log", type=Path, default=Path("./logs/oracle.jsonl"))
    args = parser.parse_args()
    asyncio.run(serve(args.port, args.log))


if __name__ == "__main__":
    main()
