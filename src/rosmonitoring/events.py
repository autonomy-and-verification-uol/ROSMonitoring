from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import itertools
import time
from typing import Any, Callable


def message_to_dict(message: Any) -> Any:
    """Convert ROS-like messages or plain Python values to JSON-compatible data."""
    if message is None or isinstance(message, (str, int, float, bool)):
        return message
    if isinstance(message, bytes):
        return message.decode("utf-8", errors="replace")
    if isinstance(message, (list, tuple)):
        return [message_to_dict(item) for item in message]
    if isinstance(message, dict):
        return {str(key): message_to_dict(value) for key, value in message.items()}
    slots = getattr(message, "__slots__", None)
    if slots:
        return {slot.lstrip("_"): message_to_dict(getattr(message, slot)) for slot in slots if hasattr(message, slot)}
    if hasattr(message, "__dict__"):
        return {
            str(key): message_to_dict(value)
            for key, value in vars(message).items()
            if not key.startswith("_")
        }
    return str(message)


def stamp_to_float(stamp: Any) -> float | None:
    if stamp is None:
        return None
    if isinstance(stamp, (int, float)):
        return float(stamp)
    sec = getattr(stamp, "sec", getattr(stamp, "secs", None))
    nanosec = getattr(stamp, "nanosec", getattr(stamp, "nsecs", None))
    if sec is not None:
        return float(sec) + (float(nanosec or 0) / 1_000_000_000.0)
    if isinstance(stamp, dict):
        sec = stamp.get("sec", stamp.get("secs"))
        nanosec = stamp.get("nanosec", stamp.get("nsecs", 0))
        if sec is not None:
            return float(sec) + (float(nanosec or 0) / 1_000_000_000.0)
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


@dataclass(order=True)
class BufferedEvent:
    source_time: float
    sequence: int
    event: dict[str, Any] = field(compare=False)


class OrderedEventBuffer:
    """Small deterministic reorder buffer keyed by publication/source timestamp."""

    def __init__(self, max_delay_ms: int = 0, clock: Callable[[], float] = time.time):
        self.max_delay = max(0, max_delay_ms) / 1000.0
        self.clock = clock
        self._sequence = itertools.count()
        self._heap: list[BufferedEvent] = []
        self._watermark: float | None = None

    def push(self, event: dict[str, Any]) -> None:
        event_time = float(event["source_time"])
        if self._watermark is None or event_time > self._watermark:
            self._watermark = event_time
        heapq.heappush(
            self._heap,
            BufferedEvent(event_time, next(self._sequence), event),
        )

    def flush_ready(self) -> list[dict[str, Any]]:
        if self.max_delay == 0:
            return self.flush_all()
        if self._watermark is None:
            return []
        threshold = self._watermark - self.max_delay
        ready: list[dict[str, Any]] = []
        while self._heap and self._heap[0].source_time <= threshold:
            ready.append(heapq.heappop(self._heap).event)
        return ready

    def flush_all(self) -> list[dict[str, Any]]:
        ready: list[dict[str, Any]] = []
        while self._heap:
            ready.append(heapq.heappop(self._heap).event)
        return ready

    def __len__(self) -> int:
        return len(self._heap)
