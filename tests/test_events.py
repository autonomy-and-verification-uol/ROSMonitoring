from rosmonitoring.events import OrderedEventBuffer, message_to_dict, source_time


class Stamp:
    sec = 10
    nanosec = 500


class Header:
    stamp = Stamp()


class Message:
    __slots__ = ("header", "data")

    def __init__(self):
        self.header = Header()
        self.data = "ok"


def test_message_to_dict_uses_slots():
    assert message_to_dict(Message())["data"] == "ok"


def test_message_to_dict_handles_nested_plain_values_and_private_state():
    class Plain:
        def __init__(self):
            self.visible = [b"abc", {"nested": 3}]
            self._hidden = "ignored"

    assert message_to_dict(Plain()) == {"visible": ["abc", {"nested": 3}]}


def test_source_time_prefers_header_stamp():
    assert source_time(Message(), fallback=1.0) == 10.0000005


def test_source_time_accepts_ros1_and_dict_stamps():
    assert source_time({"header": {"stamp": {"secs": 4, "nsecs": 25}}}, fallback=1.0) == 4.000000025
    assert source_time({"stamp": {"sec": 8, "nanosec": 9}}, fallback=1.0) == 8.000000009


def test_ordered_buffer_flushes_by_source_time():
    buffer = OrderedEventBuffer()
    buffer.push({"source_time": 2.0, "value": "second"})
    buffer.push({"source_time": 1.0, "value": "first"})
    assert [event["value"] for event in buffer.flush_ready()] == ["first", "second"]


def test_ordered_buffer_uses_source_time_watermark_for_delayed_flush():
    buffer = OrderedEventBuffer(max_delay_ms=10_000)
    buffer.push({"source_time": 20.0, "value": "second"})
    assert buffer.flush_ready() == []
    buffer.push({"source_time": 10.0, "value": "first"})
    assert [event["value"] for event in buffer.flush_ready()] == ["first"]
    assert [event["value"] for event in buffer.flush_all()] == ["second"]


def test_ordered_buffer_preserves_arrival_order_for_equal_source_time():
    buffer = OrderedEventBuffer()
    buffer.push({"source_time": 1.0, "value": "first"})
    buffer.push({"source_time": 1.0, "value": "second"})
    assert [event["value"] for event in buffer.flush_all()] == ["first", "second"]


def test_ordered_buffer_stress_flushes_large_out_of_order_trace_deterministically():
    buffer = OrderedEventBuffer()
    values = list(range(500))
    # Deterministic pseudo-shuffle: enough disorder to catch heap/sequence bugs
    # without making the test random.
    for value in sorted(values, key=lambda item: (item * 137) % 503):
        buffer.push({"source_time": float(value), "value": value})

    assert [event["value"] for event in buffer.flush_all()] == values
    assert len(buffer) == 0


def test_ordered_buffer_delayed_flush_handles_dense_watermark_stress():
    buffer = OrderedEventBuffer(max_delay_ms=50)
    released = []

    for value in range(100):
        buffer.push({"source_time": value / 1000.0, "value": value})
        released.extend(event["value"] for event in buffer.flush_ready())

    released.extend(event["value"] for event in buffer.flush_all())

    assert released == list(range(100))


def test_message_to_dict_converts_mixed_ros_like_payloads():
    class SlotStamp:
        __slots__ = ("sec", "nanosec")

        def __init__(self):
            self.sec = 10
            self.nanosec = 500

    class SlotHeader:
        __slots__ = ("stamp",)

        def __init__(self):
            self.stamp = SlotStamp()

    class Nested:
        __slots__ = ("numbers", "blob")

        def __init__(self):
            self.numbers = (1, 2, {"three": 3})
            self.blob = b"payload"

    class Outer:
        __slots__ = ("header", "nested", "items")

        def __init__(self):
            self.header = SlotHeader()
            self.nested = Nested()
            self.items = [Nested(), {"raw": b"bytes"}]

    assert message_to_dict(Outer()) == {
        "header": {"stamp": {"nanosec": 500, "sec": 10}},
        "nested": {"blob": "payload", "numbers": [1, 2, {"three": 3}]},
        "items": [
            {"blob": "payload", "numbers": [1, 2, {"three": 3}]},
            {"raw": "bytes"},
        ],
    }
