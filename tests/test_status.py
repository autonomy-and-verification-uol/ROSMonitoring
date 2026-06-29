import json

from rosmonitoring.status import HTML, dashboard_payload, read_jsonl_tail, read_status


def test_read_status_aggregates_events(tmp_path):
    log = tmp_path / "status.jsonl"
    rows = [
        {"monitor": "m0", "status": "started", "time": 1.0},
        {"monitor": "m0", "status": "event", "time": 2.0, "interface": "/a", "verdict": True},
        {"monitor": "m0", "status": "event", "time": 3.0, "interface": "/a", "verdict": False},
    ]
    log.write_text("\n".join(json.dumps(row) for row in rows), encoding="utf-8")

    status = read_status(log)
    assert status["events"] == 2
    assert status["violations"] == 1
    assert status["monitors"]["m0"]["interfaces"]["/a"] == 2


def test_read_status_tolerates_missing_and_corrupt_logs(tmp_path):
    missing = tmp_path / "missing.jsonl"
    assert read_status(missing) == {"monitors": {}, "events": 0, "violations": 0}

    log = tmp_path / "status.jsonl"
    log.write_text(
        "\n".join(
            [
                "{not-json",
                json.dumps({"monitor": "m0", "status": "event", "interface": "/a", "verdict": False}),
                "",
            ]
        ),
        encoding="utf-8",
    )
    assert read_status(log)["violations"] == 1


def test_read_jsonl_tail_skips_bad_rows_and_limits_results(tmp_path):
    log = tmp_path / "events.jsonl"
    log.write_text(
        "\n".join(
            [
                json.dumps({"seq": 1}),
                "bad-json",
                json.dumps({"seq": 2}),
                json.dumps({"seq": 3}),
            ]
        ),
        encoding="utf-8",
    )

    assert read_jsonl_tail(log, limit=2) == [{"seq": 2}, {"seq": 3}]


def test_dashboard_payload_combines_status_events_and_config(tmp_path):
    status_log = tmp_path / "status.jsonl"
    event_log = tmp_path / "monitor.jsonl"
    status_log.write_text(
        json.dumps({"monitor": "m0", "status": "event", "interface": "/topic", "verdict": True}) + "\n",
        encoding="utf-8",
    )
    event_log.write_text(json.dumps({"topic": "topic", "data": "ok"}) + "\n", encoding="utf-8")

    payload = dashboard_payload(
        status_log,
        event_log_path=event_log,
        monitor_config={"id": "m0", "interfaces": [{"name": "/topic"}]},
    )

    assert payload["status"]["events"] == 1
    assert payload["recent_events"] == [{"topic": "topic", "data": "ok", "_monitor": "m0"}]
    assert payload["monitor_config"]["id"] == "m0"


def test_dashboard_payload_collects_event_logs_advertised_by_multiple_monitors(tmp_path):
    status_log = tmp_path / "status.jsonl"
    m0_log = tmp_path / "m0.jsonl"
    m1_log = tmp_path / "m1.jsonl"
    status_log.write_text(
        "\n".join(
            [
                json.dumps({"monitor": "m0", "status": "started", "log_path": str(m0_log), "interfaces": [{"name": "/a"}]}),
                json.dumps({"monitor": "m1", "status": "started", "log_path": str(m1_log), "interfaces": [{"name": "/s"}]}),
            ]
        ),
        encoding="utf-8",
    )
    m0_log.write_text(json.dumps({"topic": "a", "time": 1.0}) + "\n", encoding="utf-8")
    m1_log.write_text(json.dumps({"service": "s", "time": 2.0, "request": {"x": 1}}) + "\n", encoding="utf-8")

    payload = dashboard_payload(status_log)

    assert payload["event_logs"] == {"m0": str(m0_log), "m1": str(m1_log)}
    assert payload["monitor_configs"]["m0"]["interfaces"] == [{"name": "/a"}]
    assert payload["monitor_configs"]["m1"]["interfaces"] == [{"name": "/s"}]
    assert payload["recent_events_by_monitor"]["m0"] == [{"topic": "a", "time": 1.0, "_monitor": "m0"}]
    assert payload["recent_events_by_monitor"]["m1"] == [
        {"service": "s", "time": 2.0, "request": {"x": 1}, "_monitor": "m1"}
    ]
    assert [event["_monitor"] for event in payload["recent_events"]] == ["m0", "m1"]


def test_dashboard_payload_limits_and_orders_large_multi_monitor_event_sets(tmp_path):
    status_log = tmp_path / "status.jsonl"
    monitor_logs = {}
    status_rows = []
    for monitor_index in range(3):
        monitor = f"m{monitor_index}"
        event_log = tmp_path / f"{monitor}.jsonl"
        monitor_logs[monitor] = event_log
        status_rows.append(
            {
                "monitor": monitor,
                "status": "started",
                "log_path": str(event_log),
                "interfaces": [{"name": f"/topic_{monitor_index}", "kind": "topic"}],
            }
        )
        rows = []
        for event_index in range(90):
            rows.append(
                json.dumps(
                    {
                        "topic": f"topic_{monitor_index}",
                        "time": monitor_index * 1000 + event_index,
                        "data": f"{monitor}-{event_index}",
                    }
                )
            )
        event_log.write_text("\n".join(rows) + "\n", encoding="utf-8")
    status_log.write_text("\n".join(json.dumps(row) for row in status_rows), encoding="utf-8")

    payload = dashboard_payload(status_log)

    assert set(payload["event_logs"]) == {"m0", "m1", "m2"}
    assert len(payload["recent_events_by_monitor"]["m0"]) == 90
    assert len(payload["recent_events"]) == 200
    assert payload["recent_events"][0]["time"] == 70
    assert payload["recent_events"][-1]["time"] == 2089
    assert payload["recent_events"][-1]["_monitor"] == "m2"


def test_read_status_stress_counts_many_monitors_interfaces_and_violations(tmp_path):
    status_log = tmp_path / "status.jsonl"
    rows = []
    expected_violations = 0
    for monitor_index in range(5):
        rows.append({"monitor": f"m{monitor_index}", "status": "started", "time": monitor_index})
        for event_index in range(80):
            verdict = event_index % 7 != 0
            if not verdict:
                expected_violations += 1
            rows.append(
                {
                    "monitor": f"m{monitor_index}",
                    "status": "event",
                    "interface": f"/iface_{event_index % 4}",
                    "time": monitor_index * 1000 + event_index,
                    "verdict": verdict,
                }
            )
    status_log.write_text("\n".join(json.dumps(row) for row in rows), encoding="utf-8")

    status = read_status(status_log)

    assert status["events"] == 400
    assert status["violations"] == expected_violations
    assert set(status["monitors"]) == {f"m{index}" for index in range(5)}
    for monitor in status["monitors"].values():
        assert monitor["events"] == 80
        assert sum(monitor["interfaces"].values()) == 80


def test_dashboard_html_shows_payload_separately_from_raw_json():
    assert "monitorSelect" in HTML
    assert "All monitors" in HTML
    assert "Statistics" in HTML
    assert "Sequence" in HTML
    assert "Observed Sequence" in HTML
    assert "function eventPayload" in HTML
    assert "function renderStatistics" in HTML
    assert "function renderSequence" in HTML
    assert "function observedEndpoint" in HTML
    assert "function forwardedEndpoint" in HTML
    assert "Event Timeline" in HTML
    assert "Events by Monitor" in HTML
    assert "forwarded by filter/proxy" in HTML
    assert "negative verdict or blocked event" in HTML
    assert "logged violation" in HTML
    assert "function decisionLabel" in HTML
    assert "verdict ${esc(shortLabel" in HTML
    assert "forward ${esc(direction)}" in HTML
    assert "Raw Event" in HTML
    assert "Oracle Verdict" in HTML
    assert "Payload" in HTML
