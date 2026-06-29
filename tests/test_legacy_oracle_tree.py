from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_trusted_legacy_oracle_tree_is_present():
    assert (ROOT / "oracle" / "RMLOracle" / "prolog" / "online_monitor.pl").is_file()
    assert (ROOT / "oracle" / "RMLOracle" / "rml" / "test.rml").is_file()
    assert (ROOT / "oracle" / "TLOracle" / "oracle.py").is_file()
    assert (ROOT / "oracle" / "LamaConvOracle" / "oracle.py").is_file()


def test_rml_examples_document_legacy_flat_topic_events():
    source = (ROOT / "oracle" / "RMLOracle" / "rml" / "test.rml").read_text(encoding="utf-8")
    assert "{topic:'chatter', data:'hello'}" in source
    assert "{topic:'count', data:val}" in source


def test_tutorial_oracle_rejects_only_documented_demo_cases():
    from examples.oracles.tutorial_oracle import verdict_for

    assert verdict_for({"topic": "chatter", "data": "hello"}) == "currently_true"
    assert verdict_for({"topic": "chatter", "data": "drop"}) == "currently_false"
    assert verdict_for({"service": "add_two_ints", "request": {"a": 2, "b": 3}}) == "currently_true"
    assert verdict_for({"service": "add_two_ints", "request": {"a": -1, "b": 7}}) == "currently_false"


def test_lamaconv_oracle_prefers_bundled_legacy_websocket_server():
    source = (ROOT / "oracle" / "LamaConvOracle" / "oracle.py").read_text(encoding="utf-8")
    assert "sys.path.insert(0, _tl_oracle_path)" in source
    assert "sys.path.remove(_tl_oracle_path)" in source
    assert "message_dict['verdict'] = 'unknown'" in source
    message_received = source[source.index("def message_received") : source.index("def monitor_result_callback")]
    assert message_received.count("self.check_event(message)") == 1
