from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
from typing import Any
from urllib.parse import urlparse


def read_status(log_path: str | Path) -> dict[str, Any]:
    path = Path(log_path)
    monitors: dict[str, dict[str, Any]] = {}
    events = 0
    violations = 0
    if not path.exists():
        return {"monitors": monitors, "events": events, "violations": violations}
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                item = json.loads(line)
            except json.JSONDecodeError:
                continue
            monitor_id = item.get("monitor", "unknown")
            monitor = monitors.setdefault(
                monitor_id,
                {"status": "unknown", "last_seen": None, "events": 0, "violations": 0, "interfaces": {}},
            )
            monitor["status"] = item.get("status", monitor["status"])
            monitor["last_seen"] = item.get("time", monitor["last_seen"])
            if item.get("status") == "event":
                events += 1
                monitor["events"] += 1
                interface = item.get("interface", "unknown")
                monitor["interfaces"][interface] = monitor["interfaces"].get(interface, 0) + 1
                if item.get("verdict") is False:
                    violations += 1
                    monitor["violations"] += 1
    return {"monitors": monitors, "events": events, "violations": violations}


def read_jsonl_tail(log_path: str | Path, limit: int = 200) -> list[dict[str, Any]]:
    path = Path(log_path)
    if not path.exists():
        return []
    rows: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            if not line.strip():
                continue
            try:
                rows.append(json.loads(line))
            except json.JSONDecodeError:
                continue
    return rows[-limit:]


def monitor_event_log_paths(status_rows: list[dict[str, Any]]) -> dict[str, str]:
    paths: dict[str, str] = {}
    for item in status_rows:
        monitor = item.get("monitor")
        log_path = item.get("log_path")
        if monitor and log_path:
            paths[str(monitor)] = str(log_path)
    return paths


def read_monitor_events(
    status_rows: list[dict[str, Any]],
    *,
    default_monitor: str | None = None,
    default_log_path: str | Path | None = None,
) -> dict[str, list[dict[str, Any]]]:
    paths = monitor_event_log_paths(status_rows)
    if default_monitor and default_log_path:
        paths.setdefault(default_monitor, str(default_log_path))
    events: dict[str, list[dict[str, Any]]] = {}
    for monitor, path in sorted(paths.items()):
        rows = []
        for event in read_jsonl_tail(path):
            annotated = dict(event)
            annotated["_monitor"] = monitor
            rows.append(annotated)
        events[monitor] = rows
    return events


def monitor_configs_from_status_rows(
    status_rows: list[dict[str, Any]],
    current_config: dict[str, Any] | None = None,
) -> dict[str, dict[str, Any]]:
    configs: dict[str, dict[str, Any]] = {}
    if current_config and current_config.get("id"):
        configs[str(current_config["id"])] = dict(current_config)
    for item in status_rows:
        monitor = item.get("monitor")
        if not monitor:
            continue
        config = configs.setdefault(str(monitor), {"id": str(monitor)})
        if item.get("ros_version"):
            config["ros_version"] = item["ros_version"]
        if item.get("log_path"):
            config["log_path"] = item["log_path"]
        if item.get("interfaces"):
            config["interfaces"] = item["interfaces"]
    return configs


def dashboard_payload(
    status_log_path: str | Path,
    *,
    event_log_path: str | Path | None = None,
    monitor_config: dict[str, Any] | None = None,
) -> dict[str, Any]:
    recent_status = read_jsonl_tail(status_log_path)
    default_monitor = None
    if monitor_config:
        default_monitor = monitor_config.get("id")
    events_by_monitor = read_monitor_events(
        recent_status,
        default_monitor=default_monitor,
        default_log_path=event_log_path,
    )
    recent_events = []
    for rows in events_by_monitor.values():
        recent_events.extend(rows)
    recent_events.sort(key=lambda item: item.get("time", 0))
    return {
        "status": read_status(status_log_path),
        "status_log": str(status_log_path),
        "event_log": str(event_log_path) if event_log_path else None,
        "event_logs": monitor_event_log_paths(recent_status),
        "recent_status": recent_status,
        "recent_events": recent_events[-200:],
        "recent_events_by_monitor": events_by_monitor,
        "monitor_config": monitor_config or {},
        "monitor_configs": monitor_configs_from_status_rows(recent_status, monitor_config),
    }


HTML = """<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ROSMonitoring Dashboard</title>
  <style>
    :root { color-scheme: light; --bg: #f4f6f8; --panel: #fff; --ink: #16202a; --muted: #647181; --line: #d9e0e7; --accent: #0b5cad; --bad: #b4232f; --ok: #146c43; }
    * { box-sizing: border-box; }
    body { font-family: system-ui, sans-serif; margin: 0; background: var(--bg); color: var(--ink); }
    header { padding: 16px 24px; background: #101820; color: white; display: flex; align-items: center; justify-content: space-between; gap: 16px; }
    header h1 { margin: 0; font-size: 20px; }
    header .meta { color: #cbd5df; font-size: 13px; }
    main { padding: 18px 24px 28px; }
    .cards { display: grid; grid-template-columns: repeat(4, minmax(150px, 1fr)); gap: 12px; margin-bottom: 14px; }
    .card, .panel { background: var(--panel); border: 1px solid var(--line); border-radius: 8px; }
    .card { padding: 14px; }
    .card .label { color: var(--muted); font-size: 12px; text-transform: uppercase; letter-spacing: .04em; }
    .card .value { font-size: 26px; font-weight: 700; margin-top: 4px; }
    .tabs { display: flex; gap: 8px; margin: 12px 0; flex-wrap: wrap; }
    button, input, select { font: inherit; }
    button { border: 1px solid var(--line); background: white; border-radius: 6px; padding: 8px 10px; cursor: pointer; }
    button.active { background: var(--accent); color: white; border-color: var(--accent); }
    .toolbar { display: flex; gap: 10px; align-items: center; flex-wrap: wrap; margin-bottom: 12px; }
    input, select { border: 1px solid var(--line); border-radius: 6px; padding: 8px 10px; background: white; }
    .panel { overflow: hidden; }
    .panel h2 { margin: 0; padding: 12px 14px; font-size: 15px; border-bottom: 1px solid var(--line); background: #f9fbfc; }
    table { width: 100%; border-collapse: collapse; background: white; }
    th, td { text-align: left; padding: 10px 12px; border-bottom: 1px solid #d8dde3; }
    th { background: #eef1f5; font-weight: 650; }
    td code, pre { font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; }
    pre { margin: 0; white-space: pre-wrap; word-break: break-word; max-height: 520px; overflow: auto; padding: 12px; background: #0f1720; color: #d8e4ef; }
    .split { display: grid; grid-template-columns: repeat(2, minmax(260px, 1fr)); gap: 12px; padding: 12px; }
    .metric-grid { display: grid; grid-template-columns: repeat(4, minmax(120px, 1fr)); gap: 10px; padding: 12px; }
    .metric { border: 1px solid var(--line); border-radius: 8px; padding: 10px; background: #fbfcfd; }
    .metric .label { color: var(--muted); font-size: 12px; }
    .metric .value { font-size: 22px; font-weight: 750; margin-top: 3px; }
    .plot { border: 1px solid var(--line); border-radius: 8px; overflow: hidden; background: white; min-height: 180px; }
    .plot h3 { margin: 0; padding: 10px 12px; border-bottom: 1px solid var(--line); font-size: 13px; background: #f9fbfc; }
    .bars { padding: 12px; display: grid; gap: 8px; }
    .bar-row { display: grid; grid-template-columns: minmax(96px, 170px) 1fr 42px; align-items: center; gap: 8px; font-size: 13px; }
    .bar-track { height: 12px; border-radius: 4px; background: #e8edf2; overflow: hidden; }
    .bar-fill { height: 100%; background: var(--accent); }
    .bar-fill.bad-fill { background: var(--bad); }
    .plot svg, .sequence svg { width: 100%; display: block; }
    .sequence { padding: 12px; overflow: auto; background: white; }
    .sequence .empty, .plot .empty { color: var(--muted); padding: 12px; }
    .legend { display: flex; gap: 12px; flex-wrap: wrap; padding: 0 12px 12px; color: var(--muted); font-size: 12px; }
    .dot { width: 10px; height: 10px; border-radius: 50%; display: inline-block; margin-right: 5px; background: var(--accent); }
    .dot.bad-dot { background: var(--bad); }
    .dot.ok-dot { background: var(--ok); }
    .ok { color: var(--ok); font-weight: 650; }
    .bad { color: var(--bad); font-weight: 650; }
    .muted { color: var(--muted); }
    .hidden { display: none; }
    @media (max-width: 900px) { .cards, .metric-grid, .split { grid-template-columns: repeat(2, 1fr); } }
    @media (max-width: 560px) { .cards, .metric-grid, .split { grid-template-columns: 1fr; } header { align-items: flex-start; flex-direction: column; } }
  </style>
</head>
<body>
  <header>
    <h1>ROSMonitoring Dashboard</h1>
    <div class="meta"><span id="updated">not loaded</span> · <label><input id="auto" type="checkbox" checked> auto-refresh</label></div>
  </header>
  <main>
    <section class="cards">
      <div class="card"><div class="label">Monitors</div><div class="value" id="monitorCount">0</div></div>
      <div class="card"><div class="label">Events</div><div class="value" id="eventCount">0</div></div>
      <div class="card"><div class="label">Violations</div><div class="value bad" id="violationCount">0</div></div>
      <div class="card"><div class="label">Observed Payloads</div><div class="value" id="payloadCount">0</div></div>
    </section>
    <div class="tabs">
      <button class="active" data-tab="overview">Overview</button>
      <button data-tab="statistics">Statistics</button>
      <button data-tab="sequence">Sequence</button>
      <button data-tab="interfaces">Interfaces</button>
      <button data-tab="events">Observed Events</button>
      <button data-tab="status">Status Timeline</button>
      <button data-tab="raw">Raw JSON</button>
    </div>
    <div class="toolbar">
      <select id="monitorSelect"><option value="all">All monitors</option></select>
      <input id="filter" placeholder="Filter text, interface, verdict">
      <button id="refresh">Refresh</button>
    </div>
    <section id="overview" class="panel tab"><h2>Monitor Overview</h2><div id="overviewBody"></div></section>
    <section id="statistics" class="panel tab hidden"><h2>Statistics</h2><div id="statisticsBody"></div></section>
    <section id="sequence" class="panel tab hidden"><h2>Observed Sequence</h2><div id="sequenceBody"></div></section>
    <section id="interfaces" class="panel tab hidden"><h2>Configured Interfaces</h2><div id="interfacesBody"></div></section>
    <section id="events" class="panel tab hidden"><h2>Recent Observed Events</h2><div id="eventsBody"></div></section>
    <section id="status" class="panel tab hidden"><h2>Status Timeline</h2><div id="statusBody"></div></section>
    <section id="raw" class="panel tab hidden"><h2>Raw Dashboard Payload</h2><pre id="rawBody"></pre></section>
  </main>
  <script>
    let latest = null;
    let activeTab = 'overview';
    let selectedMonitor = 'all';
    const esc = (v) => String(v ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
    const json = (v) => esc(JSON.stringify(v, null, 2));
    function table(headers, rows) {
      if (!rows.length) return '<p class="muted" style="padding:12px">No data yet.</p>';
      return `<table><thead><tr>${headers.map(h => `<th>${esc(h)}</th>`).join('')}</tr></thead><tbody>${rows.join('')}</tbody></table>`;
    }
    function row(cells) { return `<tr>${cells.map(c => `<td>${c}</td>`).join('')}</tr>`; }
    function matchesFilter(item) {
      const q = document.getElementById('filter').value.trim().toLowerCase();
      return !q || JSON.stringify(item).toLowerCase().includes(q);
    }
    function allMonitorIds(data) {
      return Array.from(new Set([
        ...Object.keys(data.status.monitors || {}),
        ...Object.keys(data.monitor_configs || {}),
        ...(data.recent_events || []).map(item => item._monitor).filter(Boolean),
      ])).sort();
    }
    function monitorMatches(item) {
      if (selectedMonitor === 'all') return true;
      return item.monitor === selectedMonitor || item._monitor === selectedMonitor;
    }
    function renderMonitorSelect(data) {
      const select = document.getElementById('monitorSelect');
      const ids = allMonitorIds(data);
      if (selectedMonitor !== 'all' && !ids.includes(selectedMonitor)) {
        selectedMonitor = 'all';
      }
      select.innerHTML = [
        '<option value="all">All monitors</option>',
        ...ids.map(id => `<option value="${esc(id)}">${esc(id)}</option>`),
      ].join('');
      select.value = selectedMonitor;
    }
    function monitorConfigs(data) {
      const configs = {...(data.monitor_configs || {})};
      if (data.monitor_config && data.monitor_config.id && !configs[data.monitor_config.id]) {
        configs[data.monitor_config.id] = data.monitor_config;
      }
      return configs;
    }
    function eventDirection(item) {
      if (Object.prototype.hasOwnProperty.call(item, 'request')) return 'request';
      if (Object.prototype.hasOwnProperty.call(item, 'response')) return 'response';
      return item.__direction || item.direction || 'message';
    }
    function eventPayload(item) {
      if (Object.prototype.hasOwnProperty.call(item, 'payload')) return item.payload;
      if (Object.prototype.hasOwnProperty.call(item, 'request')) return item.request;
      if (Object.prototype.hasOwnProperty.call(item, 'response')) return item.response;
      if (Object.prototype.hasOwnProperty.call(item, 'data')) return item.data;
      const copy = {...item};
      ['time', 'topic', 'service', '_monitor', 'monitor', 'status', 'interface', 'kind',
       'verdict', 'verdict_raw', 'terminal', 'will_stop', 'ordered', 'oracle_response',
       'event', 'direction', 'session_id', 'ros_version'].forEach(k => delete copy[k]);
      return copy;
    }
    function verdictClass(value) {
      const text = String(value ?? '').toLowerCase();
      return text.includes('false') || text === 'violation' || text === 'violated' ? 'bad' : 'ok';
    }
    function filteredStatusEvents(data) {
      return (data.recent_status || [])
        .filter(item => item.status === 'event')
        .filter(monitorMatches)
        .filter(matchesFilter);
    }
    function filteredObservedEvents(data) {
      return (data.recent_events || [])
        .filter(monitorMatches)
        .filter(matchesFilter);
    }
    function eventKey(item) {
      return item.interface || item.topic || item.service || '';
    }
    function verdictLabel(item) {
      if (item.verdict_raw !== undefined && item.verdict_raw !== null) return String(item.verdict_raw);
      if (item.verdict === true) return 'allow';
      if (item.verdict === false) return 'block';
      return 'unknown';
    }
    function countBy(items, keyFn) {
      const counts = new Map();
      items.forEach(item => {
        const key = keyFn(item) || 'unknown';
        counts.set(key, (counts.get(key) || 0) + 1);
      });
      return Array.from(counts.entries()).sort((a, b) => b[1] - a[1] || String(a[0]).localeCompare(String(b[0])));
    }
    function barPlot(title, rows, options = {}) {
      const max = Math.max(1, ...rows.map(([, value]) => value));
      const body = rows.length ? rows.map(([label, value]) => {
        const width = Math.max(3, Math.round((value / max) * 100));
        const fill = options.bad && options.bad(label, value) ? 'bar-fill bad-fill' : 'bar-fill';
        return `<div class="bar-row"><code title="${esc(label)}">${esc(label)}</code><div class="bar-track"><div class="${fill}" style="width:${width}%"></div></div><strong>${esc(value)}</strong></div>`;
      }).join('') : '<div class="empty">No data yet.</div>';
      return `<div class="plot"><h3>${esc(title)}</h3><div class="bars">${body}</div></div>`;
    }
    function timelinePlot(events) {
      const timed = events
        .map(item => ({...item, _t: Number(item.time)}))
        .filter(item => Number.isFinite(item._t))
        .sort((a, b) => a._t - b._t);
      if (!timed.length) {
        return '<div class="plot"><h3>Event Timeline</h3><div class="empty">No timed events yet.</div></div>';
      }
      const min = timed[0]._t;
      const max = timed[timed.length - 1]._t;
      const bins = 24;
      const counts = Array.from({length: bins}, () => ({events: 0, violations: 0}));
      timed.forEach(item => {
        const index = max === min ? 0 : Math.min(bins - 1, Math.floor(((item._t - min) / (max - min)) * bins));
        counts[index].events += 1;
        if (item.verdict === false || String(item.verdict_raw || '').toLowerCase().includes('false')) {
          counts[index].violations += 1;
        }
      });
      const width = 760;
      const height = 180;
      const pad = 24;
      const innerW = width - pad * 2;
      const innerH = height - pad * 2;
      const maxCount = Math.max(1, ...counts.map(item => item.events));
      const barW = innerW / bins;
      const bars = counts.map((item, index) => {
        const h = (item.events / maxCount) * innerH;
        const badH = item.events ? (item.violations / maxCount) * innerH : 0;
        const x = pad + index * barW + 2;
        const y = height - pad - h;
        const badY = height - pad - badH;
        return `<rect x="${x}" y="${y}" width="${Math.max(2, barW - 4)}" height="${h}" rx="2" fill="#0b5cad"></rect>` +
          `<rect x="${x}" y="${badY}" width="${Math.max(2, barW - 4)}" height="${badH}" rx="2" fill="#b4232f"></rect>`;
      }).join('');
      return `<div class="plot"><h3>Event Timeline</h3><svg viewBox="0 0 ${width} ${height}" role="img" aria-label="event timeline"><line x1="${pad}" y1="${height - pad}" x2="${width - pad}" y2="${height - pad}" stroke="#9aa6b2"></line>${bars}<text x="${pad}" y="16" font-size="11" fill="#647181">${esc(min.toFixed(3))}</text><text x="${width - pad}" y="16" text-anchor="end" font-size="11" fill="#647181">${esc(max.toFixed(3))}</text></svg><div class="legend"><span><i class="dot"></i>events</span><span><i class="dot bad-dot"></i>violations</span></div></div>`;
    }
    function renderStatistics(data) {
      const statusEvents = filteredStatusEvents(data);
      const observedEvents = filteredObservedEvents(data);
      const times = statusEvents.map(item => Number(item.time)).filter(Number.isFinite);
      const duration = times.length > 1 ? Math.max(...times) - Math.min(...times) : 0;
      const violations = statusEvents.filter(item => item.verdict === false || String(item.verdict_raw || '').toLowerCase().includes('false')).length;
      const terminal = statusEvents.filter(item => item.terminal === true).length;
      const interfaces = new Set(statusEvents.map(eventKey).filter(Boolean)).size;
      const rate = duration > 0 ? (statusEvents.length / duration).toFixed(2) : '0';
      const metrics = [
        ['Selected Events', statusEvents.length],
        ['Payload Rows', observedEvents.length],
        ['Violations', violations],
        ['Terminal Verdicts', terminal],
        ['Interfaces', interfaces],
        ['Events / Second', rate],
        ['Time Span', duration ? `${duration.toFixed(2)}s` : '0s'],
        ['Selected Monitor', selectedMonitor === 'all' ? 'All' : selectedMonitor],
      ].map(([label, value]) => `<div class="metric"><div class="label">${esc(label)}</div><div class="value">${esc(value)}</div></div>`).join('');
      const monitorRows = countBy(statusEvents, item => item.monitor);
      const interfaceRows = countBy(statusEvents, eventKey);
      const verdictRows = countBy(statusEvents, verdictLabel);
      document.getElementById('statisticsBody').innerHTML =
        `<div class="metric-grid">${metrics}</div>` +
        `<div class="split">` +
        barPlot('Events by Monitor', monitorRows) +
        barPlot('Events by Interface', interfaceRows) +
        barPlot('Verdicts', verdictRows, {bad: label => String(label).toLowerCase().includes('false') || String(label).toLowerCase() === 'block'}) +
        timelinePlot(statusEvents) +
        `</div>`;
    }
    function interfaceConfigFor(data, item) {
      const config = (monitorConfigs(data)[item.monitor] || {}).interfaces || [];
      return config.find(interface => interface.name === item.interface || interface.legacy_name === item.interface) || {};
    }
    function observedEndpoint(data, item) {
      const config = interfaceConfigFor(data, item);
      const direction = item.direction || eventDirection(item);
      if (config.kind === 'topic' && config.intercepting) return config.remapped_name || item.interface || eventKey(item);
      if (config.kind === 'service' && direction === 'request') return config.remapped_name || item.interface || eventKey(item);
      if (config.kind === 'service' && direction === 'response') return config.name || item.interface || eventKey(item);
      return item.interface || eventKey(item);
    }
    function forwardedEndpoint(data, item) {
      const config = interfaceConfigFor(data, item);
      const direction = item.direction || eventDirection(item);
      const allowed = item.verdict !== false;
      if (config.kind === 'topic' && config.intercepting && config.action === 'filter' && allowed) {
        return config.name || item.interface || eventKey(item);
      }
      if (config.kind === 'service' && direction === 'request' && allowed) {
        return config.name || item.interface || eventKey(item);
      }
      if (config.kind === 'service' && direction === 'response') {
        return config.remapped_name || item.interface || eventKey(item);
      }
      return '';
    }
    function sequenceParticipants(events, data) {
      const lanes = [];
      const add = value => {
        const label = String(value || '').trim();
        if (label && !lanes.includes(label)) lanes.push(label);
      };
      events.forEach(item => {
        add(observedEndpoint(data, item));
        add(item.monitor);
        if (item.oracle_response !== null && item.oracle_response !== undefined) add('oracle');
        add(forwardedEndpoint(data, item));
      });
      return lanes;
    }
    function shortLabel(value, max = 52) {
      const text = String(value ?? '');
      return text.length > max ? text.slice(0, max - 3) + '...' : text;
    }
    function renderSequence(data) {
      const events = filteredStatusEvents(data).slice(-80);
      if (!events.length) {
        document.getElementById('sequenceBody').innerHTML = '<div class="sequence"><div class="empty">No observed events yet.</div></div>';
        return;
      }
      const lanes = sequenceParticipants(events, data);
      const laneGap = 190;
      const left = 70;
      const top = 54;
      const rowGap = 70;
      const width = Math.max(780, left * 2 + Math.max(1, lanes.length - 1) * laneGap);
      const height = top + events.length * rowGap + 70;
      const xFor = label => left + lanes.indexOf(label) * laneGap;
      const laneLines = lanes.map(label => {
        const x = xFor(label);
        return `<line x1="${x}" y1="${top - 24}" x2="${x}" y2="${height - 24}" stroke="#d9e0e7" stroke-dasharray="4 5"></line><rect x="${x - 68}" y="10" width="136" height="28" rx="6" fill="#f9fbfc" stroke="#d9e0e7"></rect><text x="${x}" y="29" text-anchor="middle" font-size="12" fill="#16202a">${esc(shortLabel(label, 18))}</text>`;
      }).join('');
      const arrows = events.map((item, index) => {
        const y = top + index * rowGap;
        const source = observedEndpoint(data, item);
        const target = forwardedEndpoint(data, item);
        const monitor = item.monitor;
        const sourceX = xFor(source);
        const monitorX = xFor(monitor);
        const verdict = verdictLabel(item);
        const bad = verdictClass(verdict) === 'bad';
        const payload = eventPayload(item);
        const direction = item.direction || eventDirection(item);
        const label = `observe ${direction} ${verdict}`;
        let eventArrow = `<line x1="${sourceX}" y1="${y}" x2="${monitorX}" y2="${y}" stroke="${bad ? '#b4232f' : '#0b5cad'}" stroke-width="2" marker-end="url(#arrow)"></line><circle cx="${monitorX}" cy="${y}" r="4" fill="${bad ? '#b4232f' : '#146c43'}"></circle><text x="${(sourceX + monitorX) / 2}" y="${y - 8}" text-anchor="middle" font-size="11" fill="#16202a">${esc(shortLabel(label, 42))}</text><text x="${(sourceX + monitorX) / 2}" y="${y + 17}" text-anchor="middle" font-size="10" fill="#647181">${esc(shortLabel(JSON.stringify(payload), 58))}</text>`;
        if (target) {
          const targetX = xFor(target);
          const fy = y + 44;
          eventArrow += `<line x1="${monitorX}" y1="${fy}" x2="${targetX}" y2="${fy}" stroke="#146c43" stroke-width="2" marker-end="url(#arrowOk)"></line><text x="${(monitorX + targetX) / 2}" y="${fy - 7}" text-anchor="middle" font-size="10" fill="#146c43">forward ${esc(direction)}</text>`;
        } else if (bad) {
          eventArrow += `<text x="${monitorX + 12}" y="${y + 5}" font-size="12" fill="#b4232f">blocked</text>`;
        }
        if (item.oracle_response === null || item.oracle_response === undefined || !lanes.includes('oracle')) {
          return eventArrow;
        }
        const oracleX = xFor('oracle');
        const oy = y + 26;
        return eventArrow + `<line x1="${oracleX}" y1="${oy}" x2="${monitorX}" y2="${oy}" stroke="#647181" stroke-dasharray="4 4" marker-end="url(#arrowMuted)"></line><text x="${(monitorX + oracleX) / 2}" y="${oy - 7}" text-anchor="middle" font-size="10" fill="#647181">verdict ${esc(shortLabel(JSON.stringify(item.oracle_response), 36))}</text>`;
      }).join('');
      document.getElementById('sequenceBody').innerHTML =
        `<div class="sequence"><svg viewBox="0 0 ${width} ${height}" style="min-width:${width}px" role="img" aria-label="observed monitor sequence"><defs><marker id="arrow" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#0b5cad"></path></marker><marker id="arrowOk" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#146c43"></path></marker><marker id="arrowMuted" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M 0 0 L 10 5 L 0 10 z" fill="#647181"></path></marker></defs>${laneLines}${arrows}</svg><div class="legend"><span><i class="dot"></i>observed by monitor</span><span><i class="dot ok-dot"></i>forwarded by filter/proxy</span><span><i class="dot bad-dot"></i>blocked verdict</span></div></div>`;
    }
    function renderOverview(data) {
      const rows = Object.entries(data.status.monitors || {})
      .filter(([id]) => selectedMonitor === 'all' || id === selectedMonitor)
      .map(([id, monitor]) => row([
        `<code>${esc(id)}</code>`,
        esc(monitor.status),
        esc(monitor.events),
        `<span class="${monitor.violations > 0 ? 'bad' : 'ok'}">${esc(monitor.violations)}</span>`,
        esc(Object.keys(monitor.interfaces || {}).join(', ')),
        esc(monitor.last_seen || '')
      ]));
      document.getElementById('overviewBody').innerHTML = table(['Monitor', 'Status', 'Events', 'Violations', 'Interfaces', 'Last Seen'], rows);
    }
    function renderInterfaces(data) {
      const rows = Object.entries(monitorConfigs(data))
      .filter(([id]) => selectedMonitor === 'all' || id === selectedMonitor)
      .flatMap(([id, config]) => (config.interfaces || []).map(item => ({...item, _monitor: id})))
      .filter(matchesFilter).map(item => row([
        `<code>${esc(item._monitor)}</code>`,
        `<code>${esc(item.name)}</code>`,
        esc(item.kind),
        esc(item.type_fqn || item.type),
        esc(item.action),
        esc(item.ordered),
        `<code>${esc(item.remapped_name)}</code>`,
        esc(item.intercepting)
      ]));
      document.getElementById('interfacesBody').innerHTML = table(['Monitor', 'Name', 'Kind', 'Type', 'Action', 'Ordered', 'Remapped', 'Intercepting'], rows);
    }
    function renderEvents(data) {
      const rows = (data.recent_events || []).filter(monitorMatches).filter(matchesFilter).slice().reverse().map(item => row([
        esc(item.time || ''),
        `<code>${esc(item._monitor || '')}</code>`,
        esc(item.topic || item.service || ''),
        esc(eventDirection(item)),
        `<pre>${json(eventPayload(item))}</pre>`,
        `<pre>${json(item)}</pre>`
      ]));
      document.getElementById('eventsBody').innerHTML = table(['Time', 'Monitor', 'Topic/Service', 'Direction', 'Payload', 'Raw Event'], rows);
    }
    function renderStatus(data) {
      const rows = (data.recent_status || []).filter(monitorMatches).filter(matchesFilter).slice().reverse().map(item => row([
        esc(item.time || ''),
        `<code>${esc(item.monitor || '')}</code>`,
        esc(item.status || ''),
        esc(item.interface || ''),
        item.verdict === false ? '<span class="bad">block</span>' : esc(item.verdict === true ? 'allow' : ''),
        item.verdict_raw ? `<span class="${verdictClass(item.verdict_raw)}">${esc(item.verdict_raw)}</span>` : '',
        esc(item.terminal ?? ''),
        esc(item.will_stop ?? ''),
        `<pre>${json(eventPayload(item))}</pre>`,
        `<pre>${json(item)}</pre>`
      ]));
      document.getElementById('statusBody').innerHTML = table(['Time', 'Monitor', 'Status', 'Interface', 'Decision', 'Oracle Verdict', 'Terminal', 'Will Stop', 'Payload', 'Raw'], rows);
    }
    function render(data) {
      latest = data;
      renderMonitorSelect(data);
      document.getElementById('monitorCount').textContent = Object.keys(data.status.monitors || {}).length;
      document.getElementById('eventCount').textContent = data.status.events || 0;
      document.getElementById('violationCount').textContent = data.status.violations || 0;
      document.getElementById('payloadCount').textContent = (data.recent_events || []).length;
      document.getElementById('updated').textContent = new Date().toLocaleTimeString();
      renderOverview(data);
      renderStatistics(data);
      renderSequence(data);
      renderInterfaces(data);
      renderEvents(data);
      renderStatus(data);
      document.getElementById('rawBody').textContent = JSON.stringify(data, null, 2);
    }
    async function refresh() {
      const response = await fetch('/api/dashboard');
      const data = await response.json();
      render(data);
    }
    document.querySelectorAll('.tabs button').forEach(btn => btn.addEventListener('click', () => {
      activeTab = btn.dataset.tab;
      document.querySelectorAll('.tabs button').forEach(b => b.classList.toggle('active', b === btn));
      document.querySelectorAll('.tab').forEach(tab => tab.classList.toggle('hidden', tab.id !== activeTab));
    }));
    document.getElementById('refresh').addEventListener('click', refresh);
    document.getElementById('filter').addEventListener('input', () => latest && render(latest));
    document.getElementById('monitorSelect').addEventListener('change', event => {
      selectedMonitor = event.target.value;
      if (latest) render(latest);
    });
    refresh();
    setInterval(() => { if (document.getElementById('auto').checked) refresh(); }, 1000);
  </script>
</body>
</html>
"""


def serve_status(log_path: str | Path, host: str = "127.0.0.1", port: int = 8765) -> None:
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802 - stdlib API
            path = urlparse(self.path).path
            if path == "/api/status":
                body = json.dumps(read_status(log_path)).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/api/dashboard":
                body = json.dumps(dashboard_payload(log_path)).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, format, *args):  # noqa: A003
            return

    server = ThreadingHTTPServer((host, port), Handler)
    print(f"ROSMonitoring status dashboard: http://{host}:{port}")
    server.serve_forever()
