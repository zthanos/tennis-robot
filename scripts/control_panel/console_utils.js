/* Shared console helpers (window.ConsoleUtils).
 *
 * Pure formatting + small DOM-write helpers and court-geometry derivations used
 * across the console views. Extracted from app.js so feature modules
 * (collection_map, survey_view, …) can share them without duplicating logic.
 */
window.ConsoleUtils = (() => {
  "use strict";

  function fmt(value, suffix = "") {
    if (value === null || value === undefined || Number.isNaN(value)) return "none";
    if (typeof value === "number") return `${value.toFixed(Math.abs(value) >= 10 ? 1 : 2)}${suffix}`;
    return String(value);
  }

  function canonicalFenceBounds(bounds) {
    const corners = (bounds?.canonical_fence_model || {}).corners || {};
    const pts = Object.values(corners).filter(p =>
      Number.isFinite(p?.x_m) && Number.isFinite(p?.y_m)
    );
    if (!pts.length) return {};
    const xs = pts.map(p => p.x_m);
    const ys = pts.map(p => p.y_m);
    return {
      west_x: Math.min(...xs),
      east_x: Math.max(...xs),
      south_y: Math.min(...ys),
      north_y: Math.max(...ys),
    };
  }

  function courtFrameModel(bounds) {
    if (!bounds || bounds.schema !== "court_knowledge_model/v2") return null;
    const frame = (bounds.map_artifact || {}).court_frame || {};
    const center = frame.center || bounds.net?.center || {};
    const axisLength = frame.axis_length || bounds.net?.axis_length || {};
    const axisWidth = frame.axis_width || bounds.net?.axis_width || {};
    const lines = bounds.court?.lines_court_frame || {};
    const baselines = Array.isArray(lines.baselines_x) ? lines.baselines_x.map(Number).filter(Number.isFinite).sort((a, b) => a - b) : [];
    const service = Array.isArray(lines.service_x) ? lines.service_x.map(Number).filter(Number.isFinite).sort((a, b) => a - b) : [];
    const sidelines = Array.isArray(lines.sidelines_y) ? lines.sidelines_y.map(Number).filter(Number.isFinite).sort((a, b) => a - b) : [];
    const ok = [center.x_m, center.y_m, axisLength.x_m, axisLength.y_m, axisWidth.x_m, axisWidth.y_m].every(Number.isFinite);
    if (!ok || baselines.length < 2 || service.length < 2 || sidelines.length < 2) return null;
    const toMap = (x, y) => ({
      x_m: center.x_m + x * axisLength.x_m + y * axisWidth.x_m,
      y_m: center.y_m + x * axisLength.y_m + y * axisWidth.y_m,
    });
    const xs = [baselines[0], baselines[baselines.length - 1]];
    const ys = [sidelines[0], sidelines[sidelines.length - 1]];
    const corners = [
      toMap(xs[0], ys[0]), toMap(xs[1], ys[0]),
      toMap(xs[1], ys[1]), toMap(xs[0], ys[1]),
    ];
    return {
      baselines_x: xs,
      service_x: [service[0], service[service.length - 1]],
      sidelines_y: ys,
      toMap,
      bounds: {
        min_x: Math.min(...corners.map(p => p.x_m)),
        max_x: Math.max(...corners.map(p => p.x_m)),
        min_y: Math.min(...corners.map(p => p.y_m)),
        max_y: Math.max(...corners.map(p => p.y_m)),
        net_x: center.x_m,
      },
    };
  }

  function timeText(seconds) {
    if (!seconds) return "0.0s";
    if (seconds < 90) return `${seconds.toFixed(1)}s`;
    return `${Math.floor(seconds / 60)}m ${Math.floor(seconds % 60)}s`;
  }

  function dateText(ts) {
    if (!ts) return "none";
    return new Date(ts * 1000).toLocaleTimeString();
  }

  function escapeHtml(value) {
    return String(value)
      .replaceAll("&", "&amp;")
      .replaceAll("<", "&lt;")
      .replaceAll(">", "&gt;")
      .replaceAll('"', "&quot;")
      .replaceAll("'", "&#039;");
  }

  // --- DOM-write helpers (null-safe so render() tolerates lazy-loaded views) ---
  function setText(id, value) {
    const el = document.getElementById(id);
    if (el) el.textContent = value;
  }

  function safe(fn) {
    // Run a per-view render function; swallow errors from views not yet
    // injected into the DOM (lazy loading). Keeps render() from aborting.
    try { return fn(); } catch (e) { /* view not loaded yet */ }
  }

  function setKv(id, rows) {
    const el = document.getElementById(id);
    if (!el) return;
    el.innerHTML = rows.map(([label, value]) => (
      `<div><span>${label}</span><strong>${value}</strong></div>`
    )).join("");
  }

  return { fmt, canonicalFenceBounds, courtFrameModel, timeText, dateText, escapeHtml, setText, safe, setKv };
})();
