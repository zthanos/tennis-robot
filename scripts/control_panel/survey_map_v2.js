/* Survey Map v2 renderer — self-contained module for the Court Knowledge Model
 * (court_knowledge_model/v2). Draws the full court map, fence run-off distances,
 * obstacles and status directly from the v2 court_boundary.json schema. Wired in
 * app.js at the renderSurveyDiscovery call site when the v2 schema is present, so
 * it bypasses the legacy v1 renderer entirely.
 *
 * window.SurveyMapV2.render(diagnostics, survey)
 */
(function () {
  "use strict";
  const C = {
    pts: "rgba(87,166,255,0.55)", fence: "#a855f7", court: "#2fd08f",
    dist: "#ffbd5a", net: "#4aa3ff", robot: "#57a6ff", obst: "#ff7a45",
    text: "#eef4f8", muted: "rgba(238,244,248,0.6)", grid: "rgba(255,255,255,0.05)",
  };
  const f2 = (v, s = "") => (Number.isFinite(v) ? v.toFixed(2) + s : "—");
  const fs = (v) => (Number.isFinite(v) ? (v < 90 ? `${v.toFixed(1)}s` : `${Math.floor(v / 60)}m ${Math.floor(v % 60)}s`) : "—");

  // court-frame (x' length, y' width) -> map, via the net frame in the v2 model
  function makeToMap(net) {
    const c = net.center, al = net.axis_length, aw = net.axis_width;
    return (xp, yp) => ({
      x_m: c.x_m + xp * al.x_m + yp * aw.x_m,
      y_m: c.y_m + xp * al.y_m + yp * aw.y_m,
    });
  }

  function bbox(pts) {
    let minx = Infinity, maxx = -Infinity, miny = Infinity, maxy = -Infinity;
    for (const p of pts) {
      if (!Number.isFinite(p.x_m) || !Number.isFinite(p.y_m)) continue;
      if (p.x_m < minx) minx = p.x_m; if (p.x_m > maxx) maxx = p.x_m;
      if (p.y_m < miny) miny = p.y_m; if (p.y_m > maxy) maxy = p.y_m;
    }
    return { minx, maxx, miny, maxy };
  }

  function surveyLifecycle(cb, live) {
    const boundaryNewer = Number.isFinite(cb.surveyed_at) && Number.isFinite(live.updated_at)
      ? cb.surveyed_at >= live.updated_at
      : cb.completed === true && !live.updated_at;
    const running = !!live.running;
    const failed = live.result === "FAILED" || cb.status === "FAILED";
    const completed = live.result === "OK"
      || (cb.completed === true && (!running || boundaryNewer))
      || (cb.status === "OK" && cb.map_artifact && (!running || boundaryNewer));
    const cov = live.coverage || {};
    const n = Number.isFinite(cov.n) ? cov.n : 0;
    const i = Number.isFinite(cov.i) ? cov.i : 0;
    if (failed) {
      return {
        phase: "failed",
        failedStep: Math.min(3, Math.max(0, live.state === "saving_map" ? 2 : live.state === "coverage" ? 1 : 0)),
        progress: Math.max(8, n ? Math.min(95, (i / n) * 100) : 100),
      };
    }
    if (completed) return { phase: "completed", progress: 100 };
    if (running && live.state === "saving_map") return { phase: "saving", progress: 88 };
    if (running && live.state === "coverage") return { phase: "running", progress: n ? Math.max(28, Math.min(82, 28 + (i / n) * 54)) : 32 };
    if (running || live.state || live.survey_start_pose || live.net) return { phase: "started", progress: 14 };
    return { phase: "idle", progress: 0 };
  }

  function setLifecycle(cb, live) {
    const life = surveyLifecycle(cb || {}, live || {});
    const order = ["started", "running", "saving", "completed"];
    const phaseRank = { idle: -1, failed: -1, started: 0, running: 1, saving: 2, completed: 3 };
    const rank = phaseRank[life.phase] ?? -1;
    order.forEach((step, idx) => {
      const el = document.querySelector(`#surveyLifecycle .survey-step[data-step="${step}"]`);
      if (!el) return;
      el.classList.toggle("is-active", life.phase === step);
      el.classList.toggle("is-complete", life.phase !== "failed" && idx < rank);
      el.classList.toggle("is-failed", life.phase === "failed" && idx === life.failedStep);
    });
    const bar = document.querySelector("#surveyLifecycleProgress");
    const fill = bar ? bar.querySelector("span") : null;
    if (bar && fill) {
      bar.classList.toggle("is-complete", life.phase === "completed");
      bar.classList.toggle("is-failed", life.phase === "failed");
      fill.style.width = `${Math.max(0, Math.min(100, life.progress))}%`;
    }
    return life;
  }

  function render(diagnostics, survey) {
    const canvas = document.getElementById("surveyDiscoveryMap");
    const statusEl = document.getElementById("surveyDiscoveryStatus");
    const metaEl = document.getElementById("surveyDiscoveryMeta");
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const W = canvas.width, H = canvas.height;
    const cb = diagnostics.court_boundary || {};
    const live = diagnostics.court_survey_live || {};
    const setStatus = (t, col) => { if (statusEl) { statusEl.textContent = t; statusEl.style.color = col || "var(--muted)"; } };
    const lifecycle = setLifecycle(cb, live);

    // Fail-loud: a running survey that cannot build the map must be visible.
    if (live.running && live.error) {
      ctx.fillStyle = "#1a0d0d"; ctx.fillRect(0, 0, W, H);
      ctx.fillStyle = "#ff6b6b"; ctx.font = "bold 14px system-ui";
      ctx.fillText("LIVE MAP ERROR: " + live.error, 24, 40);
      ctx.fillStyle = C.muted; ctx.font = "12px system-ui";
      ctx.fillText("No fabricated court shown — fix the source above.", 24, 64);
      setStatus("live map error · " + live.error, "var(--warn)");
      if (metaEl) metaEl.innerHTML = "";
      return;
    }

    const points = Array.isArray(live.map_points) ? live.map_points : [];
    const robot = live.robot || {};
    const hasModel = cb.schema === "court_knowledge_model/v2" && cb.status === "OK" && cb.net && cb.fence;
    const running = !!live.running;

    // ---- world bbox + transform (prefer the measured fence, else live points) ----
    let bb;
    if (hasModel && Array.isArray(cb.fence.corners) && cb.fence.corners.length) {
      bb = bbox(cb.fence.corners.concat(points));
    } else if (points.length) {
      bb = bbox(points);
    } else {
      const failed = live.result === "FAILED" || cb.status === "FAILED";
      ctx.fillStyle = failed ? "#1a0d0d" : "#090d12"; ctx.fillRect(0, 0, W, H);
      ctx.fillStyle = failed ? "#ff6b6b" : C.muted; ctx.font = "13px system-ui";
      ctx.fillText(failed ? ("survey FAILED: " + (live.failure_reason || cb.failure_reason || "unknown"))
        : running ? "measuring… waiting for LiDAR points" : "no court map yet — run Map Court", 24, 36);
      setStatus(failed ? ("v2 - FAILED - " + (live.failure_reason || cb.failure_reason || "unknown"))
        : running ? "survey started - waiting for LiDAR points" : "idle - waiting for Map Court",
        failed ? "var(--warn)" : undefined);
      if (metaEl) metaEl.innerHTML = "";
      return;
    }
    const plotW = W - 270, pad = 44;
    const wW = Math.max(0.5, bb.maxx - bb.minx), wH = Math.max(0.5, bb.maxy - bb.miny);
    const scale = Math.min((plotW - 2 * pad) / wW, (H - 2 * pad) / wH);
    const cxw = (bb.minx + bb.maxx) / 2, cyw = (bb.miny + bb.maxy) / 2;
    const sx = (x) => (plotW / 2) + (x - cxw) * scale;
    const sy = (y) => (H / 2) - (y - cyw) * scale; // flip y (map up -> screen down)
    const SX = (p) => sx(p.x_m), SY = (p) => sy(p.y_m);

    ctx.fillStyle = "#090d12"; ctx.fillRect(0, 0, W, H);

    // ---- LiDAR point cloud ----
    ctx.fillStyle = C.pts;
    for (const p of points) {
      if (!Number.isFinite(p.x_m) || !Number.isFinite(p.y_m)) continue;
      ctx.fillRect(sx(p.x_m) - 0.7, sy(p.y_m) - 0.7, 1.4, 1.4);
    }

    // ---- live robot breadcrumb path (the survey trail) ----
    const trail = Array.isArray(diagnostics.robot_path) ? diagnostics.robot_path : [];
    if (trail.length > 1) {
      ctx.strokeStyle = "#ff8800"; ctx.lineWidth = 1.5;
      ctx.beginPath();
      let drawn = false;
      for (const p of trail) {
        if (!Number.isFinite(p.x_m) || !Number.isFinite(p.y_m)) continue;
        const X = sx(p.x_m), Y = sy(p.y_m);
        drawn ? ctx.lineTo(X, Y) : ctx.moveTo(X, Y); drawn = true;
      }
      ctx.stroke();
    }

    if (hasModel) {
      const net = cb.net, toMap = makeToMap(net);
      const L = (cb.court && cb.court.lines_court_frame) || {};
      const ext = cb.fence.extents_court_frame || {};
      const sl = L.sidelines_y || [-5.485, 5.485];
      const bl = L.baselines_x || [-11.885, 11.885];
      const drawCourtLine = (x1, y1, x2, y2) => {
        const a = toMap(x1, y1), b = toMap(x2, y2);
        ctx.beginPath(); ctx.moveTo(SX(a), SY(a)); ctx.lineTo(SX(b), SY(b)); ctx.stroke();
      };

      // ---- fence rectangle ----
      const cor = cb.fence.corners;
      ctx.strokeStyle = C.fence; ctx.lineWidth = 2; ctx.setLineDash([6, 4]);
      ctx.beginPath();
      cor.forEach((p, i) => { i ? ctx.lineTo(SX(p), SY(p)) : ctx.moveTo(SX(p), SY(p)); });
      ctx.closePath(); ctx.stroke(); ctx.setLineDash([]);

      // ---- court internal lines (green) ----
      ctx.strokeStyle = C.court; ctx.lineWidth = 1.5;
      bl.forEach((bx) => drawCourtLine(bx, sl[0], bx, sl[1]));        // baselines
      (L.service_x || [-6.4, 6.4]).forEach((sx2) => drawCourtLine(sx2, sl[0], sx2, sl[1])); // service lines
      sl.forEach((sy2) => drawCourtLine(bl[0], sy2, bl[1], sy2));     // singles sidelines
      drawCourtLine(-6.4, 0, 6.4, 0);                                 // centre service line

      // ---- run-off distance arrows (orange) ----
      const D = cb.distances_to_fence_m || {};
      ctx.font = "11px system-ui";
      const arrow = (cxp, cyp, fxp, fyp, label) => {
        const a = toMap(cxp, cyp), b = toMap(fxp, fyp);
        ctx.strokeStyle = C.dist; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(SX(a), SY(a)); ctx.lineTo(SX(b), SY(b)); ctx.stroke();
        ctx.fillStyle = C.dist;
        const mx = (SX(a) + SX(b)) / 2, my = (SY(a) + SY(b)) / 2;
        if (Number.isFinite(label)) { ctx.fillText(label.toFixed(2) + "m", mx + 4, my - 3); }
      };
      if (Number.isFinite(ext.x_near)) arrow(bl[0], 0, ext.x_near, 0, D.near_baseline);
      if (Number.isFinite(ext.x_far)) arrow(bl[1], 0, ext.x_far, 0, D.far_baseline);
      if (Number.isFinite(ext.y_left)) arrow(0, sl[0], 0, ext.y_left, D.left_sideline);
      if (Number.isFinite(ext.y_right)) arrow(0, sl[1], 0, ext.y_right, D.right_sideline);

      // ---- net line (blue, thick) + posts ----
      if (Array.isArray(net.posts) && net.posts.length === 2) {
        const pa = net.posts[0], pb = net.posts[1];
        ctx.strokeStyle = C.net; ctx.lineWidth = 3;
        ctx.beginPath(); ctx.moveTo(SX(pa), SY(pa)); ctx.lineTo(SX(pb), SY(pb)); ctx.stroke();
        ctx.fillStyle = C.net;
        [pa, pb].forEach((p) => { ctx.beginPath(); ctx.arc(SX(p), SY(p), 3, 0, 7); ctx.fill(); });
        ctx.fillText("net", SX(net.center) + 6, SY(net.center) - 6);
      }

      // ---- obstacles ----
      ctx.fillStyle = C.obst; ctx.strokeStyle = C.obst;
      (cb.obstacles || []).forEach((o) => {
        const c = o.center; if (!c || !Number.isFinite(c.x_m)) return;
        const r = Math.max(4, (o.size_m ? Math.max(o.size_m.w, o.size_m.h) : 0.3) * scale / 2);
        ctx.lineWidth = 1.5; ctx.beginPath(); ctx.arc(SX(c), SY(c), r, 0, 7); ctx.stroke();
        ctx.beginPath(); ctx.arc(SX(c), SY(c), 2, 0, 7); ctx.fill();
      });
    }

    // ---- robot pose ----
    if (Number.isFinite(robot.x_m) && Number.isFinite(robot.y_m)) {
      const rx = sx(robot.x_m), ry = sy(robot.y_m), yaw = robot.yaw_rad || 0;
      ctx.save(); ctx.translate(rx, ry); ctx.rotate(-yaw);
      ctx.fillStyle = C.robot; ctx.beginPath();
      ctx.moveTo(9, 0); ctx.lineTo(-6, 6); ctx.lineTo(-6, -6); ctx.closePath(); ctx.fill();
      ctx.restore();
    }

    // ---- right-side distances panel ----
    const px = plotW + 14;
    ctx.fillStyle = C.text; ctx.font = "bold 13px system-ui";
    ctx.fillText("Distances to Fences", px, 28);
    ctx.font = "12px system-ui";
    const D = (hasModel && cb.distances_to_fence_m) || {};
    const rows = [
      ["Near baseline", D.near_baseline], ["Far baseline", D.far_baseline],
      ["Left sideline", D.left_sideline], ["Right sideline", D.right_sideline],
    ];
    let yy = 54;
    rows.forEach(([k, v]) => {
      ctx.fillStyle = C.muted; ctx.fillText(k, px, yy);
      ctx.fillStyle = C.text; ctx.textAlign = "right"; ctx.fillText(f2(v, "m"), W - 14, yy);
      ctx.textAlign = "left"; yy += 22;
    });
    const vals = rows.map((r) => r[1]).filter(Number.isFinite);
    if (vals.length) {
      yy += 8;
      const stat = (k, v) => { ctx.fillStyle = C.muted; ctx.fillText(k, px, yy); ctx.fillStyle = C.text; ctx.textAlign = "right"; ctx.fillText(f2(v, "m"), W - 14, yy); ctx.textAlign = "left"; yy += 22; };
      stat("Min", Math.min(...vals)); stat("Max", Math.max(...vals));
      stat("Avg", vals.reduce((a, b) => a + b, 0) / vals.length);
    }

    // ---- status + meta ----
    const npts = Number.isFinite(live.map_point_count) ? live.map_point_count : points.length;
    const lifecycleText = lifecycle.phase === "completed" ? "completed"
      : lifecycle.phase === "saving" ? "saving map"
      : lifecycle.phase === "running" ? "running"
      : lifecycle.phase === "failed" ? "failed"
      : lifecycle.phase === "started" ? "started" : "idle";
    if (live.result === "FAILED" || cb.status === "FAILED") {
      setStatus("v2 - FAILED - " + (live.failure_reason || cb.failure_reason || "unknown"), "var(--warn)");
    } else if (lifecycle.phase === "completed" && hasModel) {
      setStatus(`v2 - completed - ${npts} pts - saved`, "var(--ok)");
    } else if (running) {
      const cov = live.coverage || {};
      const phase = live.state === "find_net" ? "finding net"
        : live.state === "saving_map" ? "saving map"
        : (cov.n ? `coverage ${Math.min(cov.i + 1, cov.n)}/${cov.n}` : "survey running");
      setStatus(`${phase} - ${npts} pts`, live.state === "saving_map" ? "var(--ok)" : "var(--accent-2)");
    } else if (hasModel) {
      setStatus(`v2 - completed - ${npts} pts - saved`, "var(--ok)");
    } else {
      setStatus("idle - waiting for Map Court", "var(--muted)");
    }

    if (metaEl) {
      const timing = live.timing || cb.timing || {};
      const phaseDurations = timing.phase_durations_s || {};
      const currentPhase = timing.current_phase || lifecycle.phase;
      const currentPhaseS = Number.isFinite(timing.current_phase_s)
        ? timing.current_phase_s
        : (Number.isFinite(phaseDurations[currentPhase]) ? phaseDurations[currentPhase] : null);
      if (hasModel) {
        const c = cb.court || {}, net = cb.net, D2 = cb.distances_to_fence_m || {};
        const obsRows = (cb.obstacles || []).map((o, i) =>
          `<tr><td>#${o.id != null ? o.id : i + 1}</td><td>${o.class || "obstacle"}</td>` +
          `<td>${o.size_m ? f2(o.size_m.w) + "×" + f2(o.size_m.h) + " m" : "—"}</td>` +
          `<td>${o.point_count != null ? o.point_count : "—"}</td></tr>`).join("");
        metaEl.innerHTML =
          `<div><span>Survey</span><strong>${lifecycleText}</strong></div>` +
          `<div><span>Elapsed</span><strong>${fs(timing.elapsed_s)}</strong></div>` +
          `<div><span>Phase</span><strong>${currentPhase} ${fs(currentPhaseS)}</strong></div>` +
          `<div><span>Court</span><strong>${f2(c.length_m)} × ${f2(c.width_m)} m · ${c.is_doubles ? "doubles" : "singles"}</strong></div>` +
          `<div><span>Net centre (map)</span><strong>${f2(net.center.x_m)}, ${f2(net.center.y_m)} · span ${f2(net.span_m, "m")}</strong></div>` +
          `<div><span>Run-off N/F</span><strong>${f2(D2.near_baseline, "m")} / ${f2(D2.far_baseline, "m")}</strong></div>` +
          `<div><span>Run-off L/R</span><strong>${f2(D2.left_sideline, "m")} / ${f2(D2.right_sideline, "m")}</strong></div>` +
          `<div><span>Occupancy</span><strong>${(cb.occupancy && cb.occupancy.point_count) || npts} pts</strong></div>` +
          `<div><span>Obstacles</span><strong>${(cb.obstacles || []).length}</strong></div>` +
          (obsRows ? `<table style="margin-top:8px;width:100%;border-collapse:collapse;font-size:12px;"><thead><tr style="color:var(--muted);text-align:left;"><th>ID</th><th>Class</th><th>Size</th><th>Pts</th></tr></thead><tbody>${obsRows}</tbody></table>` : "");
      } else {
        metaEl.innerHTML = `<div><span>Survey</span><strong>${lifecycleText}</strong></div>` +
          `<div><span>Elapsed</span><strong>${fs(timing.elapsed_s)}</strong></div>` +
          `<div><span>Phase</span><strong>${currentPhase} ${fs(currentPhaseS)}</strong></div>` +
          `<div><span>Points</span><strong>${npts}</strong></div>`;
      }
    }
  }

  window.SurveyMapV2 = { render };
})();
