    const titles = {
      dashboard: ["Dashboard", "Observe the robot mode, collector state, current target, and command stream while the simulation runs."],
      control: ["Command Center", "Send high-level commands and inspect the selected robot mode."],
      survey: ["Survey Workspace", "Run Map Court with the native live survey map, camera feed, survey metrics, and boundary status in one operational view."],
      collection: ["Collection Workspace", "Run collection with the court map and half-court mapping grid visible together."],
      sensors: ["Diagnostics", "Inspect raw sensor feeds and lower-level debug data without mixing them into the mission workspaces."],
      telemetry: ["Telemetry", "Inspect live robot pose, detection, command output, survey data, and raw status."],
      stats: ["Command Stats", "Review per-mode command counts and recent command usage."],
      history: ["History", "Audit the local command stream written by this console and controller startup."],
      webcam: ["Webcam", "Live webcam feed with HSV tennis ball detection and monocular distance estimation."],
      vendors: ["Vendors", "Manage vendors, venues and courts. Set the active session so survey results are tagged with the correct location."],
      surveys: ["Court Map History", "Saved Map Court runs from DuckDB, grouped by active vendor and court."]
    };
    let diagnostics = { command: {}, robot: {}, history: [], stats: {} };
    let sensors = {};
    let lastSurveyDiscovery = null;
    let robotPath = [];
    let discoveryCleared = false;
    let discoveryClearBaseline = 0;
    async function clearSurveyPath() {
      try {
        await fetch("/api/path/clear", { method: "POST" });
        robotPath = [];
        // Hide the estimate overlay (waypoints, estimated fence, net line,
        // distances) for a clean court + legend + robot + trail view. The
        // overlay returns automatically once a NEW survey adds discovery points.
        discoveryCleared = true;
        discoveryClearBaseline = ((diagnostics.court_survey_live || {}).navigation_points || []).length;
        lastSurveyDiscovery = null;
        refresh();
      } catch (e) { /* ignore */ }
    }

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
    function renderTelemetryEvents(events) {
      const target = document.getElementById("telemetryEvents");
      if (!target) return;
      if (!events || events.length === 0) {
        target.innerHTML = `<div style="color:var(--muted);font-size:13px;">No telemetry events yet.</div>`;
        return;
      }
      const hidden = new Set(["t_s", "wall_time_s", "type", "severity"]);
      target.innerHTML = events.slice(-24).reverse().map(event => {
        const details = Object.entries(event)
          .filter(([key, value]) => !hidden.has(key) && value !== null && value !== undefined)
          .map(([key, value]) => `${key}=${Array.isArray(value) || typeof value === "object" ? JSON.stringify(value) : value}`)
          .join(" · ");
        const severity = event.severity || "info";
        const color = severity === "error" ? "var(--danger)" : severity === "warning" ? "var(--warn)" : "var(--accent-2)";
        return `<div class="event">
          <span>${fmt(event.t_s, "s")}</span>
          <strong style="color:${color};">${escapeHtml(severity)}</strong>
          <div><strong>${escapeHtml(event.type || "event")}</strong><br><span>${escapeHtml(details || "no details")}</span></div>
        </div>`;
      }).join("");
    }
    // --- Lazy view loading ----------------------------------------------------
    // Each <section class="view" data-partial="X"> starts empty; its markup lives
    // in /static/views/X.html and is fetched the first time the view is opened.
    // VIEW_INIT[name] wires up that view's event listeners after injection.
    const VIEW_INIT = {};
    const _viewLoaded = new Set();
    const _viewLoading = {};
    function loadView(name) {
      if (_viewLoaded.has(name)) return Promise.resolve();
      if (_viewLoading[name]) return _viewLoading[name];
      const section = document.getElementById(name);
      if (!section || !section.dataset.partial) {
        _viewLoaded.add(name);
        return Promise.resolve();
      }
      _viewLoading[name] = fetch(`/static/views/${section.dataset.partial}.html`, { cache: "no-store" })
        .then(res => res.text())
        .then(html => {
          section.innerHTML = html;
          _viewLoaded.add(name);
          if (typeof VIEW_INIT[name] === "function") {
            try { VIEW_INIT[name](); } catch (e) { /* ignore init errors */ }
          }
        })
        .catch(() => { /* leave view empty on failure */ })
        .finally(() => { delete _viewLoading[name]; });
      return _viewLoading[name];
    }

    async function setView(name) {
      document.querySelectorAll("nav button").forEach(btn => btn.classList.toggle("active", btn.dataset.view === name));
      document.getElementById("viewTitle").textContent = titles[name][0];
      document.getElementById("viewHelp").textContent = titles[name][1];
      await loadView(name);
      document.querySelectorAll("section.view").forEach(view => view.classList.toggle("active", view.id === name));
      if (name === "webcam") startWebcam(); else stopWebcam();
      if (name === "vendors") window.ControlPanelVendors.load();
      if (name === "surveys") window.ControlPanelSurveyHistory.load();
    }
    document.querySelectorAll("nav button").forEach(btn => btn.addEventListener("click", () => setView(btn.dataset.view)));

    let _wcInterval = null;
    function startWebcam() {
      if (_wcInterval) return;
      refreshWebcam();
      _wcInterval = setInterval(refreshWebcam, 125);
    }
    function stopWebcam() {
      if (_wcInterval) { clearInterval(_wcInterval); _wcInterval = null; }
    }
    async function refreshWebcam() {
      try {
        const res = await fetch("/api/webcam/frame", { cache: "no-store" });
        renderWebcam(await res.json());
      } catch (_) {}
    }
    function renderWebcam(data) {
      const img = document.getElementById("wcFrame");
      const empty = document.getElementById("wcEmpty");
      const status = document.getElementById("wcStatus");
      if (!data.available) {
        img.style.display = "none";
        empty.style.display = "";
        empty.textContent = data.error || "webcam unavailable";
        status.textContent = "— unavailable";
        status.style.color = "var(--danger)";
        return;
      }
      img.src = data.data_url;
      img.style.display = "block";
      empty.style.display = "none";
      if (data.detected) {
        status.textContent = "— ball detected";
        status.style.color = "var(--accent)";
        document.getElementById("wcDistance").textContent = data.distance_m != null ? `${data.distance_m.toFixed(2)} m` : "—";
        const b = data.bearing_deg;
        document.getElementById("wcBearing").textContent = b != null ? `${b >= 0 ? "+" : ""}${b.toFixed(1)}°` : "—";
        document.getElementById("wcDiameter").textContent = data.diameter_px != null ? `${Math.round(data.diameter_px)} px` : "—";
      } else {
        status.textContent = "— no ball";
        status.style.color = "var(--muted)";
        ["wcDistance", "wcBearing", "wcDiameter"].forEach(id => { document.getElementById(id).textContent = "—"; });
      }
    }

    const DPAD_MODES = new Set(["move_forward", "move_backward", "move_left", "move_right",
      "move_forward_left", "move_forward_right", "move_backward_left", "move_backward_right"]);
    const AUTONOMOUS_MODES = new Set(["map_court", "map_left_side", "collect_pattern", "collect", "collect_one", "search", "scan_side"]);

    function updateCommandButtons() {
      const active = window.ControlPanelVendors?.getActive() || {};
      const hasSession = !!(active.vendor_id && active.court_id);
      const cb = diagnostics.court_boundary;
      const surveyReady = !!(cb && (cb.survey_complete || cb.completed || cb.status === "OK"));
      const surveyCourtMatches = !cb?.court_id || cb.court_id === active.court_id;
      const hasSurvey = hasSession && surveyReady && surveyCourtMatches;
      const actualMode = (diagnostics.robot || {}).actual_mode || "idle";
      const isAutonomous = AUTONOMOUS_MODES.has(actualMode);

      const btnMapCourt = document.querySelector('#commandForm [value="map_court"]');
      const btnCollect  = document.querySelector('#commandForm [value="collect"]');
      const hintEl      = document.getElementById("commandHint");

      if (btnMapCourt) btnMapCourt.disabled = !hasSession || isAutonomous;
      if (btnCollect)  btnCollect.disabled  = !hasSurvey || isAutonomous;

      const MANUAL_MODES = new Set([...DPAD_MODES, "turn_180"]);
      document.querySelectorAll("#commandForm .command").forEach(btn => {
        if (MANUAL_MODES.has(btn.value)) btn.disabled = isAutonomous;
      });

      if (hintEl) {
        if (isAutonomous)    hintEl.textContent = `Αυτόνομη λειτουργία ενεργή (${actualMode}) — πάτα Stop για έλεγχο`;
        else if (!hasSession) hintEl.textContent = "Επίλεξε vendor και γήπεδο πρώτα (Vendors →)";
        else if (!hasSurvey) hintEl.textContent = "Collect χρειάζεται Map Court για το ενεργό γήπεδο";
        else                 hintEl.textContent = "";
      }
    }

    let _dpadTimer = null;

    async function _sendRawCommand(mode) {
      await fetch("/api/command", {
        method: "POST",
        headers: { "Content-Type": "application/x-www-form-urlencoded" },
        body: new URLSearchParams({ mode })
      });
    }

    function routeForMode(mode) {
      if (mode === "map_court") return "survey";
      if (["collect", "collect_one", "collect_pattern", "search", "scan_side", "map_left_side"].includes(mode)) return "collection";
      return null;
    }

    function wireMissionCommands(rootId) {
      const root = document.getElementById(rootId);
      if (!root) return;
      root.querySelectorAll("[data-command-mode]").forEach(btn => {
        btn.addEventListener("click", async () => {
          const mode = btn.dataset.commandMode;
          if (!mode) return;
          await _sendRawCommand(mode);
          const nextView = routeForMode(mode);
          if (nextView) setView(nextView);
          await refresh();
        });
      });
    }

    async function _stopDpad() {
      if (_dpadTimer !== null) {
        clearInterval(_dpadTimer);
        _dpadTimer = null;
        await _sendRawCommand("idle");
        await refresh();
      }
    }

    // Wired after the Control view partial is injected (see loadView).
    VIEW_INIT.control = function () {
      document.querySelectorAll("#commandForm .command").forEach(btn => {
        if (!DPAD_MODES.has(btn.value)) return;
        btn.addEventListener("pointerdown", e => {
          e.preventDefault();
          if (_dpadTimer !== null) return;
          btn.setPointerCapture(e.pointerId);
          _sendRawCommand(btn.value);
          _dpadTimer = setInterval(() => _sendRawCommand(btn.value), 120);
        });
        btn.addEventListener("pointerup", () => _stopDpad());
        btn.addEventListener("pointercancel", () => _stopDpad());
      });

      const commandForm = document.getElementById("commandForm");
      if (commandForm) commandForm.addEventListener("submit", async event => {
        event.preventDefault();
        const mode = event.submitter?.value;
        if (!mode || DPAD_MODES.has(mode)) return;
        await _sendRawCommand(mode);
        const nextView = routeForMode(mode);
        if (nextView) setView(nextView);
        await refresh();
      });
      updateCommandButtons();
    };
    VIEW_INIT.survey = function () { wireMissionCommands("survey"); };
    VIEW_INIT.collection = function () { wireMissionCommands("collection"); };

    async function refresh() {
      const response = await fetch("/api/diagnostics", { cache: "no-store" });
      diagnostics = await response.json();
      const sensorResponse = await fetch("/api/sensors", { cache: "no-store" });
      sensors = await sensorResponse.json();
      render();
    }
    function render() {
      const command = diagnostics.command || {};
      const robot = diagnostics.robot || {};
      const obs = robot.observation || {};
      const out = robot.command || {};
      const _poseRaw = robot.robot || {};
      const pose = {
        x_m: Number.isFinite(_poseRaw.x_m) ? _poseRaw.x_m : robot.robot_x_m,
        y_m: Number.isFinite(_poseRaw.y_m) ? _poseRaw.y_m : robot.robot_y_m,
        yaw_rad: Number.isFinite(_poseRaw.yaw_rad) ? _poseRaw.yaw_rad : robot.robot_yaw_rad,
      };
      const survey = robot.survey || {};
      const search = robot.search || {};
      const mounts = robot.sensor_mounts || {};
      const oakDepth = robot.oak_depth || {};
      const scan = robot.scan || {};
      const collectOne = robot.collect_one || {};
      const collectPattern = robot.collect_pattern || {};
      const balls = robot.balls || {};
      const completion = robot.completion || {};
      const diag = robot.diagnostics || {};
      const connected = !!robot.connected;

      const liveDot = document.getElementById("liveDot");
      if (liveDot) liveDot.classList.toggle("live", connected);
      setText("connectionText", connected ? "Robot status live" : `Robot status stale (${fmt(robot.age_s, "s")})`);
      setText("lastRefresh", `Refreshed ${new Date().toLocaleTimeString()}`);
      setText("commandFile", `Sequence ${command.sequence ?? 0} from ${command.source ?? "default"}`);

      setText("kRequested", command.mode || "idle");
      setText("kSource", `source ${command.source || "default"}`);
      setText("kState", robot.collector_state || "idle");
      setText("kActual", `mode ${robot.actual_mode || "idle"}`);
      setText("kBalls", robot.balls_collected ?? 0);
      setText("kUptime", `remaining ${balls.same_side_remaining ?? "?"} same-side`);
      setText("kDetection", obs.visible ? "visible" : "hidden");
      setText("kDistance", `OAK-D Depth ${fmt(oakDepth.range_m ?? obs.distance_m, "m")} bearing ${fmt(obs.bearing_deg, "deg")}`);

      setKv("snapshot", [
        ["Robot position", `${fmt(pose.x_m, "m")}, ${fmt(pose.y_m, "m")}`],
        ["Robot yaw", fmt((pose.yaw_rad || 0) * 180 / Math.PI, "deg")],
        ["Collector state", robot.collector_state || "idle"],
        ["Search", `${search.search_state || "idle"} / Zone ${search.zone_id || "?"}`],
        ["Collect pattern", collectPattern.phase || "idle"],
        ["Coverage", fmt(search.coverage_pct, "%")],
        ["LiDAR height", fmt(mounts.front_lidar?.world_z_m, "m")],
        ["OAK-D height", fmt(mounts.front_camera?.world_z_m, "m")],
        ["OAK-D Depth", oakDepth.used_for_current_observation ? `${fmt(oakDepth.range_m, "m")} used` : (oakDepth.available ? "available" : "unavailable")],
        ["Collect one", collectOne.phase || "idle"],
        ["Side complete", completion.current_side_complete ? "yes" : "no"],
        ["Remaining balls", `${balls.same_side_remaining ?? "?"} same-side / ${balls.total_remaining ?? "?"} total`],
        ["Across net", balls.across_net_remaining ?? "none"],
        ["Intake", out.intake_enabled ? "enabled" : "disabled"],
        ["Base command", `${fmt(out.linear_speed_m_s, "m/s")} / ${fmt(out.angular_speed_rad_s, "rad/s")}`],
        ["Lift wheel", fmt(out.lift_wheel_speed)],
        ["Map court state", survey.state || "idle"],
        ["Map court event", survey.navigation?.last_event || "none"],
        ["Front range", fmt(survey.navigation?.front_lidar_range_m ?? survey.front_range_m, "m")]
      ]);
      setKv("selectedMode", [
        ["Requested mode", command.mode || "idle"],
        ["Actual mode", robot.actual_mode || "idle"],
        ["Sequence", command.sequence ?? 0],
        ["Updated", dateText(command.updated_at)],
        ["Source", command.source || "default"],
        ["Controller state", robot.collector_state || "idle"],
        ["Search state", search.search_state || "idle"],
        ["Search target", `${fmt(search.target_x_m, "m")}, ${fmt(search.target_y_m, "m")}`],
        ["Collect pattern", `${collectPattern.phase || "idle"} / failures ${collectPattern.failures ?? 0}`],
        ["Collect one phase", collectOne.phase || "idle"]
      ]);
      setKv("telemetryKv", [
        ["Mission health", diag.health || "unknown"],
        ["Health reasons", (diag.reasons || []).join(", ") || "none"],
        ["Last event", diag.last_event ? `${diag.last_event.type} @ ${fmt(diag.last_event.t_s, "s")}` : "none"],
        ["Telemetry enabled", robot.telemetry_enabled ? "yes" : "no"],
        ["Vision enabled", robot.vision_enabled ? "yes" : "no"],
        ["Route overlay", robot.route_visualization_enabled ? "yes" : "no"],
        ["Status age", fmt(robot.age_s, "s")],
        ["Visible candidates", balls.visible_candidates ?? 0],
        ["Nearest same-side", fmt(balls.nearest_same_side_distance_m, "m")],
        ["Loop count", robot.loop_count ?? 0],
        ["Ball visible", obs.visible ? "yes" : "no"],
        ["OAK-D Depth range", oakDepth.used_for_current_observation ? fmt(oakDepth.range_m, "m") : "not used"],
        ["OAK-D Depth limits", `${fmt(oakDepth.min_range_m, "m")} - ${fmt(oakDepth.max_range_m, "m")}`],
        ["Ball world", `${fmt(obs.world_x_m, "m")}, ${fmt(obs.world_y_m, "m")}`],
        ["Confidence", fmt(obs.confidence)],
        ["Animation", robot.collection_animation_active ? "active" : "idle"],
        ["Scan progress", `${fmt((scan.progress || 0) * 100, "%")} (${fmt(scan.elapsed_s, "s")}/${fmt(scan.full_turn_s, "s")})`],
        ["Scan best target", scan.best_visible ? `${fmt(scan.best_distance_m, "m")} @ ${fmt((scan.best_bearing_rad || 0) * 180 / Math.PI, "deg")}` : "none"],
        ["Search path", search.path_status || "idle"],
        ["Search resume", search.resume_marker || "none"],
        ["Map court state", survey.state || "idle"],
        ["Map court distance", fmt(survey.navigation?.distance_traveled_m, "m")]
      ]);
      safe(() => renderTelemetryEvents(robot.timeline_events || []));
      setText("rawStatus", JSON.stringify(robot, null, 2));

      // Each render*() targets a view that may not be loaded yet; safe() swallows
      // the resulting missing-element errors so the rest of render() still runs.
      safe(renderHistory);
      safe(renderStats);
      safe(renderCourtMap);
      safe(renderSensors);
      safe(() => renderSurveyBoundary(robot.survey || {}));
      // Persisted breadcrumb trail served by the backend (survives reloads/restarts).
      if (Array.isArray(diagnostics.robot_path)) robotPath = diagnostics.robot_path;
      safe(() => {
        const _cb = diagnostics.court_boundary;
        if (window.SurveyMapV2 && _cb && _cb.schema === "court_knowledge_model/v2") {
          window.SurveyMapV2.render(diagnostics, robot.survey || {});
        } else {
          renderSurveyDiscovery(robot.survey || {});
        }
      });
      safe(() => renderObstacleSurveyDebug((robot.survey || {}).navigation || {}));
      safe(() => renderObstacleRunsHistory(diagnostics.obstacle_runs || []));
      const collectionScan = robot.collection_scan || {};
      const mapMissionForGrid = (
        collectionScan.active || collectionScan.complete
          ? { ...collectionScan, source_label: "Collection scan" }
          : { ...(robot.map_mission || {}), source_label: "Mapping mission" }
      );
      safe(() => renderMapMission(mapMissionForGrid));
      safe(updateCommandButtons);

      // Auto-navigate to the mission workspace while survey/collection is active.
      const mapMission = robot.map_mission || {};
      const liveSurvey = diagnostics.court_survey_live || {};
      const surveyActive = liveSurvey.running || (survey.state && !["idle", "done", "complete", "completed"].includes(String(survey.state)));
      const collectionActive = (mapMission.active && !mapMission.complete) || (collectionScan.active && !collectionScan.complete);
      if (collectionActive || surveyActive) {
        const activeView = document.querySelector("section.view.active");
        const targetView = collectionActive ? "collection" : "survey";
        const autoRouteFrom = new Set(["dashboard", "control"]);
        if (activeView && activeView.id !== targetView && autoRouteFrom.has(activeView.id)) setView(targetView);
      }
    }
    function renderSurveyBoundary(survey) {
      const statusEl = document.getElementById("surveyBoundaryStatus");
      const kvEl = document.getElementById("surveyBoundaryKv");
      if (!statusEl || !kvEl) return;
      const bounds = survey.bounds;
      const inProgress = survey.state && survey.state !== "done";
      const isComplete = bounds && (bounds.status === "SUCCESS" || bounds.survey_complete);
      if (isComplete) {
        const cg = bounds.court_geometry || {};
        const fg = canonicalFenceBounds(bounds);
        const w  = fg.west_x  != null ? fg.west_x.toFixed(2)  : "—";
        const e  = fg.east_x  != null ? fg.east_x.toFixed(2)  : "—";
        const s  = fg.south_y != null ? fg.south_y.toFixed(2) : "—";
        const n  = fg.north_y != null ? fg.north_y.toFixed(2) : "—";
        const len = cg.length_m != null ? cg.length_m.toFixed(2) : "—";
        const wid = cg.width_m  != null ? cg.width_m.toFixed(2)  : "—";
        statusEl.textContent = `— SUCCESS · ${bounds.sample_count ?? "?"} samples`;
        statusEl.style.color = "var(--accent)";
        setKv("surveyBoundaryKv", [
          ["Court length (E-W)", `${len} m`],
          ["Court width (N-S)", `${wid} m`],
          ["West fence x",  `${w} m`],
          ["East fence x",  `${e} m`],
          ["South fence y", `${s} m`],
          ["North fence y", `${n} m`],
        ]);
      } else if (inProgress) {
        statusEl.textContent = `— mapping · ${survey.sample_count ?? 0} samples`;
        statusEl.style.color = "var(--accent-2)";
        setKv("surveyBoundaryKv", [
          ["State", survey.state || "—"],
          ["Event", (survey.navigation || {}).last_event || "none"],
          ["Distance", fmt((survey.navigation || {}).distance_traveled_m, "m")],
          ["Samples collected", survey.sample_count ?? 0],
        ]);
      } else {
        statusEl.textContent = "no court map data";
        statusEl.style.color = "var(--muted)";
        kvEl.innerHTML = "";
      }
    }
    function renderSurveyDiscovery(survey) {
      const canvas = document.getElementById("surveyDiscoveryMap");
      const statusEl = document.getElementById("surveyDiscoveryStatus");
      if (!canvas) return;
      const ctx = canvas.getContext("2d");
      const nav = survey.navigation || {};
      const points = Array.isArray(nav.map_points) ? nav.map_points : [];
      const bounds = survey.bounds || diagnostics.court_boundary || {};
      const net = nav.net_boundary || (bounds.boundaries || {}).net || (bounds.court_features || {}).net_boundary || bounds.net;
      const width = canvas.width;
      const height = canvas.height;
      const pad = 36;

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#090d12";
      ctx.fillRect(0, 0, width, height);

      const hasPoints = points.some(p => Number.isFinite(p.x_m) && Number.isFinite(p.y_m));
      const hasNet = !!(net && ((net.post_a && net.post_b) || net.center || net.midpoint));
      if (!hasPoints && !hasNet) {
        ctx.fillStyle = "rgba(145,162,178,0.7)";
        ctx.font = "14px system-ui";
        ctx.fillText("waiting for Map Court LiDAR points", pad, pad + 10);
        if (statusEl) statusEl.textContent = "waiting for points";
        setKv("surveyDiscoveryMeta", [
          ["State", survey.state || "idle"],
          ["Points", nav.map_point_count ?? 0],
          ["Net boundary", net ? "detected" : "none"],
        ]);
        return;
      }

      const fg = canonicalFenceBounds(bounds);
      const worldExtents = nav.scan_coverage?.world_extents || bounds.lidar_boundary_estimate?.combined || {};
      const hasFence = Number.isFinite(fg.west_x) && Number.isFinite(fg.east_x) && Number.isFinite(fg.south_y) && Number.isFinite(fg.north_y);
      const hasExtents = Number.isFinite(worldExtents.min_x_m) && Number.isFinite(worldExtents.max_x_m) && Number.isFinite(worldExtents.min_y_m) && Number.isFinite(worldExtents.max_y_m);
      let minX = hasFence ? fg.west_x - 0.5 : (hasExtents ? worldExtents.min_x_m - 1.0 : -14.0);
      let maxX = hasFence ? fg.east_x + 0.5 : (hasExtents ? worldExtents.max_x_m + 1.0 : 14.0);
      let minY = hasFence ? fg.south_y - 0.5 : (hasExtents ? worldExtents.min_y_m - 1.0 : -7.5);
      let maxY = hasFence ? fg.north_y + 0.5 : (hasExtents ? worldExtents.max_y_m + 1.0 : 7.5);
      const netCenter = net?.center || net?.midpoint || (net?.post_a && net?.post_b ? {
        x_m: (net.post_a.x_m + net.post_b.x_m) / 2,
        y_m: (net.post_a.y_m + net.post_b.y_m) / 2,
      } : null);
      const courtCorners = (() => {
        if (!net?.post_a || !net?.post_b || !netCenter) return [];
        const ax = net.post_a.x_m, ay = net.post_a.y_m;
        const bx = net.post_b.x_m, by = net.post_b.y_m;
        const dx = bx - ax, dy = by - ay;
        const postSpan = Math.hypot(dx, dy);
        if (!Number.isFinite(postSpan) || postSpan < 0.5) return [];
        const tx = dx / postSpan, ty = dy / postSpan;
        const nx = -ty, ny = tx;
        const courtWidth = Number.isFinite(net.length_m) && net.length_m > 6 ? net.length_m : 10.97;
        const halfW = courtWidth / 2;
        const halfL = 23.77 / 2;
        return [
          { x_m: netCenter.x_m + nx * halfL + tx * halfW, y_m: netCenter.y_m + ny * halfL + ty * halfW },
          { x_m: netCenter.x_m + nx * halfL - tx * halfW, y_m: netCenter.y_m + ny * halfL - ty * halfW },
          { x_m: netCenter.x_m - nx * halfL - tx * halfW, y_m: netCenter.y_m - ny * halfL - ty * halfW },
          { x_m: netCenter.x_m - nx * halfL + tx * halfW, y_m: netCenter.y_m - ny * halfL + ty * halfW },
        ];
      })();
      [...points, net?.post_a, net?.post_b, netCenter, ...courtCorners, (diagnostics.robot || {}).robot]
        .filter(Boolean)
        .forEach(p => {
          if (!Number.isFinite(p.x_m) || !Number.isFinite(p.y_m)) return;
          minX = Math.min(minX, p.x_m - 0.5);
          maxX = Math.max(maxX, p.x_m + 0.5);
          minY = Math.min(minY, p.y_m - 0.5);
          maxY = Math.max(maxY, p.y_m + 0.5);
        });
      const spanX = Math.max(1, maxX - minX);
      const spanY = Math.max(1, maxY - minY);
      const scale = Math.min((width - pad * 2) / spanX, (height - pad * 2) / spanY);
      const ox = (width - spanX * scale) / 2;
      const oy = (height - spanY * scale) / 2;
      const sx = x => ox + (x - minX) * scale;
      const sy = y => height - (oy + (y - minY) * scale);

      ctx.strokeStyle = "rgba(255,255,255,0.07)";
      ctx.lineWidth = 1;
      for (let gx = Math.ceil(minX); gx <= Math.floor(maxX); gx += 1) {
        ctx.beginPath(); ctx.moveTo(sx(gx), sy(minY)); ctx.lineTo(sx(gx), sy(maxY)); ctx.stroke();
      }
      for (let gy = Math.ceil(minY); gy <= Math.floor(maxY); gy += 1) {
        ctx.beginPath(); ctx.moveTo(sx(minX), sy(gy)); ctx.lineTo(sx(maxX), sy(gy)); ctx.stroke();
      }

      if (hasFence || hasExtents) {
        const bx0 = hasFence ? fg.west_x : worldExtents.min_x_m;
        const bx1 = hasFence ? fg.east_x : worldExtents.max_x_m;
        const by0 = hasFence ? fg.south_y : worldExtents.min_y_m;
        const by1 = hasFence ? fg.north_y : worldExtents.max_y_m;
        ctx.strokeStyle = hasFence ? "rgba(255,255,255,0.38)" : "rgba(47,208,143,0.42)";
        ctx.lineWidth = 2;
        ctx.setLineDash(hasFence ? [] : [8, 6]);
        ctx.strokeRect(sx(bx0), sy(by1), sx(bx1) - sx(bx0), sy(by0) - sy(by1));
        ctx.setLineDash([]);
      }
      if (courtCorners.length === 4) {
        ctx.strokeStyle = "rgba(255,255,255,0.22)";
        ctx.lineWidth = 2;
        ctx.beginPath();
        courtCorners.forEach((p, i) => {
          if (i === 0) ctx.moveTo(sx(p.x_m), sy(p.y_m));
          else ctx.lineTo(sx(p.x_m), sy(p.y_m));
        });
        ctx.closePath();
        ctx.stroke();
      }
      ctx.fillStyle = "rgba(145,162,178,0.8)";
      ctx.font = "12px system-ui";
      ctx.fillText("world boundary map", pad, 20);

      ctx.fillStyle = "rgba(47,208,143,0.55)";
      points.forEach(p => {
        if (!Number.isFinite(p.x_m) || !Number.isFinite(p.y_m)) return;
        ctx.fillRect(sx(p.x_m) - 1.2, sy(p.y_m) - 1.2, 2.4, 2.4);
      });

      if (net && net.post_a && net.post_b) {
        const a = net.post_a;
        const b = net.post_b;
        ctx.strokeStyle = "rgba(80,220,255,0.95)";
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.moveTo(sx(a.x_m), sy(a.y_m));
        ctx.lineTo(sx(b.x_m), sy(b.y_m));
        ctx.stroke();
        ctx.fillStyle = "#50dcff";
        [[a, "post A"], [b, "post B"]].forEach(([p, label]) => {
          ctx.beginPath();
          ctx.arc(sx(p.x_m), sy(p.y_m), 5, 0, Math.PI * 2);
          ctx.fill();
          ctx.fillText(label, sx(p.x_m) + 7, sy(p.y_m) - 7);
        });
        ctx.font = "bold 13px system-ui";
        const c = netCenter || { x_m: (a.x_m + b.x_m) / 2, y_m: (a.y_m + b.y_m) / 2 };
        const clearance = net.front_clearance_m ?? net.distance_m;
        ctx.fillText(`NET ${fmt(net.length_m, "m")} · clearance ${fmt(clearance, "m")}`, sx(c.x_m) + 8, sy(c.y_m) - 8);
      } else if (netCenter) {
        const c = netCenter;
        ctx.fillStyle = "#50dcff";
        ctx.beginPath();
        ctx.arc(sx(c.x_m), sy(c.y_m), 6, 0, Math.PI * 2);
        ctx.fill();
        ctx.font = "bold 13px system-ui";
        ctx.fillText(`NET ${fmt(net.distance_m, "m")}`, sx(c.x_m) + 8, sy(c.y_m) - 8);
      }

      const _rs = diagnostics.robot || {};
      const _rp = _rs.robot || {};
      const robot = {
        x_m: Number.isFinite(_rp.x_m) ? _rp.x_m : _rs.robot_x_m,
        y_m: Number.isFinite(_rp.y_m) ? _rp.y_m : _rs.robot_y_m,
        yaw_rad: Number.isFinite(_rp.yaw_rad) ? _rp.yaw_rad : _rs.robot_yaw_rad,
      };
      if (Number.isFinite(robot.x_m) && Number.isFinite(robot.y_m)) {
        ctx.fillStyle = "#ffbd5a";
        ctx.beginPath();
        ctx.arc(sx(robot.x_m), sy(robot.y_m), 5, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillText("robot", sx(robot.x_m) + 8, sy(robot.y_m) + 4);
      }

      if (statusEl) {
        const pointCount = nav.map_point_count ?? points.length;
        statusEl.textContent = `${pointCount} world points · net ${net ? "localized" : "not fitted"}`;
        statusEl.style.color = net ? "var(--accent)" : "var(--warn)";
      }
      setKv("surveyDiscoveryMeta", [
        ["State", survey.state || "idle"],
        ["Event", nav.last_event || "none"],
        ["Points", nav.map_point_count ?? points.length],
        ["Shown sample", points.length],
        ["Frame", "world/court coordinates"],
        ["Sensor frame", nav.sensor_frame || "none"],
        ["LiDAR coverage", `F ${fmt(nav.scan_coverage?.front_m, "m")} · R ${fmt(nav.scan_coverage?.rear_m, "m")} · L ${fmt(nav.scan_coverage?.left_m, "m")} · Rt ${fmt(nav.scan_coverage?.right_m, "m")}`],
        ["World extents", nav.scan_coverage?.world_extents ? `x ${fmt(nav.scan_coverage.world_extents.min_x_m, "m")}..${fmt(nav.scan_coverage.world_extents.max_x_m, "m")} · y ${fmt(nav.scan_coverage.world_extents.min_y_m, "m")}..${fmt(nav.scan_coverage.world_extents.max_y_m, "m")}` : "none"],
        ["Net boundary", net ? `${fmt(net.length_m, "m")} @ front ${fmt(net.front_clearance_m ?? net.distance_m, "m")}` : "not fitted"],
        ["Net source", net?.source || nav.net_boundary_source || "none"],
      ]);
    }
    function renderSurveyDiscovery(survey) {
      const canvas = document.getElementById("surveyDiscoveryMap");
      const statusEl = document.getElementById("surveyDiscoveryStatus");
      const metaEl = document.getElementById("surveyDiscoveryMeta");
      if (!canvas) return;
      const ctx = canvas.getContext("2d");
      const liveNav = survey.navigation || {};
      const liveBounds = survey.bounds || {};
      const persistedBounds = diagnostics.court_boundary || {};
      const liveDiscovery = diagnostics.court_survey_live || {};
      // Fail-loud: a RUNNING survey that cannot build the live map (e.g. missing
      // TF) must be visible, never masked by a fabricated/default court.
      if (liveDiscovery.running && liveDiscovery.error) {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = "#1a0d0d"; ctx.fillRect(0, 0, canvas.width, canvas.height);
        ctx.fillStyle = "#ff6b6b"; ctx.font = "bold 14px system-ui";
        ctx.fillText("LIVE MAP ERROR: " + liveDiscovery.error, 24, 40);
        ctx.fillStyle = "rgba(238,244,248,0.7)"; ctx.font = "12px system-ui";
        ctx.fillText("No fabricated court shown — fix the source above.", 24, 64);
        if (statusEl) { statusEl.textContent = "live map error · " + liveDiscovery.error; statusEl.style.color = "var(--warn)"; }
        return;
      }
      const livePoints = Array.isArray(liveDiscovery.map_points) ? liveDiscovery.map_points
        : (Array.isArray(liveNav.map_points) ? liveNav.map_points : []);
      const persistedPoints = Array.isArray(persistedBounds.point_cloud_sample) ? persistedBounds.point_cloud_sample : [];
      const liveSurveyPoints = Array.isArray(liveDiscovery.navigation_points) ? liveDiscovery.navigation_points : [];
      const persistedSurveyPoints = Array.isArray(persistedBounds.navigation_points) ? persistedBounds.navigation_points : [];
      const liveNet = liveNav.net_boundary || liveBounds.net || (liveBounds.boundaries || {}).net || (liveBounds.court_features || {}).net_boundary;
      const persistedNet = persistedBounds.net || (persistedBounds.boundaries || {}).net || (persistedBounds.court_features || {}).net_boundary;
      const liveWaypointNet = liveDiscovery.locked_net || liveDiscovery.net;
      const hasLiveDiscovery = livePoints.length > 0 || liveSurveyPoints.length > 0 || !!liveNet || !!liveWaypointNet;
      const hasPersistedDiscovery = persistedBounds.survey_complete && (persistedPoints.length > 0 || persistedSurveyPoints.length > 0 || !!persistedNet);
      const source = hasLiveDiscovery ? {
        nav: liveNav,
        bounds: liveBounds,
        points: livePoints,
        surveyPoints: liveSurveyPoints,
        net: liveNet || liveWaypointNet,
        discovery: liveDiscovery,
        sourceLabel: "live"
      } : (lastSurveyDiscovery || (hasPersistedDiscovery ? {
        nav: {},
        bounds: persistedBounds,
        points: persistedPoints,
        surveyPoints: persistedSurveyPoints,
        net: persistedNet || persistedBounds.locked_net,
        discovery: persistedBounds,
        sourceLabel: "saved"
      } : {
        nav: liveNav,
        bounds: liveBounds.survey_complete ? liveBounds : {},
        points: [],
        surveyPoints: [],
        net: null,
        discovery: {},
        sourceLabel: "expected"
      }));
      if (hasLiveDiscovery) {
        lastSurveyDiscovery = source;
      }
      const nav = source.nav || {};
      const bounds = source.bounds || {};
      const net = source.net;
      const points = source.points || [];
      const surveyPoints = source.surveyPoints || [];
      const discovery = source.discovery || {};
      const surveyByLabel = {};
      surveyPoints.forEach(p => { if (p && p.label) surveyByLabel[p.label] = p; });
      const width = canvas.width;
      const height = canvas.height;
      const plotW = width - 270;
      const pad = 52;
      const _rs = diagnostics.robot || {};
      const _rp = _rs.robot || {};
      const _liveRobot = liveDiscovery.robot || {};
      const robot = {
        x_m: Number.isFinite(_liveRobot.x_m) ? _liveRobot.x_m : (Number.isFinite(_rp.x_m) ? _rp.x_m : _rs.robot_x_m),
        y_m: Number.isFinite(_liveRobot.y_m) ? _liveRobot.y_m : (Number.isFinite(_rp.y_m) ? _rp.y_m : _rs.robot_y_m),
        yaw_rad: Number.isFinite(_liveRobot.yaw_rad) ? _liveRobot.yaw_rad : (Number.isFinite(_rp.yaw_rad) ? _rp.yaw_rad : _rs.robot_yaw_rad),
      };
      // Camera-refined net anchor (fallback when LiDAR/waypoint net is absent).
      // Only when the OAK-D confidently sees the net ahead AND within depth range
      // (~<=9.5 m); beyond that the net is invisible to the camera so we leave the
      // LiDAR anchor untouched. Court is oriented to robot heading so the robot
      // lands at the camera-measured distance from the net.
      const vision = survey.vision || {};
      const cameraNetFrame = (() => {
        if (![robot.x_m, robot.y_m, robot.yaw_rad].every(Number.isFinite)) return null;
        const hx = Math.cos(robot.yaw_rad), hy = Math.sin(robot.yaw_rad); // toward net
        const lx = -hy, ly = hx;                                          // robot-left
        const off = Number.isFinite(vision.line_offset_m) ? vision.line_offset_m : 0;
        const mk = (dist, label) => ({
          center: { x_m: robot.x_m + hx * dist + lx * off, y_m: robot.y_m + hy * dist + ly * off },
          normal: { x_m: hx, y_m: hy }, tangent: { x_m: lx, y_m: ly }, _camera: true, _label: label, _dist: dist,
        });
        // (a) Direct net sighting within OAK-D depth range (net classified ahead).
        const d = vision.center_m;
        const conf = Number.isFinite(vision.line_confidence) ? vision.line_confidence : 0;
        const cls = vision.obstacle_class == null ? "" : String(vision.obstacle_class);
        if (Number.isFinite(d) && d > 0.3 && d <= 9.5 && /net/i.test(cls) && conf >= 0.4) {
          return mk(d, "camera-net");
        }
        // (b) Classified court-line junction — works when the net is out of range.
        //     T = service line (6.40 m from net); L = baseline corner (11.885 m).
        //     Robot is on the baseline side facing the net, so it sits at
        //     (lineFromNet + depth-to-junction) from the net.
        const jt = vision.junction_type;
        const jd = vision.junction_distance_m;
        const jc = Number.isFinite(vision.junction_confidence) ? vision.junction_confidence : 0;
        // Inverted-T (and a strong T mis-read as +) = SERVICE line; L = BASELINE.
        const isService = jt === "T" || jt === "+";
        const isBaseline = jt === "L";
        if ((isService || isBaseline) && Number.isFinite(jd) && jd > 0.1 && jd <= 9.5 && jc >= 0.4) {
          const lineFromNet = isService ? 6.40 : 11.885;
          return mk(lineFromNet + jd, isService ? "camera-service" : "camera-baseline");
        }
        return null;
      })();
      const cameraNet = cameraNetFrame ? cameraNetFrame.center : null;
      const fg = canonicalFenceBounds(bounds);
      const worldExtents = nav.scan_coverage?.world_extents || bounds.lidar_boundary_estimate?.combined || {};
      const hasFence = Number.isFinite(fg.west_x) && Number.isFinite(fg.east_x) && Number.isFinite(fg.south_y) && Number.isFinite(fg.north_y);
      const hasExtents = Number.isFinite(worldExtents.min_x_m) && Number.isFinite(worldExtents.max_x_m) && Number.isFinite(worldExtents.min_y_m) && Number.isFinite(worldExtents.max_y_m);
      // Real LiDAR-measured net position (locked_net carries the map-frame net
      // centre, confidence 1.0). Anchor the court HERE — not on the standoff
      // turn waypoint (first_net_left_turn_reference), which sits ~net_standoff
      // metres in front of the net and shifts the whole court.
      const lockedNetCenter = (Number.isFinite(net?.map_x_m) && Number.isFinite(net?.map_y_m))
        ? { x_m: net.map_x_m, y_m: net.map_y_m } : null;
      const netCenter = lockedNetCenter || net?.center || net?.midpoint || (net?.post_a && net?.post_b ? {
        x_m: (net.post_a.x_m + net.post_b.x_m) / 2,
        y_m: (net.post_a.y_m + net.post_b.y_m) / 2,
      } : (surveyByLabel.first_net_left_turn_reference || surveyByLabel.net_detected_from_far_side || cameraNet || null));
      // Court length axis from the net detection geometry (robot -> net).
      const lockedNetFrame = (() => {
        if (!lockedNetCenter || !Number.isFinite(net?.robot_x_m) || !Number.isFinite(net?.robot_y_m)) return null;
        const dx = lockedNetCenter.x_m - net.robot_x_m, dy = lockedNetCenter.y_m - net.robot_y_m;
        const mag = Math.hypot(dx, dy);
        if (!Number.isFinite(mag) || mag < 0.5) return null;
        const nx = dx / mag, ny = dy / mag;
        return { center: lockedNetCenter, normal: { x_m: nx, y_m: ny }, tangent: { x_m: -ny, y_m: nx }, _locked: true };
      })();
      const liveNetFrame = (() => {
        const seen = surveyByLabel.first_obstacle_net_detected;
        const ref = surveyByLabel.first_net_left_turn_reference || netCenter;
        if (!seen || !ref || !Number.isFinite(seen.x_m) || !Number.isFinite(seen.y_m) || !Number.isFinite(ref.x_m) || !Number.isFinite(ref.y_m)) return null;
        const dx = ref.x_m - seen.x_m;
        const dy = ref.y_m - seen.y_m;
        const mag = Math.hypot(dx, dy);
        if (!Number.isFinite(mag) || mag < 0.5) return null;
        const nx = dx / mag;
        const ny = dy / mag;
        return {
          center: ref,
          normal: { x_m: nx, y_m: ny },
          tangent: { x_m: -ny, y_m: nx },
        };
      })();
      const netFrame = lockedNetFrame || liveNetFrame || cameraNetFrame;
      const netSource = (lockedNetCenter || net?.center || net?.midpoint || (net?.post_a && net?.post_b)) ? "lidar"
        : (surveyByLabel.first_net_left_turn_reference || surveyByLabel.net_detected_from_far_side) ? "waypoint"
        : cameraNet ? (cameraNetFrame._label || "camera") : "none";
      // Camera-refined robot position: correct forward pose-drift along the court.
      // When the camera confidently measures the robot's distance to the net/line,
      // trust that distance over the drifting SLAM pose for the ALONG-court axis;
      // keep the lateral component from the pose. Only the forward error is fixed.
      const robotCourtPos = (() => {
        if (!netCenter || !Number.isFinite(robot.x_m) || !Number.isFinite(robot.y_m)) return null;
        if (!cameraNetFrame || !Number.isFinite(cameraNetFrame._dist)) return null;
        let nx, ny;
        if (netFrame) { nx = netFrame.normal.x_m; ny = netFrame.normal.y_m; }
        else if (net?.post_a && net?.post_b) {
          const dx = net.post_b.x_m - net.post_a.x_m, dy = net.post_b.y_m - net.post_a.y_m;
          const m = Math.hypot(dx, dy) || 1; nx = -dy / m; ny = dx / m; // perpendicular to net
        } else if (Number.isFinite(robot.yaw_rad)) { nx = Math.cos(robot.yaw_rad); ny = Math.sin(robot.yaw_rad); }
        else return null;
        const tx = -ny, ty = nx;
        const rx = robot.x_m - netCenter.x_m, ry = robot.y_m - netCenter.y_m;
        const along = rx * nx + ry * ny;
        const lateral = rx * tx + ry * ty;
        const alongFixed = (along >= 0 ? 1 : -1) * cameraNetFrame._dist;
        return { x_m: netCenter.x_m + nx * alongFixed + tx * lateral, y_m: netCenter.y_m + ny * alongFixed + ty * lateral, _corrected: true };
      })();
      const courtGeometry = bounds.court_geometry || {};
      const courtLength = Number.isFinite(courtGeometry.length_m) ? courtGeometry.length_m : 23.77;
      const courtWidth = Number.isFinite(courtGeometry.width_m) ? courtGeometry.width_m : (
        Number.isFinite(net?.length_m) && net.length_m > 6 ? net.length_m : 10.97
      );
      const extSpanX = hasExtents ? worldExtents.max_x_m - worldExtents.min_x_m : 0;
      const extSpanY = hasExtents ? worldExtents.max_y_m - worldExtents.min_y_m : 0;
      const extentsCanContainCourt = (
        (extSpanX >= courtLength && extSpanY >= courtWidth)
        || (extSpanY >= courtLength && extSpanX >= courtWidth)
      );
      let fenceRect = hasFence ? {
        minX: fg.west_x, maxX: fg.east_x, minY: fg.south_y, maxY: fg.north_y, source: "verified"
      } : (hasExtents && extentsCanContainCourt ? {
        minX: worldExtents.min_x_m, maxX: worldExtents.max_x_m, minY: worldExtents.min_y_m, maxY: worldExtents.max_y_m, source: "lidar boundary estimate"
      } : null);  // no synthetic Gazebo box — boundary only from real detections

      // Override fenceRect with measured two-point survey geometry.
      const twoPointGeo = persistedBounds.geometry;
      if (persistedBounds.survey_type === "two_point_net_baseline" && twoPointGeo) {
        const netP   = twoPointGeo.net_world_pos;
        const fenceP = twoPointGeo.fence_world_pos;
        const btf    = twoPointGeo.baseline_to_fence_m;
        if (netP && fenceP && Number.isFinite(btf)) {
          const dx = fenceP.x_m - netP.x_m;
          const dy = fenceP.y_m - netP.y_m;
          const surveyDist = Math.hypot(dx, dy);
          const ux = dx / surveyDist, uy = dy / surveyDist; // unit vector net→fence
          const px = -uy, py = ux;                          // perpendicular
          const halfLen = courtLength / 2;
          const halfWid = courtWidth / 2;
          // Unknown side margin: use current fenceRect or Gazebo default.
          const knownSideMargin = !fenceRect ? NaN : (Math.abs(px) > 0.5
            ? (fenceRect.maxX - fenceRect.minX) / 2 - halfLen
            : (fenceRect.maxY - fenceRect.minY) / 2 - halfWid);
          const sideMargin = Number.isFinite(knownSideMargin) && knownSideMargin > 0
            ? knownSideMargin : 3.01;
          // Build 4 corner points of the outer fence rectangle.
          const corners = [
            [netP.x_m + ux*(halfLen+btf) + px*(halfWid+sideMargin), netP.y_m + uy*(halfLen+btf) + py*(halfWid+sideMargin)],
            [netP.x_m + ux*(halfLen+btf) - px*(halfWid+sideMargin), netP.y_m + uy*(halfLen+btf) - py*(halfWid+sideMargin)],
            [netP.x_m - ux*(halfLen+btf) + px*(halfWid+sideMargin), netP.y_m - uy*(halfLen+btf) + py*(halfWid+sideMargin)],
            [netP.x_m - ux*(halfLen+btf) - px*(halfWid+sideMargin), netP.y_m - uy*(halfLen+btf) - py*(halfWid+sideMargin)],
          ];
          fenceRect = {
            minX: Math.min(...corners.map(c => c[0])),
            maxX: Math.max(...corners.map(c => c[0])),
            minY: Math.min(...corners.map(c => c[1])),
            maxY: Math.max(...corners.map(c => c[1])),
            source: "two_point_survey",
          };
        }
      }
      const axisAlignedFromFence = (
        !fenceRect
        || fenceRect.source !== "verified"
        || !net?.post_a
        || !net?.post_b
        || Math.abs(fenceRect.maxX - fenceRect.minX) < Math.abs(fenceRect.maxY - fenceRect.minY)
      );
      const courtCorners = (() => {
        if (axisAlignedFromFence) return [];
        if (!net?.post_a || !net?.post_b || !netCenter) return [];
        const ax = net.post_a.x_m, ay = net.post_a.y_m;
        const bx = net.post_b.x_m, by = net.post_b.y_m;
        const dx = bx - ax, dy = by - ay;
        const span = Math.hypot(dx, dy);
        if (!Number.isFinite(span) || span < 0.5) return [];
        const tx = dx / span, ty = dy / span;
        const nx = -ty, ny = tx;
        const halfL = courtLength / 2;
        const halfW = courtWidth / 2;
        return [
          { x_m: netCenter.x_m + nx * halfL + tx * halfW, y_m: netCenter.y_m + ny * halfL + ty * halfW },
          { x_m: netCenter.x_m + nx * halfL - tx * halfW, y_m: netCenter.y_m + ny * halfL - ty * halfW },
          { x_m: netCenter.x_m - nx * halfL - tx * halfW, y_m: netCenter.y_m - ny * halfL - ty * halfW },
          { x_m: netCenter.x_m - nx * halfL + tx * halfW, y_m: netCenter.y_m - ny * halfL + ty * halfW },
        ];
      })();
      const fallbackCourtCorners = [
        { x_m: -courtLength / 2, y_m: courtWidth / 2 },
        { x_m: courtLength / 2, y_m: courtWidth / 2 },
        { x_m: courtLength / 2, y_m: -courtWidth / 2 },
        { x_m: -courtLength / 2, y_m: -courtWidth / 2 },
      ];
      const liveCourtEstimate = (() => {
        const ref = surveyByLabel.first_net_left_turn_reference || netCenter;
        if (!ref || !Number.isFinite(ref.x_m) || !Number.isFinite(ref.y_m)) return [];
        if (netFrame) {
          const nx = netFrame.normal.x_m;
          const ny = netFrame.normal.y_m;
          const tx = netFrame.tangent.x_m;
          const ty = netFrame.tangent.y_m;
          return [
            { x_m: ref.x_m + nx * courtLength / 2 + tx * courtWidth / 2, y_m: ref.y_m + ny * courtLength / 2 + ty * courtWidth / 2 },
            { x_m: ref.x_m + nx * courtLength / 2 - tx * courtWidth / 2, y_m: ref.y_m + ny * courtLength / 2 - ty * courtWidth / 2 },
            { x_m: ref.x_m - nx * courtLength / 2 - tx * courtWidth / 2, y_m: ref.y_m - ny * courtLength / 2 - ty * courtWidth / 2 },
            { x_m: ref.x_m - nx * courtLength / 2 + tx * courtWidth / 2, y_m: ref.y_m - ny * courtLength / 2 + ty * courtWidth / 2 },
          ];
        }
        return [
          { x_m: ref.x_m - courtLength / 2, y_m: ref.y_m + courtWidth / 2 },
          { x_m: ref.x_m + courtLength / 2, y_m: ref.y_m + courtWidth / 2 },
          { x_m: ref.x_m + courtLength / 2, y_m: ref.y_m - courtWidth / 2 },
          { x_m: ref.x_m - courtLength / 2, y_m: ref.y_m - courtWidth / 2 },
        ];
      })();
      const displayCourtCorners = courtCorners.length === 4 ? courtCorners : (liveCourtEstimate.length === 4 ? liveCourtEstimate : fallbackCourtCorners);
      const courtRect = displayCourtCorners.length === 4 ? {
        minX: Math.min(...displayCourtCorners.map(p => p.x_m)),
        maxX: Math.max(...displayCourtCorners.map(p => p.x_m)),
        minY: Math.min(...displayCourtCorners.map(p => p.y_m)),
        maxY: Math.max(...displayCourtCorners.map(p => p.y_m)),
      } : {
        minX: -courtLength / 2, maxX: courtLength / 2,
        minY: -courtWidth / 2, maxY: courtWidth / 2,
      };
      const rawMargin = fenceRect ? {
        top: fenceRect.maxY - courtRect.maxY,
        bottom: courtRect.minY - fenceRect.minY,
        left: courtRect.minX - fenceRect.minX,
        right: fenceRect.maxX - courtRect.maxX,
      } : {};
      // Trustworthy display: a negative or missing clearance means that fence
      // side is not measured yet — show nothing rather than an impossible value.
      const cleanMargin = v => (Number.isFinite(v) && v >= 0 ? v : null);
      const margin = {
        top: cleanMargin(rawMargin.top),
        bottom: cleanMargin(rawMargin.bottom),
        left: cleanMargin(rawMargin.left),
        right: cleanMargin(rawMargin.right),
      };
      const marginValues = Object.values(margin).filter(v => Number.isFinite(v));
      const validPoints = points.filter(p => Number.isFinite(p.x_m) && Number.isFinite(p.y_m));
      let minX = fenceRect ? fenceRect.minX - 0.7 : courtRect.minX - 1.0;
      let maxX = fenceRect ? fenceRect.maxX + 0.7 : courtRect.maxX + 1.0;
      let minY = fenceRect ? fenceRect.minY - 0.7 : courtRect.minY - 1.0;
      let maxY = fenceRect ? fenceRect.maxY + 0.7 : courtRect.maxY + 1.0;
      [...validPoints, ...surveyPoints, net?.post_a, net?.post_b, netCenter, ...displayCourtCorners, robot, ...robotPath].filter(Boolean).forEach(p => {
        if (!Number.isFinite(p.x_m) || !Number.isFinite(p.y_m)) return;
        minX = Math.min(minX, p.x_m - 0.5);
        maxX = Math.max(maxX, p.x_m + 0.5);
        minY = Math.min(minY, p.y_m - 0.5);
        maxY = Math.max(maxY, p.y_m + 0.5);
      });
      const spanX = Math.max(1, maxX - minX);
      const spanY = Math.max(1, maxY - minY);
      const scale = Math.min((plotW - pad * 2) / spanX, (height - pad * 2) / spanY);
      const ox = (plotW - spanX * scale) / 2;
      const oy = (height - spanY * scale) / 2;
      const sx = x => ox + (x - minX) * scale;
      const sy = y => height - (oy + (y - minY) * scale);

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#090d12";
      ctx.fillRect(0, 0, width, height);
      ctx.strokeStyle = "rgba(255,255,255,0.07)";
      ctx.lineWidth = 1;
      for (let gx = Math.ceil(minX); gx <= Math.floor(maxX); gx += 1) {
        ctx.beginPath(); ctx.moveTo(sx(gx), sy(minY)); ctx.lineTo(sx(gx), sy(maxY)); ctx.stroke();
      }
      for (let gy = Math.ceil(minY); gy <= Math.floor(maxY); gy += 1) {
        ctx.beginPath(); ctx.moveTo(sx(minX), sy(gy)); ctx.lineTo(sx(maxX), sy(gy)); ctx.stroke();
      }

      // Clear-path view: suppress the estimate overlay until a new survey runs.
      if (discoveryCleared) {
        const liveDiscCount = ((diagnostics.court_survey_live || {}).navigation_points || []).length;
        if (liveDiscCount > discoveryClearBaseline) discoveryCleared = false;
      }
      const showOverlay = !discoveryCleared;

      ctx.fillStyle = "rgba(145,162,178,0.28)";
      if (showOverlay) validPoints.forEach(p => ctx.fillRect(sx(p.x_m) - 1, sy(p.y_m) - 1, 2, 2));
      const waypointStyle = {
        first_net_left_turn_reference: "#50dcff",
        net_to_fence_corner: "#a855f7",
        fence_corner: "#a855f7",
        net_detected_from_far_side: "#50dcff",
        crossing_net_right_side: "#50dcff",
        cross_net_exit: "#50dcff",
        second_half_start: "#50dcff",
        second_half_corner: "#a855f7",
        loop_closed: "#2fd08f",
      };
      const discoveredFenceSegments = [
        ["net_to_fence_corner", "fence_corner"],
        ["second_half_start", "second_half_corner"],
      ]
        .map(labels => labels.map(label => surveyByLabel[label]).filter(p => p && Number.isFinite(p.x_m) && Number.isFinite(p.y_m)))
        .filter(segment => segment.length > 1);
      const discoveredFencePoints = discoveredFenceSegments.flat();
      if (showOverlay) discoveredFenceSegments.forEach(segment => {
        ctx.strokeStyle = "rgba(168,85,247,0.95)";
        ctx.lineWidth = 3.5;
        ctx.setLineDash([9, 5]);
        ctx.beginPath();
        segment.forEach((p, i) => i === 0 ? ctx.moveTo(sx(p.x_m), sy(p.y_m)) : ctx.lineTo(sx(p.x_m), sy(p.y_m)));
        ctx.stroke();
        ctx.setLineDash([]);
      });
      if (showOverlay && netCenter && Number.isFinite(netCenter.x_m) && Number.isFinite(netCenter.y_m)) {
        const tx = netFrame ? netFrame.tangent.x_m : 0;
        const ty = netFrame ? netFrame.tangent.y_m : 1;
        const lineHalf = courtWidth / 2;
        ctx.strokeStyle = "rgba(80,220,255,0.88)";
        ctx.lineWidth = 3;
        ctx.setLineDash([6, 5]);
        ctx.beginPath();
        ctx.moveTo(sx(netCenter.x_m - tx * lineHalf), sy(netCenter.y_m - ty * lineHalf));
        ctx.lineTo(sx(netCenter.x_m + tx * lineHalf), sy(netCenter.y_m + ty * lineHalf));
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.fillStyle = "#50dcff";
        ctx.font = "bold 12px system-ui";
        ctx.fillText("discovered net line", sx(netCenter.x_m) + 8, sy(netCenter.y_m) - 8);
      }
      if (showOverlay) surveyPoints.forEach(p => {
        if (!Number.isFinite(p.x_m) || !Number.isFinite(p.y_m)) return;
        const color = waypointStyle[p.label] || "#ffbd5a";
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.arc(sx(p.x_m), sy(p.y_m), 4.5, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "rgba(238,244,248,0.78)";
        ctx.font = "10px system-ui";
        ctx.fillText(String(p.label || "point").replace(/_/g, " "), sx(p.x_m) + 7, sy(p.y_m) - 6);
      });
      if (showOverlay && fenceRect) {
        ctx.strokeStyle = discoveredFencePoints.length > 1 ? "rgba(168,85,247,0.36)" : "rgba(168,85,247,0.92)";
        ctx.lineWidth = 3;
        ctx.setLineDash(fenceRect.source === "verified" ? [] : [8, 6]);
        ctx.strokeRect(sx(fenceRect.minX), sy(fenceRect.maxY), sx(fenceRect.maxX) - sx(fenceRect.minX), sy(fenceRect.minY) - sy(fenceRect.maxY));
        ctx.setLineDash([]);
      }
      if (displayCourtCorners.length === 4) {
        ctx.fillStyle = "rgba(47,208,143,0.06)";
        ctx.beginPath();
        displayCourtCorners.forEach((p, i) => i === 0 ? ctx.moveTo(sx(p.x_m), sy(p.y_m)) : ctx.lineTo(sx(p.x_m), sy(p.y_m)));
        ctx.closePath();
        ctx.fill();
        ctx.strokeStyle = courtCorners.length === 4 ? "rgba(47,208,143,0.9)" : "rgba(47,208,143,0.48)";
        ctx.lineWidth = 2.5;
        ctx.setLineDash(courtCorners.length === 4 ? [] : [7, 5]);
        ctx.stroke();
        ctx.setLineDash([]);
      }
        // Internal court lines (service lines, centre service line, singles
        // sidelines, centre marks) drawn in the court's own frame derived from
        // the four corners — so they track the court even when it is rotated.
        const _C = displayCourtCorners;
        const _ctr = {
          x_m: (_C[0].x_m + _C[1].x_m + _C[2].x_m + _C[3].x_m) / 4,
          y_m: (_C[0].y_m + _C[1].y_m + _C[2].y_m + _C[3].y_m) / 4,
        };
        const _m01 = { x_m: (_C[0].x_m + _C[1].x_m) / 2, y_m: (_C[0].y_m + _C[1].y_m) / 2 };
        const _m12 = { x_m: (_C[1].x_m + _C[2].x_m) / 2, y_m: (_C[1].y_m + _C[2].y_m) / 2 };
        const _halfL = Math.hypot(_m01.x_m - _ctr.x_m, _m01.y_m - _ctr.y_m);
        const _halfW = Math.hypot(_m12.x_m - _ctr.x_m, _m12.y_m - _ctr.y_m);
        if (_halfL > 1 && _halfW > 1) {
          const _u = { x: (_m01.x_m - _ctr.x_m) / _halfL, y: (_m01.y_m - _ctr.y_m) / _halfL };
          const _v = { x: (_m12.x_m - _ctr.x_m) / _halfW, y: (_m12.y_m - _ctr.y_m) / _halfW };
          const _P = (a, b) => [sx(_ctr.x_m + _u.x * a + _v.x * b), sy(_ctr.y_m + _u.y * a + _v.y * b)];
          const _ln = (a1, b1, a2, b2) => {
            const p1 = _P(a1, b1), p2 = _P(a2, b2);
            ctx.beginPath(); ctx.moveTo(p1[0], p1[1]); ctx.lineTo(p2[0], p2[1]); ctx.stroke();
          };
          const _serviceA = Math.min(6.40, _halfL * 0.9);   // service line 6.40 m from net
          const _singlesB = Math.max(_halfW * 0.4, _halfW - 1.37); // singles sideline (doubles alley 1.37 m)
          ctx.strokeStyle = "rgba(47,208,143,0.55)";
          ctx.lineWidth = 1.4;
          ctx.setLineDash([]);
          _ln(-_halfL, _singlesB, _halfL, _singlesB);     // singles sideline +
          _ln(-_halfL, -_singlesB, _halfL, -_singlesB);    // singles sideline -
          _ln(_serviceA, -_singlesB, _serviceA, _singlesB);  // service line far
          _ln(-_serviceA, -_singlesB, -_serviceA, _singlesB); // service line near
          _ln(-_serviceA, 0, _serviceA, 0);                 // centre service line
          _ln(_halfL, 0, _halfL - 0.3, 0);                  // centre mark far baseline
          _ln(-_halfL, 0, -_halfL + 0.3, 0);                // centre mark near baseline
        }
      if (showOverlay && net?.post_a && net?.post_b) {
        ctx.strokeStyle = "rgba(80,220,255,0.95)";
        ctx.lineWidth = 4;
        ctx.beginPath();
        ctx.moveTo(sx(net.post_a.x_m), sy(net.post_a.y_m));
        ctx.lineTo(sx(net.post_b.x_m), sy(net.post_b.y_m));
        ctx.stroke();
      }

      function drawDistance(x1, y1, x2, y2, label) {
        if (![x1, y1, x2, y2].every(Number.isFinite)) return;
        ctx.strokeStyle = "rgba(255,189,90,0.92)";
        ctx.fillStyle = "#ffbd5a";
        ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
        ctx.font = "bold 13px system-ui";
        ctx.fillText(label, (x1 + x2) / 2 + 6, (y1 + y2) / 2 - 6);
      }
      if (showOverlay && fenceRect) {
        const midX = (courtRect.minX + courtRect.maxX) / 2;
        const midY = (courtRect.minY + courtRect.maxY) / 2;
        if (Number.isFinite(margin.left))   drawDistance(sx(fenceRect.minX), sy(midY), sx(courtRect.minX), sy(midY), fmt(margin.left, "m"));
        if (Number.isFinite(margin.right))  drawDistance(sx(courtRect.maxX), sy(midY), sx(fenceRect.maxX), sy(midY), fmt(margin.right, "m"));
        if (Number.isFinite(margin.top))    drawDistance(sx(midX), sy(courtRect.maxY), sx(midX), sy(fenceRect.maxY), fmt(margin.top, "m"));
        if (Number.isFinite(margin.bottom)) drawDistance(sx(midX), sy(fenceRect.minY), sx(midX), sy(courtRect.minY), fmt(margin.bottom, "m"));
      }
      const pathPoints = robotPath.filter(p => Number.isFinite(p.x_m) && Number.isFinite(p.y_m));
      if (pathPoints.length > 1) {
        ctx.strokeStyle = "rgba(255,136,0,0.85)";
        ctx.lineWidth = 2.5;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.beginPath();
        pathPoints.forEach((p, i) => i === 0 ? ctx.moveTo(sx(p.x_m), sy(p.y_m)) : ctx.lineTo(sx(p.x_m), sy(p.y_m)));
        ctx.stroke();
        const start = pathPoints[0];
        ctx.fillStyle = "#ff8800";
        ctx.beginPath(); ctx.arc(sx(start.x_m), sy(start.y_m), 4, 0, Math.PI * 2); ctx.fill();
      }
      const robotDraw = robotCourtPos
        ? { x_m: robotCourtPos.x_m, y_m: robotCourtPos.y_m, yaw_rad: robot.yaw_rad }
        : robot;
      if (Number.isFinite(robotDraw.x_m) && Number.isFinite(robotDraw.y_m)) {
        const rx = sx(robotDraw.x_m), ry = sy(robotDraw.y_m);
        const corrected = !!robotCourtPos;
        // Faint marker at the raw (uncorrected) pose, so the drift is visible.
        if (corrected && Number.isFinite(robot.x_m) && Number.isFinite(robot.y_m)) {
          ctx.strokeStyle = "rgba(120,140,160,0.5)";
          ctx.setLineDash([3, 3]);
          ctx.beginPath(); ctx.arc(sx(robot.x_m), sy(robot.y_m), 5, 0, Math.PI * 2); ctx.stroke();
          ctx.setLineDash([]);
        }
        ctx.fillStyle = corrected ? "rgba(120,230,170,0.18)" : "rgba(87,166,255,0.15)";
        ctx.beginPath(); ctx.arc(rx, ry, 28, 0, Math.PI * 2); ctx.fill();
        const mainColor = corrected ? "#2fd08f" : "#57a6ff";
        if (Number.isFinite(robotDraw.yaw_rad)) {
          const hx = Math.cos(robotDraw.yaw_rad), hy = -Math.sin(robotDraw.yaw_rad);
          const px = -hy, py = hx;
          ctx.fillStyle = mainColor;
          ctx.beginPath();
          ctx.moveTo(rx + hx * 16, ry + hy * 16);
          ctx.lineTo(rx + px * 7, ry + py * 7);
          ctx.lineTo(rx - px * 7, ry - py * 7);
          ctx.closePath();
          ctx.fill();
        }
        ctx.fillStyle = mainColor;
        ctx.beginPath(); ctx.arc(rx, ry, 6, 0, Math.PI * 2); ctx.fill();
        ctx.fillStyle = "#cfe2ff";
        ctx.font = "bold 12px system-ui";
        ctx.fillText(corrected ? "robot (cam)" : "robot", rx + 12, ry + 4);
      }

      ctx.fillStyle = "rgba(238,244,248,0.9)";
      ctx.font = "bold 13px system-ui";
      ctx.fillText("Court Map (Top-Down Occupancy Grid)", 18, 24);
      const legend = [["#57a6ff", "LiDAR Points"], ["#a855f7", "Outer Boundary"], ["#2fd08f", "Court Boundary"], ["#ffbd5a", "Distances"], ["#ff8800", "Survey Path"], ["#57a6ff", "Robot Pose"]];
      let lx = 24;
      ctx.font = "11px system-ui";
      legend.forEach(([color, label]) => {
        ctx.fillStyle = color; ctx.fillRect(lx, 45, 10, 10);
        ctx.fillStyle = "rgba(238,244,248,0.82)"; ctx.fillText(label, lx + 16, 54);
        lx += ctx.measureText(label).width + 42;
      });

      const panelX = plotW + 12;
      ctx.fillStyle = "rgba(10,18,26,0.76)";
      ctx.strokeStyle = "rgba(255,255,255,0.10)";
      ctx.beginPath(); ctx.roundRect(panelX, 78, width - panelX - 18, 260, 8); ctx.fill(); ctx.stroke();
      ctx.fillStyle = "rgba(238,244,248,0.92)";
      ctx.font = "bold 12px system-ui";
      ctx.fillText("Distances to Fences", panelX + 16, 104);
      const distRows = [
        ["Top", margin.top], ["Bottom", margin.bottom], ["Left", margin.left], ["Right", margin.right],
        ["Min Distance", marginValues.length ? Math.min(...marginValues) : null],
        ["Max Distance", marginValues.length ? Math.max(...marginValues) : null],
        ["Avg Distance", marginValues.length ? marginValues.reduce((a, b) => a + b, 0) / marginValues.length : null],
      ];
      let rowY = 136;
      distRows.forEach(([label, value], idx) => {
        if (idx === 4) rowY += 16;
        ctx.fillStyle = "rgba(145,162,178,0.9)"; ctx.font = "11px system-ui"; ctx.fillText(label, panelX + 16, rowY);
        ctx.fillStyle = "rgba(238,244,248,0.96)"; ctx.font = "bold 11px system-ui"; ctx.fillText((showOverlay && Number.isFinite(value)) ? fmt(value, "m") : "—", panelX + 120, rowY);
        rowY += 28;
      });

      if (statusEl) {
        const pointCount = nav.map_point_count ?? validPoints.length;
        const discoveryCount = surveyPoints.length;
        const boundaryMode = discoveredFencePoints.length > 1 ? "fence partially discovered" : (fenceRect ? fenceRect.source : "awaiting LiDAR");
        const courtMode = courtCorners.length === 4 ? "court boundary fitted" : (liveCourtEstimate.length === 4 ? "court estimated from net" : "court expected");
        statusEl.textContent = showOverlay
          ? `${pointCount} LiDAR points · ${discoveryCount} waypoints · ${boundaryMode} · ${courtMode} · net ${netSource}`
          : "path cleared · court + robot + trail";
        statusEl.style.color = discoveredFencePoints.length > 1 || (fenceRect?.source === "verified" && courtCorners.length === 4) ? "var(--accent)" : "var(--warn)";
      }
      const fenceLength = fenceRect ? fenceRect.maxX - fenceRect.minX : null;
      const fenceWidth = fenceRect ? fenceRect.maxY - fenceRect.minY : null;
      const segments = [
        ["F-Top", fenceLength, "Fence", fenceRect?.source === "verified"],
        ["F-Bottom", fenceLength, "Fence", fenceRect?.source === "verified"],
        ["F-Left", fenceWidth, "Fence", fenceRect?.source === "verified"],
        ["F-Right", fenceWidth, "Fence", fenceRect?.source === "verified"],
        ["C-Baseline Far", courtWidth, "Court", courtCorners.length === 4],
        ["C-Baseline Near", courtWidth, "Court", courtCorners.length === 4],
        ["C-Side Left", courtLength, "Court", courtCorners.length === 4],
        ["C-Side Right", courtLength, "Court", courtCorners.length === 4],
      ];
      if (metaEl) {
        metaEl.innerHTML = `
          <div><span>Outer Boundary</span><strong>${fenceRect ? `${fmt(fenceLength, "m")} x ${fmt(fenceWidth, "m")}` : "pending"}</strong></div>
          <div><span>Court Boundary</span><strong>${fmt(courtLength, "m")} x ${fmt(courtWidth, "m")}</strong></div>
          <div><span>Boundary Source</span><strong>${discoveredFencePoints.length > 1 ? "live waypoint discovery" : (fenceRect?.source || "none")}</strong></div>
          <div><span>Discovery State</span><strong>${discovery.state || survey.state || "idle"} · ${discovery.last_event || nav.last_event || "none"}</strong></div>
          <div><span>Discovered Points</span><strong>${surveyPoints.length}</strong></div>
          <div><span>Confidence</span><strong>${fmt((bounds.diagnostics || {}).confidence, "")}</strong></div>
          <div><span>Survey Path</span><strong>${(() => {
            const pp = robotPath.filter(p => Number.isFinite(p.x_m) && Number.isFinite(p.y_m));
            if (pp.length < 2) return "not started";
            let d = 0;
            for (let i = 1; i < pp.length; i++) d += Math.hypot(pp[i].x_m - pp[i-1].x_m, pp[i].y_m - pp[i-1].y_m);
            return `${fmt(d, "m")} · ${pp.length} pts`;
          })()}</strong></div>
          <div style="grid-column:1 / -1;display:block;">
            <div style="display:grid;grid-template-columns:1fr 1fr 1fr 1fr;gap:8px 14px;align-items:center;">
              <strong>ID</strong><strong>Length</strong><strong>Type</strong><strong>Status</strong>
              ${segments.map(([id, len, type, ok]) => `
                <span>${id}</span><span>${fmt(len, "m")}</span><span>${type}</span>
                <span style="color:${ok ? "var(--accent)" : "var(--muted)"}">${ok ? "Verified" : "Pending"}</span>
              `).join("")}
            </div>
          </div>
        `;
      }
    }
    const _obsLogLines = [];
    const OBS_LOG_MAX = 200;

    function renderObstacleSurveyDebug(nav) {
      const obs = nav.obstacle_survey;
      const stateEl = document.getElementById("obsState");
      const kvEl   = document.getElementById("obsKv");
      const logEl  = document.getElementById("obsLog");
      if (!stateEl || !kvEl || !logEl) return;

      if (!obs) {
        stateEl.textContent = "idle";
        stateEl.style.color = "var(--muted)";
        return;
      }

      const stateColor = {
        scan_in_place: "var(--accent-2)",
        approach_obstacle: "var(--warn)",
        done: obs.failure_reason ? "var(--danger)" : "var(--accent)",
      }[obs.state] || "var(--muted)";
      stateEl.textContent = obs.state || "idle";
      stateEl.style.color = stateColor;

      const fr = obs.front_range_m != null ? obs.front_range_m + " m" : "—";
      const nearest = (obs.debug_log && obs.debug_log.length)
        ? (obs.debug_log[obs.debug_log.length - 1].nearest_m ?? "—") + " m" : "—";
      setKv("obsKv", [
        ["State",          obs.state || "—"],
        ["Event",          obs.last_event || "—"],
        ["Front range",    fr],
        ["Nearest (360°)", nearest],
        ["Scans",          obs.scan_count ?? "—"],
        ["Traveled",       (obs.distance_traveled_m ?? "—") + " m"],
        ["Elapsed",        (obs.elapsed_s ?? "—") + " s"],
        ["Obstacle",       obs.obstacle_type ?? "—"],
        ["Confidence",     obs.obstacle_confidence != null ? (obs.obstacle_confidence * 100).toFixed(0) + "%" : "—"],
        ["Failure",        obs.failure_reason || "—"],
      ]);

      // Append new log entries (avoid duplicates by tracking last t seen)
      const entries = Array.isArray(obs.debug_log) ? obs.debug_log : [];
      const lastSeen = _obsLogLines.length ? _obsLogLines[_obsLogLines.length - 1]._t : -1;
      let newAdded = false;
      for (const e of entries) {
        if (e.t > lastSeen) {
          const vis = e.vision ? ` vis:${e.vision}` : "";
          const tgt = e.target_heading_deg != null ? ` tgt:${e.target_heading_deg}°` : "";
          const err = e.heading_err_deg != null ? ` err:${e.heading_err_deg}°` : "";
          const front = e.front_m != null ? e.front_m : "inf";
          const near  = e.nearest_m != null ? e.nearest_m : "inf";
          const line = `[${e.t.toFixed(2).padStart(6)}s] ${e.state.padEnd(20)} ${e.decision.padEnd(24)} front:${String(front).padStart(5)}m near:${String(near).padStart(5)}m bear:${String(e.bearing_deg).padStart(6)}°${tgt}${err} cmd:(${e.cmd_lin.toFixed(2)},${e.cmd_ang.toFixed(2)})${vis}`;
          _obsLogLines.push({ _t: e.t, text: line });
          newAdded = true;
        }
      }
      if (_obsLogLines.length > OBS_LOG_MAX) _obsLogLines.splice(0, _obsLogLines.length - OBS_LOG_MAX);
      if (newAdded) {
        logEl.textContent = _obsLogLines.map(l => l.text).join("\\n");
        logEl.scrollTop = logEl.scrollHeight;
      }
    }

    function renderObstacleRunsHistory(runs) {
      const el = document.getElementById("obsRunsTable");
      if (!el || !Array.isArray(runs) || runs.length === 0) return;
      const statusColor = s => s === "SUCCESS" ? "var(--accent)" : "var(--danger)";
      const fmtTime = ts => ts ? new Date(ts * 1000).toLocaleTimeString() : "—";
      let html = `<table style="width:100%;border-collapse:collapse;font-size:11px;">
        <thead><tr style="color:var(--muted);text-align:left;border-bottom:1px solid var(--line);">
          <th style="padding:3px 6px;">#</th>
          <th style="padding:3px 6px;">Time</th>
          <th style="padding:3px 6px;">Status</th>
          <th style="padding:3px 6px;">Obstacle</th>
          <th style="padding:3px 6px;">Dist</th>
          <th style="padding:3px 6px;">Elapsed</th>
          <th style="padding:3px 6px;">Traveled</th>
          <th style="padding:3px 6px;">Failure</th>
        </tr></thead><tbody>`;
      for (const r of runs) {
        html += `<tr style="border-bottom:1px solid var(--line)20;">
          <td style="padding:3px 6px;color:var(--muted);">${r.id}</td>
          <td style="padding:3px 6px;">${fmtTime(r.started_at)}</td>
          <td style="padding:3px 6px;color:${statusColor(r.status)};">${r.status || "—"}</td>
          <td style="padding:3px 6px;">${r.obstacle_type || "—"}</td>
          <td style="padding:3px 6px;">${r.obstacle_dist_m != null ? r.obstacle_dist_m.toFixed(2) + "m" : "—"}</td>
          <td style="padding:3px 6px;">${r.elapsed_s != null ? r.elapsed_s.toFixed(1) + "s" : "—"}</td>
          <td style="padding:3px 6px;">${r.distance_traveled_m != null ? r.distance_traveled_m.toFixed(1) + "m" : "—"}</td>
          <td style="padding:3px 6px;color:var(--danger);">${r.failure_reason || ""}</td>
        </tr>`;
      }
      html += "</tbody></table>";
      el.innerHTML = html;
    }

    function renderMapMission(m) {
      const statusEl = document.getElementById("mapMissionStatus");
      if (statusEl) {
        if (m.complete) statusEl.textContent = "— complete";
        else if (m.active) statusEl.textContent = `— ${m.phase_label || m.phase}`;
        else statusEl.textContent = "idle";
        statusEl.style.color = m.active ? "#a855f7" : (m.complete ? "var(--accent)" : "var(--muted)");
      }

      const hasData = m.active || m.complete;
      const grid = m.grid || [[0,0,0],[0,0,0],[0,0,0]];
      for (let r = 0; r < 3; r++) {
        for (let c = 0; c < 3; c++) {
          const el = document.getElementById(`mapCell${r}${c}`);
          if (!el) continue;
          if (!hasData) {
            el.textContent = "—";
            el.style.background = "var(--panel-2)";
            el.style.color = "var(--muted)";
            continue;
          }
          const count = (grid[r] || [])[c] ?? 0;
          el.textContent = count;
          if (count === 0) {
            el.style.background = "rgba(255,255,255,0.03)";
            el.style.color = "rgba(145,162,178,0.5)";
          } else if (count <= 2) {
            el.style.background = "rgba(87,166,255,0.13)";
            el.style.color = "#57a6ff";
          } else if (count <= 5) {
            el.style.background = "rgba(255,189,90,0.13)";
            el.style.color = "#ffbd5a";
          } else {
            el.style.background = "rgba(47,208,143,0.15)";
            el.style.color = "#2fd08f";
          }
        }
      }

      const progressEl = document.getElementById("mapMissionProgress");
      if (progressEl) {
        if (m.active && !m.complete) {
          progressEl.style.display = "block";
          const done = m.scan_poses_done ?? 0;
          const total = m.scan_poses_total ?? 5;
          const pct = m.scan_progress_pct != null
            ? Math.round(m.scan_progress_pct)
            : (total > 0 ? Math.round(done / total * 100) : 0);
          const phaseEl = document.getElementById("mapMissionPhaseLabel");
          const posesEl = document.getElementById("mapMissionPosesLabel");
          const barEl   = document.getElementById("mapMissionBar");
          if (phaseEl) phaseEl.textContent = m.phase_label || m.phase || "";
          if (posesEl) posesEl.textContent = m.scan_progress_pct != null ? `${pct}%` : `${done} / ${total} poses`;
          if (barEl)   barEl.style.width   = `${pct}%`;
        } else {
          progressEl.style.display = "none";
        }
      }

      const totalsEl = document.getElementById("mapMissionTotals");
      if (totalsEl && hasData) {
        const flat = grid.flat ? grid.flat() : [].concat(...grid);
        const gridSum = flat.reduce((a, b) => a + b, 0);
        totalsEl.textContent = `Candidates detected: ${m.total_candidates ?? 0} · Grid sum: ${gridSum} · Elapsed: ${fmt(m.elapsed_s, "s")}`;
      } else if (totalsEl) {
        totalsEl.textContent = "";
      }
    }
    function renderSensor(id, sensor) {
      const target = document.getElementById(id);
      if (!target) return;
      if (!sensor || !sensor.data_url) {
        target.className = "sensor-empty";
        target.textContent = "not available";
        return;
      }
      target.className = "";
      target.innerHTML = `<img src="${sensor.data_url}" alt="${id}">`;
    }
    function renderLidarSensor(id, sensor) {
      const target = document.getElementById(id);
      if (!target) return;
      const ranges = Array.isArray(sensor?.ranges_m) ? sensor.ranges_m : [];
      if (!ranges.length) {
        renderSensor(id, sensor);
        return;
      }
      const canvasId = `${id}Canvas`;
      const validRanges = ranges.filter(value => Number.isFinite(value) && value > 0);
      const minRange = Number.isFinite(sensor.min_range_m) ? sensor.min_range_m : 0.05;
      const maxRange = Number.isFinite(sensor.max_range_m) ? sensor.max_range_m : Math.max(1, ...validRanges);
      const nearest = validRanges.length ? Math.min(...validRanges) : null;
      const blocked = validRanges.filter(value => value < Math.min(1.2, maxRange)).length;
      target.className = "";
      target.innerHTML = `
        <canvas id="${canvasId}" width="520" height="520" aria-label="LiDAR 360 degree scan"></canvas>
        <div class="sensor-meta">
          <div>front<strong>${fmt(sensor.front_range_m, "m")}</strong></div>
          <div>nearest<strong>${fmt(nearest, "m")}</strong></div>
          <div>hits<strong>${validRanges.length}/${ranges.length}</strong></div>
        </div>
      `;
      drawLidarScan(document.getElementById(canvasId), ranges, minRange, maxRange, blocked, sensor);
    }
    function drawLidarScan(canvas, ranges, minRange, maxRange, candidates, sensor = {}) {
      if (!canvas) return;
      const ctx = canvas.getContext("2d");
      const width = canvas.width;
      const height = canvas.height;
      const cx = width / 2;
      const cy = height / 2;
      const radius = Math.min(width, height) * 0.43;
      const span = Math.max(0.001, maxRange - minRange);
      const candList = Array.isArray(candidates) ? candidates : [];
      const blockedCount = ranges.filter(v => Number.isFinite(v) && v > 0 && v < Math.min(1.5, maxRange)).length;

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#05080b";
      ctx.fillRect(0, 0, width, height);

      // range rings
      ctx.strokeStyle = "rgba(145,162,178,0.22)";
      ctx.lineWidth = 1;
      [0.25, 0.5, 0.75, 1].forEach(scale => {
        ctx.beginPath();
        ctx.arc(cx, cy, radius * scale, 0, Math.PI * 2);
        ctx.stroke();
        const rLabel = (minRange + span * scale).toFixed(1);
        ctx.fillStyle = "rgba(145,162,178,0.5)";
        ctx.font = "11px Inter, system-ui, sans-serif";
        ctx.fillText(`${rLabel}m`, cx + radius * scale + 3, cy - 3);
      });

      // forward indicator
      ctx.strokeStyle = "rgba(87,166,255,0.55)";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(cx, cy);
      ctx.lineTo(cx, cy - radius);
      ctx.stroke();
      ctx.fillStyle = "rgba(255,189,90,0.95)";
      ctx.beginPath();
      ctx.moveTo(cx, cy - radius - 12);
      ctx.lineTo(cx - 7, cy - radius + 3);
      ctx.lineTo(cx + 7, cy - radius + 3);
      ctx.closePath();
      ctx.fill();

      // LiDAR range points
      const angleMin = Number.isFinite(sensor.angle_min_rad) ? sensor.angle_min_rad : -Math.PI;
      const angleInc = Number.isFinite(sensor.angle_increment_rad) ? sensor.angle_increment_rad : (Math.PI * 2) / Math.max(1, ranges.length);
      // LiDAR frame: angle 0 = robot forward (+x), +angle = robot left (+y)
      // Canvas convention: forward = top, left = left (so x = cx - sin(θ), y = cy - cos(θ))
      ranges.forEach((value, index) => {
        if (!Number.isFinite(value) || value <= 0) return;
        const theta = angleMin + index * angleInc;
        const clamped = Math.max(minRange, Math.min(maxRange, value));
        const r = ((clamped - minRange) / span) * radius;
        const x = cx - Math.sin(theta) * r;
        const y = cy - Math.cos(theta) * r;
        const near = value < Math.min(1.2, maxRange);
        ctx.fillStyle = near ? "rgba(255,189,90,0.95)" : "rgba(47,208,143,0.88)";
        ctx.fillRect(x - 1.7, y - 1.7, 3.4, 3.4);
      });

      // Ball candidates overlay
      // robot_x_m = forward (+x), robot_y_m = left (+y) — same canvas transform as LiDAR points
      candList.forEach((cand, i) => {
        const rx = cand.robot_x_m;
        const ry = cand.robot_y_m;
        const dist = Math.hypot(rx, ry);
        if (dist < 0.1 || dist > maxRange) return;
        const bearing = Math.atan2(ry, rx);
        const r = Math.min(1.0, (dist - minRange) / span) * radius;
        const x = cx - Math.sin(bearing) * r;
        const y = cy - Math.cos(bearing) * r;
        // Pulsing ring
        ctx.beginPath();
        ctx.arc(x, y, 11, 0, Math.PI * 2);
        ctx.strokeStyle = "rgba(255,189,90,0.35)";
        ctx.lineWidth = 3;
        ctx.stroke();
        // Filled dot
        ctx.beginPath();
        ctx.arc(x, y, 6, 0, Math.PI * 2);
        ctx.fillStyle = "#ffbd5a";
        ctx.fill();
        ctx.strokeStyle = "#07110d";
        ctx.lineWidth = 1.5;
        ctx.stroke();
        // Label
        ctx.fillStyle = "#ffbd5a";
        ctx.font = "bold 11px Inter, system-ui, sans-serif";
        ctx.fillText(`${dist.toFixed(1)}m`, x + 9, y - 5);
      });

      // Robot dot
      ctx.fillStyle = "#eef4f8";
      ctx.beginPath();
      ctx.arc(cx, cy, 7, 0, Math.PI * 2);
      ctx.fill();

      // Status text
      ctx.fillStyle = "rgba(145,162,178,0.9)";
      ctx.font = "12px Inter, system-ui, sans-serif";
      ctx.fillText(`${blockedCount} near`, 14, height - 16);
      if (candList.length > 0) {
        ctx.fillStyle = "#ffbd5a";
        ctx.fillText(`${candList.length} candidate${candList.length > 1 ? "s" : ""}`, 14, height - 32);
      }
    }
    function renderSensors() {
      renderSensor("sensorsCameraView", sensors.front_camera);
      renderSensor("sensorsDepthView", sensors.front_depth);
      renderLidarSensorFull("lidarScanView", sensors.front_lidar, sensors.lidar_candidates || []);
      renderIrIntake(sensors.ir_intake);
    }
    function renderIrIntake(ir) {
      const threshold = ir?.threshold ?? 500;
      function renderOne(valueId, barId, panelId, value, available) {
        const valEl = document.getElementById(valueId);
        const barEl = document.getElementById(barId);
        const panelEl = document.getElementById(panelId);
        if (!valEl || !barEl || !panelEl) return;
        if (!available || value === null || value === undefined) {
          valEl.textContent = "N/A";
          barEl.style.width = "0%";
          barEl.style.background = "rgba(255,255,255,0.15)";
          panelEl.style.borderColor = "var(--line)";
          return;
        }
        const pct = Math.min(100, Math.round((value / 1000) * 100));
        const triggered = value > threshold;
        valEl.textContent = Math.round(value);
        barEl.style.width = `${pct}%`;
        barEl.style.background = triggered ? "#2fd08f" : "rgba(145,162,178,0.45)";
        panelEl.style.borderColor = triggered ? "rgba(47,208,143,0.55)" : "var(--line)";
      }
      renderOne("irLeftValue", "irLeftBar", "irLeftPanel", ir?.left, ir?.left_available ?? (ir !== undefined));
      renderOne("irRightValue", "irRightBar", "irRightPanel", ir?.right, ir?.right_available ?? (ir !== undefined));
      const badge = document.getElementById("irTriggeredBadge");
      if (badge) {
        const triggered = !!ir?.triggered;
        const available = ir?.left_available || ir?.right_available;
        badge.textContent = available ? (triggered ? "TRIGGERED: YES — collection gate open" : "TRIGGERED: NO — ball not in intake zone") : "TRIGGERED: sensors not available";
        badge.style.background = triggered ? "rgba(47,208,143,0.15)" : (available ? "rgba(255,255,255,0.04)" : "rgba(255,80,80,0.10)");
        badge.style.color = triggered ? "#2fd08f" : (available ? "var(--muted)" : "#ff6060");
        badge.style.border = triggered ? "1px solid rgba(47,208,143,0.35)" : "none";
      }
    }
    function renderLidarSensorFull(id, sensor, candidates) {
      const target = document.getElementById(id);
      if (!target) return;
      const ranges = Array.isArray(sensor?.ranges_m) ? sensor.ranges_m : [];
      if (!ranges.length) {
        target.className = "sensor-empty";
        target.style.cssText = "background:#090d12;border:1px solid var(--line);border-radius:8px;min-height:200px;";
        target.textContent = "waiting for LiDAR scan";
        return;
      }
      const canvasId = "lidarFullCanvas";
      const validRanges = ranges.filter(v => Number.isFinite(v) && v > 0);
      const minRange = Number.isFinite(sensor.min_range_m) ? sensor.min_range_m : 0.05;
      const maxRange = Number.isFinite(sensor.max_range_m) ? sensor.max_range_m : Math.max(1, ...validRanges);
      const nearest = validRanges.length ? Math.min(...validRanges) : null;
      const blocked = validRanges.filter(v => v < Math.min(1.5, maxRange)).length;
      const candList = Array.isArray(candidates) ? candidates : [];
      target.className = "";
      target.style.cssText = "background:#090d12;border:1px solid var(--line);border-radius:8px;overflow:hidden;";
      target.innerHTML = `<canvas id="${canvasId}" width="700" height="700" style="display:block;width:100%;max-height:240px;object-fit:contain;" aria-label="LiDAR 360 degree scan"></canvas>`;
      const metaEl = document.getElementById("lidarScanMeta");
      if (metaEl) metaEl.innerHTML = [
        ["front", `<strong style="color:var(--ink);display:block;font-size:13px;">${fmt(sensor.front_range_m, "m")}</strong>`],
        ["nearest", `<strong style="color:var(--ink);display:block;font-size:13px;">${fmt(nearest, "m")}</strong>`],
        ["hits", `<strong style="color:var(--ink);display:block;font-size:13px;">${validRanges.length}/${ranges.length}</strong>`],
        ["candidates", `<strong style="color:${candList.length > 0 ? "#ffbd5a" : "var(--muted)"};display:block;font-size:13px;">${candList.length}</strong>`],
      ].map(([label, val]) => `<div style="border-top:1px solid var(--line);padding-top:8px;">${label}${val}</div>`).join("");
      drawLidarScan(document.getElementById(canvasId), ranges, minRange, maxRange, candList, sensor);
    }
    function renderCourtMap() {
      const canvas = document.getElementById("courtMap");
      if (!canvas) return;
      const ctx = canvas.getContext("2d");
      const map = (diagnostics.robot || {}).map || {};
      const court = map.court || { min_x: -11.885, max_x: 11.885, min_y: -5.485, max_y: 5.485, net_x: 0 };
      const width = canvas.width;
      const height = canvas.height;
      const pad = 42;
      const sx = x => pad + (x - court.min_x) / (court.max_x - court.min_x) * (width - pad * 2);
      const sy = y => height - pad - (y - court.min_y) / (court.max_y - court.min_y) * (height - pad * 2);
      const scaleM = (width - pad * 2) / (court.max_x - court.min_x);

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#7a3329";
      ctx.fillRect(0, 0, width, height);
      ctx.fillStyle = "#8f3f32";
      ctx.fillRect(pad, pad, width - pad * 2, height - pad * 2);

      ctx.strokeStyle = "rgba(255,255,255,0.78)";
      ctx.lineWidth = 3;
      ctx.strokeRect(pad, pad, width - pad * 2, height - pad * 2);
      ctx.beginPath();
      ctx.moveTo(sx(court.net_x || 0), pad);
      ctx.lineTo(sx(court.net_x || 0), height - pad);
      ctx.strokeStyle = "rgba(18,24,30,0.95)";
      ctx.lineWidth = 5;
      ctx.stroke();

      ctx.strokeStyle = "rgba(255,255,255,0.55)";
      ctx.lineWidth = 2;
      [-6.4, 6.4].forEach(x => {
        ctx.beginPath();
        ctx.moveTo(sx(x), pad);
        ctx.lineTo(sx(x), height - pad);
        ctx.stroke();
      });
      [-4.115, 4.115, 0].forEach(y => {
        ctx.beginPath();
        ctx.moveTo(pad, sy(y));
        ctx.lineTo(width - pad, sy(y));
        ctx.stroke();
      });

      const bounds = map.active_bounds;
      if (bounds) {
        ctx.fillStyle = "rgba(87,166,255,0.06)";
        ctx.fillRect(sx(bounds.min_x), sy(bounds.max_y), sx(bounds.max_x) - sx(bounds.min_x), sy(bounds.min_y) - sy(bounds.max_y));
      }

      // Survey fence measurements overlay — prefer persistent court_boundary.json over in-memory survey state
      const survBounds = diagnostics.court_boundary || ((diagnostics.robot || {}).survey || {}).bounds;
      if (survBounds && survBounds.survey_complete) {
        const fg = canonicalFenceBounds(survBounds);
        ctx.save();
        ctx.setLineDash([7, 4]);
        ctx.lineWidth = 1.8;
        ctx.strokeStyle = "rgba(255,189,90,0.75)";
        ctx.fillStyle = "rgba(255,189,90,0.92)";
        ctx.font = "bold 11px system-ui";

        if (fg.west_x != null) {
          const fx = sx(fg.west_x);
          ctx.beginPath(); ctx.moveTo(fx, pad); ctx.lineTo(fx, height - pad); ctx.stroke();
          ctx.textAlign = "right";
          ctx.fillText(`W ${fg.west_x.toFixed(1)}m`, fx - 3, pad + 20);
        }
        if (fg.east_x != null) {
          const fx = sx(fg.east_x);
          ctx.beginPath(); ctx.moveTo(fx, pad); ctx.lineTo(fx, height - pad); ctx.stroke();
          ctx.textAlign = "left";
          ctx.fillText(`E ${fg.east_x.toFixed(1)}m`, fx + 3, pad + 20);
        }
        if (fg.south_y != null) {
          const fy = sy(fg.south_y);
          ctx.beginPath(); ctx.moveTo(pad, fy); ctx.lineTo(width - pad, fy); ctx.stroke();
          ctx.textAlign = "left";
          ctx.fillText(`S ${fg.south_y.toFixed(1)}m`, pad + 6, fy - 4);
        }
        if (fg.north_y != null) {
          const fy = sy(fg.north_y);
          ctx.beginPath(); ctx.moveTo(pad, fy); ctx.lineTo(width - pad, fy); ctx.stroke();
          ctx.textAlign = "left";
          ctx.fillText(`N ${fg.north_y.toFixed(1)}m`, pad + 6, fy + 13);
        }

        // Width × depth badge bottom-right
        if (fg.east_x != null && fg.west_x != null && fg.north_y != null && fg.south_y != null) {
          const w = (fg.east_x - fg.west_x).toFixed(1);
          const d = (fg.north_y - fg.south_y).toFixed(1);
          const label = `map court: ${w} × ${d} m`;
          ctx.setLineDash([]);
          ctx.textAlign = "right";
          const lw = ctx.measureText(label).width;
          ctx.fillStyle = "rgba(12,17,22,0.72)";
          ctx.fillRect(width - pad - lw - 16, height - pad - 26, lw + 14, 20);
          ctx.fillStyle = "rgba(255,189,90,0.92)";
          ctx.fillText(label, width - pad - 4, height - pad - 11);
        }

        ctx.setLineDash([]);
        ctx.restore();
      }

      // camera FOV cone
      const robot = map.robot || {};
      if (robot.x_m !== undefined && robot.y_m !== undefined) {
        const rx = sx(robot.x_m);
        const ry = sy(robot.y_m);
        const yaw = robot.yaw_rad || 0;
        const fov = map.camera_fov_rad || 1.05;
        const fovRange = (map.camera_max_range_m || 4.5) * scaleM;
        ctx.save();
        ctx.translate(rx, ry);
        ctx.rotate(-yaw);
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.arc(0, 0, fovRange, -fov / 2, fov / 2);
        ctx.closePath();
        ctx.fillStyle = "rgba(87,166,255,0.07)";
        ctx.fill();
        ctx.strokeStyle = "rgba(87,166,255,0.22)";
        ctx.lineWidth = 1.5;
        ctx.setLineDash([6, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.restore();
      }

      const route = map.route || [];
      if (route.length > 1) {
        ctx.beginPath();
        route.forEach((point, index) => {
          const x = sx(point.x_m);
          const y = sy(point.y_m);
          if (index === 0) ctx.moveTo(x, y);
          else ctx.lineTo(x, y);
        });
        ctx.strokeStyle = "#57a6ff";
        ctx.lineWidth = 5;
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        ctx.stroke();
      }

      const activeTargetId = map.active_target_id;
      const allBalls = map.balls || [];

      // pending balls (below seen_count threshold) — drawn first so confirmed render on top
      allBalls.filter(b => !b.confirmed).forEach(ball => {
        const x = sx(ball.x_m);
        const y = sy(ball.y_m);
        ctx.globalAlpha = 0.35;
        ctx.beginPath();
        ctx.arc(x, y, 5, 0, Math.PI * 2);
        ctx.fillStyle = ball.side === "across_net" ? "#8793a0" : "#d7e85f";
        ctx.fill();
        ctx.setLineDash([3, 3]);
        ctx.strokeStyle = "rgba(255,255,255,0.4)";
        ctx.lineWidth = 1;
        ctx.stroke();
        ctx.setLineDash([]);
        ctx.globalAlpha = 1.0;
      });

      // confirmed balls
      allBalls.filter(b => b.confirmed).forEach(ball => {
        const x = sx(ball.x_m);
        const y = sy(ball.y_m);
        const isActive = ball.id === activeTargetId;
        const radius = ball.planned ? 9 : 7;

        // active target ring
        if (isActive) {
          ctx.beginPath();
          ctx.arc(x, y, radius + 6, 0, Math.PI * 2);
          ctx.strokeStyle = "#2fd08f";
          ctx.lineWidth = 2.5;
          ctx.stroke();
        }

        ctx.beginPath();
        ctx.arc(x, y, radius, 0, Math.PI * 2);
        ctx.fillStyle = ball.side === "across_net" ? "#8793a0" : (ball.visible_candidate ? "#2fd08f" : "#d7e85f");
        ctx.fill();
        ctx.strokeStyle = ball.source === "oak_depth" ? "#57a6ff" : (ball.planned ? "#ffffff" : "rgba(0,0,0,0.45)");
        ctx.lineWidth = ball.source === "oak_depth" ? 3 : (ball.planned ? 3 : 1);
        ctx.stroke();

        if (ball.order) {
          ctx.fillStyle = "#07110d";
          ctx.font = "bold 12px system-ui";
          ctx.textAlign = "center";
          ctx.textBaseline = "middle";
          ctx.fillText(String(ball.order), x, y);
        }
      });

      // robot arrow
      if (robot.x_m !== undefined && robot.y_m !== undefined) {
        const x = sx(robot.x_m);
        const y = sy(robot.y_m);
        const yaw = robot.yaw_rad || 0;
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(-yaw);
        ctx.fillStyle = "#ffbd5a";
        ctx.beginPath();
        ctx.moveTo(16, 0);
        ctx.lineTo(-10, -9);
        ctx.lineTo(-10, 9);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }

      // summary box
      const metrics = map.metrics || {};
      const confirmedCount = allBalls.filter(b => b.confirmed && b.side !== "across_net").length;
      const depthCount = allBalls.filter(b => b.confirmed && b.source === "oak_depth" && b.side !== "across_net").length;
      const pendingCount = allBalls.filter(b => !b.confirmed && b.side !== "across_net").length;
      const plannedCount = metrics.balls_collectable ?? 0;
      const summaryLine1 = `confirmed ${confirmedCount} · depth ${depthCount} · planned ${plannedCount}${pendingCount > 0 ? ` · pending ${pendingCount}` : ""}`;
      const summaryLine2 = metrics.total_distance_m != null
        ? `distance ${fmt(metrics.total_distance_m, "m")} · replans ${metrics.planned_replans ?? 0}`
        : "no route planned";
      const boxW = Math.max(280, ctx.measureText(summaryLine1).width + 28);
      ctx.fillStyle = "rgba(12,17,22,0.76)";
      ctx.fillRect(pad + 10, pad + 10, boxW, 64);
      ctx.fillStyle = "#eef4f8";
      ctx.font = "16px system-ui";
      ctx.textAlign = "left";
      ctx.fillText(summaryLine1, pad + 24, pad + 36);
      ctx.fillStyle = "#91a2b2";
      ctx.font = "13px system-ui";
      ctx.fillText(summaryLine2, pad + 24, pad + 58);
    }
    function renderHistory() {
      const history = diagnostics.history || [];
      document.getElementById("latestEvents").innerHTML = history.slice(-8).reverse().map((row, index) => (
        `<div class="event"><span>${dateText(row.updated_at)}</span><strong>${row.mode}</strong><span>sequence ${row.sequence} · ${row.source}${index === 0 ? " · latest" : ""}</span></div>`
      )).join("") || "<div class='event'><span>none</span><strong>No commands</strong><span>Waiting for input</span></div>";
      document.getElementById("historyRows").innerHTML = history.slice().reverse().map((row, index) => (
        `<tr class="${index === 0 ? "latest" : ""}"><td>${dateText(row.updated_at)}</td><td>${row.mode}</td><td>${row.sequence}</td><td>${row.source}</td></tr>`
      )).join("");
    }
    function renderStats() {
      const stats = diagnostics.stats || {};
      const total = stats.total || 0;
      const byMode = stats.by_mode || {};
      document.getElementById("sTotal").textContent = total;
      document.getElementById("sSurvey").textContent = byMode.map_court || 0;
      document.getElementById("sCollect").textContent = byMode.map_left_side || 0;
      document.getElementById("sIdle").textContent = byMode.idle || 0;
      document.getElementById("statsRows").innerHTML = ["map_court", "map_left_side", "idle"].map(mode => {
        const count = byMode[mode] || 0;
        const latest = (stats.latest_by_mode || {})[mode] || {};
        const share = total ? `${Math.round(count * 100 / total)}%` : "0%";
        return `<tr><td>${mode}</td><td>${count}</td><td>${share}</td><td>${latest.sequence ?? "none"}</td><td>${latest.source ?? "none"}</td></tr>`;
      }).join("");
    }
    // Inject the default (dashboard) view, then start the live refresh loop.
    // render() is null-safe, so the loop tolerates views that load later.
    loadView("dashboard").finally(() => {
      refresh();
      setInterval(refresh, 1000);
    });
    // Load vendor/session data at startup so sidebar status and command gating
    // work before the Vendors view is opened.
    window.ControlPanelVendors.setOnChange(updateCommandButtons);
    window.ControlPanelVendors.load();
    VIEW_INIT.vendors = function () { window.ControlPanelVendors.initView(); };
