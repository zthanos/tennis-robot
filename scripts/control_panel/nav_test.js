/* Nav Test module — manual NavigateToPose movement checks inside the surveyed
 * court, used from the Collection workspace to validate that the robot can be
 * driven to a point in the mapped space before building the collection plan.
 *
 * Extracted from app.js so the nav-test logic is self-contained and easier to
 * maintain. app.js owns the shared `diagnostics` state and `refresh()`, which
 * are injected via init(ctx). View wiring calls ControlPanelNavTest.wire() when
 * the Collection view mounts.
 *
 *   window.ControlPanelNavTest.init({ getDiagnostics, refresh })
 *   window.ControlPanelNavTest.wire()
 */
window.ControlPanelNavTest = (() => {
  "use strict";

  // Must match NavTestRunner.BOUNDS_MARGIN_M on the backend.
  const BOUNDS_MARGIN_M = 0.5;

  let _getDiagnostics = () => ({});
  let _refresh = () => {};
  let _log = () => {};
  let _onGoal = () => {};

  function init(ctx) {
    if (ctx && typeof ctx.getDiagnostics === "function") _getDiagnostics = ctx.getDiagnostics;
    if (ctx && typeof ctx.refresh === "function") _refresh = ctx.refresh;
    if (ctx && typeof ctx.log === "function") _log = ctx.log;
    if (ctx && typeof ctx.onGoal === "function") _onGoal = ctx.onGoal;
  }

  function diagnostics() {
    return _getDiagnostics() || {};
  }

  function currentRobotPose() {
    const robot = diagnostics().robot || {};
    const nested = robot.robot || {};
    const x = Number.isFinite(nested.x_m) ? nested.x_m : robot.robot_x_m;
    const y = Number.isFinite(nested.y_m) ? nested.y_m : robot.robot_y_m;
    const yaw = Number.isFinite(nested.yaw_rad) ? nested.yaw_rad : robot.robot_yaw_rad;
    if (!Number.isFinite(x) || !Number.isFinite(y)) return null;
    return { x_m: x, y_m: y, yaw_rad: Number.isFinite(yaw) ? yaw : 0 };
  }

  // Fence rectangle in the map frame, derived from the same source the backend
  // validates against (court_boundary.fence.corners). Returns null when no
  // survey is available, in which case the goal is allowed.
  function fenceBounds() {
    const corners = ((diagnostics().court_boundary || {}).fence || {}).corners || [];
    const xs = corners.filter(c => Number.isFinite(c?.x_m)).map(c => c.x_m);
    const ys = corners.filter(c => Number.isFinite(c?.y_m)).map(c => c.y_m);
    if (xs.length < 2 || ys.length < 2) return null;
    return {
      west_x: Math.min(...xs), east_x: Math.max(...xs),
      south_y: Math.min(...ys), north_y: Math.max(...ys),
    };
  }

  function withinBounds(x, y) {
    const b = fenceBounds();
    if (!b) return true;
    const m = BOUNDS_MARGIN_M;
    return (
      b.west_x - m <= x && x <= b.east_x + m &&
      b.south_y - m <= y && y <= b.north_y + m
    );
  }

  function setStatus(text, tone = "muted") {
    const status = document.getElementById("navTestStatus");
    const output = document.getElementById("navTestOutput");
    const color = tone === "ok" ? "var(--accent)"
      : tone === "error" ? "var(--danger)"
      : tone === "warn" ? "var(--warn)"
      : "var(--muted)";
    if (status) { status.textContent = text; status.style.color = color; }
    if (output) { output.textContent = text; output.style.color = color; }
  }

  function setInputs(pose) {
    const xEl = document.getElementById("navGoalX");
    const yEl = document.getElementById("navGoalY");
    const yawEl = document.getElementById("navGoalYaw");
    if (!xEl || !yEl || !yawEl || !pose) return;
    xEl.value = Number(pose.x_m).toFixed(2);
    yEl.value = Number(pose.y_m).toFixed(2);
    yawEl.value = Number(pose.yaw_rad || 0).toFixed(2);
  }

  function readInputs() {
    const x = Number(document.getElementById("navGoalX")?.value);
    const y = Number(document.getElementById("navGoalY")?.value);
    const yaw = Number(document.getElementById("navGoalYaw")?.value || 0);
    if (![x, y, yaw].every(Number.isFinite)) return null;
    return { x_m: x, y_m: y, yaw_rad: yaw };
  }

  async function sendGoal() {
    const send = document.getElementById("navSendGoal");
    const pose = readInputs();
    if (!pose) { setStatus("Enter numeric x, y, yaw", "error"); return; }
    const robot = diagnostics().robot || {};
    if ((robot.actual_mode || robot.mode) !== "idle") {
      setStatus("Stop robot before Nav Test", "warn");
      return;
    }
    const goal = `(${pose.x_m.toFixed(2)}, ${pose.y_m.toFixed(2)})`;
    if (!withinBounds(pose.x_m, pose.y_m)) {
      setStatus("Goal is outside the surveyed court bounds", "error");
      _log("nav_test_out_of_bounds", { goal });
      return;
    }
    // Show the intended target on the Collection Map immediately.
    _onGoal({ x_m: pose.x_m, y_m: pose.y_m, yaw_rad: pose.yaw_rad });
    if (send) send.disabled = true;
    setStatus(`Sending ${goal}`, "muted");
    // Lifecycle timestamps: log dispatch now, then elapsed time on the result.
    const t0 = Date.now();
    const elapsed = () => `${((Date.now() - t0) / 1000).toFixed(1)}s`;
    _log("nav_test_dispatch", { goal });
    try {
      const response = await fetch("/api/nav-test", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(pose),
      });
      const result = await response.json().catch(() => ({}));
      if (!response.ok || !result.ok) {
        const msg = result.out_of_bounds
          ? "Goal is outside the surveyed court bounds"
          : (result.message || `Nav goal failed (${response.status})`);
        setStatus(msg, "error");
        if (result.out_of_bounds) _log("nav_test_out_of_bounds", { goal });
        else if (result.timeout) _log("nav_test_timeout", { goal, elapsed: elapsed() });
        else _log("nav_test_error", { goal, detail: msg, elapsed: elapsed() });
        return;
      }
      setStatus(result.succeeded ? "Nav goal succeeded" : "Nav goal sent", result.succeeded ? "ok" : "warn");
      _log(result.succeeded ? "nav_test_succeeded" : "nav_test_sent", { goal, elapsed: elapsed() });
      await _refresh();
    } catch (error) {
      setStatus(`Nav request failed: ${error.message || error}`, "error");
      _log("nav_test_error", { goal, detail: String(error.message || error), elapsed: elapsed() });
    } finally {
      if (send) send.disabled = false;
    }
  }

  async function cancelGoal() {
    const cancel = document.getElementById("navCancelGoal");
    if (cancel) cancel.disabled = true;
    setStatus("Cancelling goal…", "muted");
    try {
      const response = await fetch("/api/nav-test/cancel", { method: "POST" });
      const result = await response.json().catch(() => ({}));
      if (!response.ok || !result.ok) {
        setStatus(result.message || `Cancel failed (${response.status})`, "error");
        return;
      }
      setStatus(result.canceled ? "Goal cancel requested" : "No active goal to cancel", result.canceled ? "ok" : "warn");
      _log("nav_test_cancel", { result: result.canceled ? "requested" : "no active goal" });
      if (result.canceled) _onGoal(null);  // clear the goal marker
      await _refresh();
    } catch (error) {
      setStatus(`Cancel request failed: ${error.message || error}`, "error");
    } finally {
      if (cancel) cancel.disabled = false;
    }
  }

  function wire() {
    const useCurrent = document.getElementById("navUseCurrent");
    const forward = document.getElementById("navForwardSmall");
    const send = document.getElementById("navSendGoal");
    const cancel = document.getElementById("navCancelGoal");
    if (useCurrent) useCurrent.addEventListener("click", () => {
      const pose = currentRobotPose();
      if (!pose) { setStatus("No live robot pose", "error"); return; }
      setInputs(pose);
      setStatus("Loaded current pose", "ok");
    });
    if (forward) forward.addEventListener("click", () => {
      const pose = currentRobotPose();
      if (!pose) { setStatus("No live robot pose", "error"); return; }
      const goal = {
        x_m: pose.x_m + Math.cos(pose.yaw_rad) * 0.5,
        y_m: pose.y_m + Math.sin(pose.yaw_rad) * 0.5,
        yaw_rad: pose.yaw_rad,
      };
      setInputs(goal);
      setStatus(withinBounds(goal.x_m, goal.y_m)
        ? "Prepared forward 0.5m goal"
        : "Forward 0.5m goal is outside court bounds", withinBounds(goal.x_m, goal.y_m) ? "ok" : "warn");
    });
    if (send) send.addEventListener("click", sendGoal);
    if (cancel) cancel.addEventListener("click", cancelGoal);
  }

  return { init, wire };
})();
