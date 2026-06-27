/* Collection Map renderer (window.ControlPanelCollectionMap).
 *
 * Draws the full Collection-workspace court canvas: court lines, fence overlay,
 * camera FOV, route/lane waypoints, balls (mapped + live camera detections),
 * the robot arrow, and the Nav Test goal marker. Extracted from app.js.
 *
 *   window.ControlPanelCollectionMap.render(diagnostics)
 *   window.ControlPanelCollectionMap.setGoal(goal | null)   // Nav Test marker
 */
window.ControlPanelCollectionMap = (() => {
  "use strict";
  const { courtFrameModel, canonicalFenceBounds, fmt } = window.ConsoleUtils;

  // Last Nav Test goal (map frame) drawn as a marker; set via nav_test onGoal.
  let _navTestGoal = null;
  function setGoal(goal) { _navTestGoal = goal; }

  function render(diagnostics) {
      const canvas = document.getElementById("courtMap");
      if (!canvas) return;
      const ctx = canvas.getContext("2d");
      const robotStatus = diagnostics.robot || {};
      const map = robotStatus.map || {};
      const survBounds = diagnostics.court_boundary || (robotStatus.survey || {}).bounds;
      const mapRobot = map.robot || {};
      const liveRobot = robotStatus.robot || {};
      const robotPose = {
        x_m: Number.isFinite(mapRobot.x_m) ? mapRobot.x_m : (Number.isFinite(liveRobot.x_m) ? liveRobot.x_m : robotStatus.robot_x_m),
        y_m: Number.isFinite(mapRobot.y_m) ? mapRobot.y_m : (Number.isFinite(liveRobot.y_m) ? liveRobot.y_m : robotStatus.robot_y_m),
        yaw_rad: Number.isFinite(mapRobot.yaw_rad) ? mapRobot.yaw_rad : (Number.isFinite(liveRobot.yaw_rad) ? liveRobot.yaw_rad : robotStatus.robot_yaw_rad),
      };
      const courtFrame = courtFrameModel(survBounds);
      const court = courtFrame
        ? {
            min_x: courtFrame.bounds.min_x - 1.0,
            max_x: courtFrame.bounds.max_x + 1.0,
            min_y: courtFrame.bounds.min_y - 1.0,
            max_y: courtFrame.bounds.max_y + 1.0,
            net_x: courtFrame.bounds.net_x,
          }
        : (map.court || { min_x: -11.885, max_x: 11.885, min_y: -5.485, max_y: 5.485, net_x: 0 });
      const width = canvas.width;
      const height = canvas.height;
      const pad = 42;
      const sx = x => pad + (x - court.min_x) / (court.max_x - court.min_x) * (width - pad * 2);
      const sy = y => height - pad - (y - court.min_y) / (court.max_y - court.min_y) * (height - pad * 2);
      const scaleM = (width - pad * 2) / (court.max_x - court.min_x);
      const strokeCourtLine = (x0, y0, x1, y1, style = "rgba(255,255,255,0.55)", lineWidth = 2) => {
        const a = courtFrame ? courtFrame.toMap(x0, y0) : { x_m: x0, y_m: y0 };
        const b = courtFrame ? courtFrame.toMap(x1, y1) : { x_m: x1, y_m: y1 };
        ctx.beginPath();
        ctx.moveTo(sx(a.x_m), sy(a.y_m));
        ctx.lineTo(sx(b.x_m), sy(b.y_m));
        ctx.strokeStyle = style;
        ctx.lineWidth = lineWidth;
        ctx.stroke();
      };
      const fillCourtPolygon = (points, fillStyle, strokeStyle = null, lineWidth = 1) => {
        const mapped = points.map(([x, y]) => courtFrame ? courtFrame.toMap(x, y) : { x_m: x, y_m: y });
        ctx.beginPath();
        mapped.forEach((point, index) => {
          if (index === 0) ctx.moveTo(sx(point.x_m), sy(point.y_m));
          else ctx.lineTo(sx(point.x_m), sy(point.y_m));
        });
        ctx.closePath();
        ctx.fillStyle = fillStyle;
        ctx.fill();
        if (strokeStyle) {
          ctx.strokeStyle = strokeStyle;
          ctx.lineWidth = lineWidth;
          ctx.stroke();
        }
      };

      ctx.clearRect(0, 0, width, height);
      ctx.fillStyle = "#7a3329";
      ctx.fillRect(0, 0, width, height);
      if (courtFrame) {
        const [xMin, xMax] = courtFrame.baselines_x;
        const [serviceNear, serviceFar] = courtFrame.service_x;
        const [yMin, yMax] = courtFrame.sidelines_y;
        fillCourtPolygon([[xMin, yMin], [xMax, yMin], [xMax, yMax], [xMin, yMax]], "#8f3f32", "rgba(255,255,255,0.78)", 3);
        strokeCourtLine(0, yMin, 0, yMax, "rgba(18,24,30,0.95)", 5);
        [serviceNear, serviceFar].forEach(x => strokeCourtLine(x, yMin, x, yMax));
        [yMin, yMax].forEach(y => strokeCourtLine(xMin, y, xMax, y));
        strokeCourtLine(serviceNear, 0, serviceFar, 0);
      } else {
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
      }

      const bounds = map.active_bounds;
      if (bounds) {
        ctx.fillStyle = "rgba(87,166,255,0.06)";
        ctx.fillRect(sx(bounds.min_x), sy(bounds.max_y), sx(bounds.max_x) - sx(bounds.min_x), sy(bounds.min_y) - sy(bounds.max_y));
      }

      // Survey fence measurements overlay — prefer persistent court_boundary.json over in-memory survey state
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
      if (Number.isFinite(robotPose.x_m) && Number.isFinite(robotPose.y_m)) {
        const rx = sx(robotPose.x_m);
        const ry = sy(robotPose.y_m);
        const yaw = Number.isFinite(robotPose.yaw_rad) ? robotPose.yaw_rad : 0;
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
      const collectionScan = (diagnostics.robot || {}).collection_scan || {};
      const laneWaypoints = Array.isArray(collectionScan.waypoints) ? collectionScan.waypoints : [];
      if (laneWaypoints.length >= 2) {
        const activeIdx = Number.isFinite(collectionScan.waypoint_index) ? collectionScan.waypoint_index : -1;
        ctx.save();
        ctx.lineJoin = "round";
        ctx.lineCap = "round";
        for (let i = 0; i + 1 < laneWaypoints.length; i += 2) {
          const a = laneWaypoints[i], b = laneWaypoints[i + 1];
          if (!a || !b) continue;
          const isActive = (i === activeIdx || i + 1 === activeIdx);
          ctx.beginPath();
          ctx.moveTo(sx(a.x_m), sy(a.y_m));
          ctx.lineTo(sx(b.x_m), sy(b.y_m));
          ctx.strokeStyle = isActive ? "rgba(168,85,247,0.95)" : "rgba(168,85,247,0.45)";
          ctx.lineWidth = isActive ? 4 : 2;
          ctx.setLineDash(isActive ? [] : [8, 6]);
          ctx.stroke();
        }
        ctx.setLineDash([]);
        laneWaypoints.forEach((wp, idx) => {
          ctx.beginPath();
          ctx.arc(sx(wp.x_m), sy(wp.y_m), 3.5, 0, Math.PI * 2);
          ctx.fillStyle = idx === activeIdx ? "#a855f7" : "rgba(168,85,247,0.55)";
          ctx.fill();
        });
        ctx.restore();
      }
      const localCandidates = Array.isArray(collectionScan.local_candidates) ? collectionScan.local_candidates : [];
      const allBalls = Array.isArray(map.balls) && map.balls.length
        ? map.balls
        : localCandidates
            .filter(ball => Number.isFinite(ball.x_m) && Number.isFinite(ball.y_m))
            .map((ball, index) => ({
              id: `local-${ball.id ?? index + 1}`,
              x_m: ball.x_m,
              y_m: ball.y_m,
              side: "same_side",
              visible_candidate: true,
              confirmed: true,
              planned: false,
              order: null,
              source: ball.source || "local_scan",
            }));

      const scanGrid = Array.isArray(collectionScan.grid) ? collectionScan.grid : null;
      if (scanGrid && (collectionScan.active || collectionScan.complete)) {
        const side = collectionScan.side || "side_neg_x";
        const isNegSide = side === "side_neg_x" || side === "left";
        const xFence = courtFrame
          ? (isNegSide ? courtFrame.baselines_x[0] : courtFrame.baselines_x[1])
          : (isNegSide ? court.min_x : court.max_x);
        const xNet = courtFrame ? 0 : (court.net_x || 0);
        const yMin = courtFrame ? courtFrame.sidelines_y[0] : court.min_y;
        const yMax = courtFrame ? courtFrame.sidelines_y[1] : court.max_y;
        const xEdges = [0, 1, 2, 3].map(i => xFence + (xNet - xFence) * (i / 3));
        const yEdges = [0, 1, 2, 3].map(i => yMax + (yMin - yMax) * (i / 3));
        const colorForCount = count => {
          if (count <= 0) return "rgba(255,255,255,0.025)";
          if (count <= 2) return "rgba(87,166,255,0.16)";
          if (count <= 5) return "rgba(255,189,90,0.18)";
          return "rgba(47,208,143,0.20)";
        };
        ctx.save();
        for (let row = 0; row < 3; row++) {
          for (let col = 0; col < 3; col++) {
            const xa = xEdges[row];
            const xb = xEdges[row + 1];
            const ya = yEdges[col];
            const yb = yEdges[col + 1];
            const count = Number((scanGrid[row] || [])[col] || 0);
            fillCourtPolygon([[xa, ya], [xb, ya], [xb, yb], [xa, yb]], colorForCount(count), "rgba(238,244,248,0.38)", 1.4);
          }
        }
        ctx.restore();
      }

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
      if (Number.isFinite(robotPose.x_m) && Number.isFinite(robotPose.y_m)) {
        const x = sx(robotPose.x_m);
        const y = sy(robotPose.y_m);
        const yaw = Number.isFinite(robotPose.yaw_rad) ? robotPose.yaw_rad : 0;
        ctx.save();
        ctx.translate(x, y);
        ctx.rotate(-yaw);
        ctx.shadowColor = "rgba(0,0,0,0.45)";
        ctx.shadowBlur = 8;
        ctx.fillStyle = "#ffbd5a";
        ctx.beginPath();
        ctx.moveTo(18, 0);
        ctx.lineTo(-11, -10);
        ctx.lineTo(-6, 0);
        ctx.lineTo(-11, 10);
        ctx.closePath();
        ctx.fill();
        ctx.shadowBlur = 0;
        ctx.strokeStyle = "rgba(12,17,22,0.9)";
        ctx.lineWidth = 2;
        ctx.stroke();
        ctx.beginPath();
        ctx.arc(0, 0, 4.5, 0, Math.PI * 2);
        ctx.fillStyle = "#0c1116";
        ctx.fill();
        ctx.restore();

        const poseText = `${robotPose.x_m.toFixed(2)}, ${robotPose.y_m.toFixed(2)}`;
        ctx.save();
        ctx.font = "bold 11px system-ui";
        ctx.textAlign = "left";
        ctx.textBaseline = "middle";
        const labelX = Math.min(width - pad - 84, x + 14);
        const labelY = Math.max(pad + 12, Math.min(height - pad - 12, y - 16));
        const labelW = ctx.measureText(poseText).width + 16;
        ctx.fillStyle = "rgba(12,17,22,0.78)";
        ctx.fillRect(labelX - 6, labelY - 10, labelW, 20);
        ctx.fillStyle = "#ffbd5a";
        ctx.fillText(poseText, labelX + 2, labelY);
        ctx.restore();
      }

      // Nav Test goal marker (map frame, same transform as the robot)
      if (_navTestGoal && Number.isFinite(_navTestGoal.x_m) && Number.isFinite(_navTestGoal.y_m)) {
        const gx = sx(_navTestGoal.x_m);
        const gy = sy(_navTestGoal.y_m);
        const gyaw = Number.isFinite(_navTestGoal.yaw_rad) ? _navTestGoal.yaw_rad : 0;
        ctx.save();
        ctx.strokeStyle = "#57a6ff";
        ctx.fillStyle = "#57a6ff";
        ctx.lineWidth = 2;
        // dashed target ring
        ctx.setLineDash([5, 4]);
        ctx.beginPath();
        ctx.arc(gx, gy, 12, 0, Math.PI * 2);
        ctx.stroke();
        ctx.setLineDash([]);
        // crosshair
        ctx.beginPath();
        ctx.moveTo(gx - 17, gy); ctx.lineTo(gx - 5, gy);
        ctx.moveTo(gx + 5, gy); ctx.lineTo(gx + 17, gy);
        ctx.moveTo(gx, gy - 17); ctx.lineTo(gx, gy - 5);
        ctx.moveTo(gx, gy + 5); ctx.lineTo(gx, gy + 17);
        ctx.stroke();
        // yaw direction arrow
        ctx.save();
        ctx.translate(gx, gy);
        ctx.rotate(-gyaw);
        ctx.beginPath();
        ctx.moveTo(22, 0); ctx.lineTo(13, -5); ctx.lineTo(13, 5);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
        // label
        const goalText = `goal ${_navTestGoal.x_m.toFixed(2)}, ${_navTestGoal.y_m.toFixed(2)}`;
        ctx.font = "bold 11px system-ui";
        ctx.textAlign = "left";
        ctx.textBaseline = "middle";
        const glx = Math.min(width - pad - 110, gx + 16);
        const gly = Math.max(pad + 12, Math.min(height - pad - 12, gy + 20));
        ctx.fillStyle = "rgba(12,17,22,0.78)";
        ctx.fillRect(glx - 6, gly - 10, ctx.measureText(goalText).width + 12, 20);
        ctx.fillStyle = "#57a6ff";
        ctx.fillText(goalText, glx, gly);
        ctx.restore();
      }

      // Live camera-detected balls (from /ball/observations via status), each
      // placed from robot pose + bearing + distance — shown before they are mapped.
      const drawCamBall = (bearing, dist) => {
        if (!Number.isFinite(bearing) || !Number.isFinite(dist)
            || !Number.isFinite(robotPose.x_m) || !Number.isFinite(robotPose.y_m)) return;
        // perception bearing_rad is +right (image x); the map is CCW, so subtract.
        const yaw = robotPose.yaw_rad || 0;
        const ang = yaw - bearing;
        // Distance is measured from the camera (0.535 m ahead of base), not base.
        const camX = robotPose.x_m + 0.535 * Math.cos(yaw);
        const camY = robotPose.y_m + 0.535 * Math.sin(yaw);
        const bx = sx(camX + dist * Math.cos(ang));
        const by = sy(camY + dist * Math.sin(ang));
        ctx.save();
        ctx.setLineDash([4, 3]);
        ctx.strokeStyle = "#ffd24a"; ctx.lineWidth = 1.5;
        ctx.beginPath(); ctx.arc(bx, by, 12, 0, Math.PI * 2); ctx.stroke();
        ctx.setLineDash([]);
        ctx.beginPath(); ctx.arc(bx, by, 7, 0, Math.PI * 2);
        ctx.fillStyle = "#ffd24a"; ctx.fill();
        ctx.strokeStyle = "#0c1116"; ctx.lineWidth = 2; ctx.stroke();
        const t = `cam ball ${dist.toFixed(2)}m`;
        ctx.font = "bold 11px system-ui";
        ctx.textAlign = "left";
        ctx.textBaseline = "middle";
        const lx = Math.min(width - pad - 120, bx + 14);
        const ly = Math.max(pad + 12, Math.min(height - pad - 12, by + 18));
        ctx.fillStyle = "rgba(12,17,22,0.78)";
        ctx.fillRect(lx - 6, ly - 10, ctx.measureText(t).width + 12, 20);
        ctx.fillStyle = "#ffd24a";
        ctx.fillText(t, lx, ly);
        ctx.restore();
      };
      const camBalls = Array.isArray(robotStatus.camera_balls) ? robotStatus.camera_balls : [];
      if (camBalls.length) {
        camBalls.forEach(b => drawCamBall(b.bearing_rad, b.distance_m));
      } else if (robotStatus.ball_visible) {
        drawCamBall(robotStatus.ball_bearing_rad, robotStatus.ball_distance_m);
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

  return { render, setGoal };
})();
