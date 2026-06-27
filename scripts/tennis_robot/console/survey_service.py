"""SurveyService — court & obstacle survey knowledge from disk.

Owns reading/parsing of the survey artifacts the mission nodes write
(``court_boundary.json``, ``court_survey_live.json``) and the fence-bounds
geometry used to gate Nav Test goals. It performs NO database writes and NO
ROS calls — persistence and ros2 are other services' jobs, orchestrated by
ConsoleApp.
"""

from __future__ import annotations

import json
import time

from .config import ConsoleConfig

# Goals may sit slightly outside the fence rectangle (e.g. approach poses).
BOUNDS_MARGIN_M = 0.5


class SurveyService:
    def __init__(self, config: ConsoleConfig) -> None:
        self._cfg = config

    # ------------------------------------------------------------------
    # persisted court boundary
    # ------------------------------------------------------------------
    def read_court_boundary(self) -> dict | None:
        path = self._cfg.court_boundary_path
        if not path.exists():
            return None
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return None

    @staticmethod
    def survey_is_complete(bounds: dict | None) -> bool:
        """v1 SUCCESS or v2 OK/FAILED. FAILED counts as complete: it is the
        audit trail for spotting process errors."""
        return bool(bounds) and (
            bounds.get("survey_complete") or bounds.get("status") in ("SUCCESS", "OK", "FAILED")
        )

    # ------------------------------------------------------------------
    # live occupancy map
    # ------------------------------------------------------------------
    def read_live_survey(self, launch_status: dict) -> dict:
        """Read the live occupancy map written by the mission node.

        Fail-loud: if the file is missing or stale we return an explicit error
        and NO fabricated waypoints/net, so a problem is visible not masked.
        """
        running = launch_status.get("running", False)
        live_path = self._cfg.court_survey_live_path
        if not live_path.exists():
            return {
                "running": running,
                "error": "court_survey_live.json missing (mission node not writing yet)",
                "map_points": [], "navigation_points": [],
            }
        try:
            data = json.loads(live_path.read_text())
        except (OSError, ValueError) as exc:
            return {
                "running": running,
                "error": f"court_survey_live.json unreadable: {exc}",
                "map_points": [], "navigation_points": [],
            }
        age = time.time() - float(data.get("updated_at", 0.0) or 0.0)
        data["age_s"] = age
        data["stale"] = age > 3.0
        data["running"] = running if launch_status.get("running") is not None else data.get("running", False)
        return data

    # ------------------------------------------------------------------
    # fence bounds (Nav Test gating)
    # ------------------------------------------------------------------
    def fence_bounds(self) -> dict[str, float] | None:
        """Axis-aligned fence rectangle in the map frame, or None if no survey."""
        data = self.read_court_boundary()
        if not data:
            return None
        corners = (data.get("fence") or {}).get("corners") or []
        xs = [c["x_m"] for c in corners if isinstance(c, dict) and isinstance(c.get("x_m"), (int, float))]
        ys = [c["y_m"] for c in corners if isinstance(c, dict) and isinstance(c.get("y_m"), (int, float))]
        if len(xs) < 2 or len(ys) < 2:
            return None
        return {"west_x": min(xs), "east_x": max(xs), "south_y": min(ys), "north_y": max(ys)}

    def check_bounds(self, x_m: float, y_m: float) -> tuple[bool, dict[str, float] | None]:
        """Return (within_bounds, bounds). When no map is available bounds is
        None and the goal is allowed (the Nav2 preflight still gates readiness)."""
        bounds = self.fence_bounds()
        if bounds is None:
            return True, None
        m = BOUNDS_MARGIN_M
        within = (
            bounds["west_x"] - m <= x_m <= bounds["east_x"] + m
            and bounds["south_y"] - m <= y_m <= bounds["north_y"] + m
        )
        return within, bounds
