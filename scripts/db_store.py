"""DuckDB persistence layer — vendors, courts, survey history, active session."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import duckdb

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DB_PATH = ROOT / "runtime" / "tennis_robot.db"
_LEGACY_VENDORS_JSON = ROOT / "runtime" / "vendors.json"

def _canonical_fence_bounds(canonical: dict) -> dict:
    corners = canonical["corners"]
    xs = [float(corner["x_m"]) for corner in corners.values()]
    ys = [float(corner["y_m"]) for corner in corners.values()]
    if not xs or not ys:
        raise KeyError("canonical_fence_model.corners")
    return {
        "west_x": min(xs),
        "east_x": max(xs),
        "south_y": min(ys),
        "north_y": max(ys),
    }


_SCHEMA = [
    """
    CREATE TABLE IF NOT EXISTS vendors (
        id      TEXT PRIMARY KEY,
        name    TEXT NOT NULL,
        address TEXT DEFAULT '',
        created_at DOUBLE DEFAULT 0
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS courts (
        id         TEXT PRIMARY KEY,
        vendor_id  TEXT NOT NULL,
        name       TEXT NOT NULL,
        surface    TEXT DEFAULT 'clay',
        notes      TEXT DEFAULT '',
        created_at DOUBLE DEFAULT 0
    )
    """,
    """
    CREATE SEQUENCE IF NOT EXISTS survey_id_seq START 1
    """,
    """
    CREATE TABLE IF NOT EXISTS surveys (
        id             INTEGER PRIMARY KEY,
        court_id       TEXT,
        vendor_id      TEXT,
        surveyed_at    DOUBLE,
        point_count    INTEGER,
        fallback_used  BOOLEAN,
        west_x         DOUBLE,
        east_x         DOUBLE,
        south_y        DOUBLE,
        north_y        DOUBLE,
        status         TEXT DEFAULT 'SUCCESS',
        court_length_m DOUBLE,
        court_width_m  DOUBLE,
        raw_json       TEXT
    )
    """,
    """
    CREATE TABLE IF NOT EXISTS active_session (
        singleton  INTEGER PRIMARY KEY,
        vendor_id  TEXT,
        court_id   TEXT
    )
    """,
    """
    CREATE SEQUENCE IF NOT EXISTS obs_run_id_seq START 1
    """,
    """
    CREATE TABLE IF NOT EXISTS obstacle_survey_runs (
        id           INTEGER PRIMARY KEY,
        started_at   DOUBLE NOT NULL,
        elapsed_s    DOUBLE,
        status       TEXT,
        failure_reason TEXT,
        obstacle_type  TEXT,
        obstacle_dist_m DOUBLE,
        obstacle_conf  DOUBLE,
        distance_traveled_m DOUBLE,
        scan_count   INTEGER,
        robot_x_m    DOUBLE,
        robot_y_m    DOUBLE,
        debug_log    TEXT
    )
    """,
]


class TennisRobotDB:
    """Single-connection DuckDB store with thread-safe access via a lock."""

    def __init__(self, path: Path = DEFAULT_DB_PATH) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self._conn = duckdb.connect(str(path))
        self._lock = threading.Lock()
        self._init_schema()
        self._migrate_legacy()

    def close(self) -> None:
        self._conn.close()

    # ── Schema ─────────────────────────────────────────────────────────────────

    _MIGRATIONS = [
        "ALTER TABLE surveys ADD COLUMN IF NOT EXISTS status TEXT DEFAULT 'SUCCESS'",
        "ALTER TABLE surveys ADD COLUMN IF NOT EXISTS court_length_m DOUBLE",
        "ALTER TABLE surveys ADD COLUMN IF NOT EXISTS court_width_m DOUBLE",
        "ALTER TABLE surveys ADD COLUMN IF NOT EXISTS is_doubles BOOLEAN",
        "ALTER TABLE surveys ADD COLUMN IF NOT EXISTS near_baseline_to_fence_m DOUBLE",
        "ALTER TABLE surveys ADD COLUMN IF NOT EXISTS far_baseline_to_fence_m DOUBLE",
        "ALTER TABLE surveys ADD COLUMN IF NOT EXISTS left_sideline_to_fence_m DOUBLE",
        "ALTER TABLE surveys ADD COLUMN IF NOT EXISTS right_sideline_to_fence_m DOUBLE",
        "ALTER TABLE surveys ADD COLUMN IF NOT EXISTS net_world_x DOUBLE",
        "ALTER TABLE surveys ADD COLUMN IF NOT EXISTS net_world_y DOUBLE",
        "ALTER TABLE surveys ADD COLUMN IF NOT EXISTS survey_type TEXT",
        "ALTER TABLE surveys ADD COLUMN IF NOT EXISTS canonical_json TEXT",
    ]

    def _init_schema(self) -> None:
        with self._lock:
            for stmt in _SCHEMA:
                self._conn.execute(stmt)
            for stmt in self._MIGRATIONS:
                try:
                    self._conn.execute(stmt)
                except Exception:
                    pass

    def _migrate_legacy(self) -> None:
        """One-shot import from vendors.json when the vendors table is still empty."""
        if not _LEGACY_VENDORS_JSON.exists():
            return
        with self._lock:
            if self._conn.execute("SELECT COUNT(*) FROM vendors").fetchone()[0] > 0:
                return
        try:
            data = json.loads(_LEGACY_VENDORS_JSON.read_text(encoding="utf-8"))
        except Exception:
            return
        now = time.time()
        for v in data.get("vendors", []):
            self.upsert_vendor(v["id"], v.get("name", ""), v.get("address", ""), now)
        for c in data.get("courts", []):
            self.upsert_court(
                c["id"], c["vendor_id"], c.get("name", ""),
                c.get("surface", "clay"), c.get("notes", ""), now,
            )
        active = data.get("active") or {}
        if active.get("vendor_id"):
            self.set_active(active["vendor_id"], active.get("court_id"))
        n_v = len(data.get("vendors", []))
        n_c = len(data.get("courts", []))
        print(f"db: migrated {n_v} vendors, {n_c} courts from vendors.json -> tennis_robot.db")

    # ── Vendors ────────────────────────────────────────────────────────────────

    def vendors(self) -> list[dict]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT id, name, address, created_at FROM vendors ORDER BY name"
            ).fetchall()
        return [{"id": r[0], "name": r[1], "address": r[2], "created_at": r[3]} for r in rows]

    def upsert_vendor(self, id: str, name: str, address: str = "", created_at: float = 0) -> None:
        with self._lock:
            self._conn.execute(
                "INSERT INTO vendors (id, name, address, created_at) VALUES (?, ?, ?, ?)"
                " ON CONFLICT (id) DO UPDATE SET name=excluded.name, address=excluded.address",
                [id, name, address, created_at or time.time()],
            )

    def delete_vendor(self, id: str) -> None:
        with self._lock:
            self._conn.execute("DELETE FROM courts WHERE vendor_id=?", [id])
            self._conn.execute("DELETE FROM vendors WHERE id=?", [id])
            self._conn.execute(
                "UPDATE active_session SET vendor_id=NULL, court_id=NULL WHERE vendor_id=?", [id]
            )

    # ── Courts ─────────────────────────────────────────────────────────────────

    def courts(self) -> list[dict]:
        with self._lock:
            rows = self._conn.execute(
                "SELECT id, vendor_id, name, surface, notes, created_at FROM courts ORDER BY name"
            ).fetchall()
        return [
            {"id": r[0], "vendor_id": r[1], "name": r[2], "surface": r[3], "notes": r[4], "created_at": r[5]}
            for r in rows
        ]

    def upsert_court(
        self, id: str, vendor_id: str, name: str,
        surface: str = "clay", notes: str = "", created_at: float = 0,
    ) -> None:
        with self._lock:
            self._conn.execute(
                "INSERT INTO courts (id, vendor_id, name, surface, notes, created_at) VALUES (?, ?, ?, ?, ?, ?)"
                " ON CONFLICT (id) DO UPDATE SET name=excluded.name, surface=excluded.surface, notes=excluded.notes",
                [id, vendor_id, name, surface, notes, created_at or time.time()],
            )

    def delete_court(self, id: str) -> None:
        with self._lock:
            self._conn.execute("DELETE FROM courts WHERE id=?", [id])
            self._conn.execute(
                "UPDATE active_session SET court_id=NULL WHERE court_id=?", [id]
            )

    # ── Active session ─────────────────────────────────────────────────────────

    def active_session(self) -> dict:
        with self._lock:
            row = self._conn.execute(
                "SELECT vendor_id, court_id FROM active_session WHERE singleton=1"
            ).fetchone()
        return {"vendor_id": row[0], "court_id": row[1]} if row else {}

    def set_active(self, vendor_id: str | None, court_id: str | None) -> None:
        with self._lock:
            self._conn.execute(
                "INSERT INTO active_session (singleton, vendor_id, court_id) VALUES (1, ?, ?)"
                " ON CONFLICT (singleton) DO UPDATE SET vendor_id=excluded.vendor_id, court_id=excluded.court_id",
                [vendor_id, court_id],
            )
        self._sync_vendors_json()

    # ── Surveys ────────────────────────────────────────────────────────────────

    def surveys(self, limit: int = 200) -> list[dict]:
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT s.id, s.court_id, s.vendor_id, s.surveyed_at,
                       s.point_count, s.fallback_used,
                       s.west_x, s.east_x, s.south_y, s.north_y,
                       s.status, s.court_length_m, s.court_width_m,
                       s.is_doubles, s.survey_type,
                       s.near_baseline_to_fence_m, s.far_baseline_to_fence_m,
                       s.left_sideline_to_fence_m, s.right_sideline_to_fence_m,
                       s.net_world_x, s.net_world_y,
                       c.name  AS court_name,
                       v.name  AS vendor_name,
                       c.surface
                FROM   surveys s
                LEFT JOIN courts  c ON s.court_id  = c.id
                LEFT JOIN vendors v ON s.vendor_id = v.id
                ORDER  BY s.surveyed_at DESC
                LIMIT  ?
                """,
                [limit],
            ).fetchall()
        cols = [
            "id", "court_id", "vendor_id", "surveyed_at",
            "point_count", "fallback_used",
            "west_x", "east_x", "south_y", "north_y",
            "status", "court_length_m", "court_width_m",
            "is_doubles", "survey_type",
            "near_baseline_to_fence_m", "far_baseline_to_fence_m",
            "left_sideline_to_fence_m", "right_sideline_to_fence_m",
            "net_world_x", "net_world_y",
            "court_name", "vendor_name", "surface",
        ]
        return [dict(zip(cols, r)) for r in rows]

    def import_survey(self, bounds: dict) -> bool:
        """Insert a Court Knowledge Model output dict. Returns True if it was a new row."""
        surveyed_at = float(bounds.get("mapped_at") or bounds.get("surveyed_at") or 0)
        if not surveyed_at:
            return False
        with self._lock:
            if self._conn.execute(
                "SELECT COUNT(*) FROM surveys WHERE surveyed_at=?", [surveyed_at]
            ).fetchone()[0]:
                return False
            next_id = self._conn.execute(
                "SELECT COALESCE(MAX(id), 0) + 1 FROM surveys"
            ).fetchone()[0]
            status = bounds.get("status") or ("SUCCESS" if bounds.get("survey_complete") else "FAILED")
            pt_count = int(bounds.get("point_count") or 0)
            cg = bounds.get("court_geometry") or {}
            canonical = bounds.get("canonical_fence_model") or {}
            fg = _canonical_fence_bounds(canonical)
            bd = bounds.get("boundary_distances") or {}
            geo = bounds.get("geometry") or {}
            net_pos = geo.get("net_world_pos") or {}
            self._conn.execute(
                """
                INSERT INTO surveys
                  (id, court_id, vendor_id, surveyed_at, point_count, fallback_used,
                   west_x, east_x, south_y, north_y, status,
                   court_length_m, court_width_m,
                   is_doubles,
                   near_baseline_to_fence_m, far_baseline_to_fence_m,
                   left_sideline_to_fence_m, right_sideline_to_fence_m,
                   net_world_x, net_world_y,
                   survey_type, canonical_json, raw_json)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                [
                    next_id,
                    bounds.get("court_id"),
                    bounds.get("vendor_id"),
                    surveyed_at,
                    pt_count,
                    pt_count < 500,
                    fg.get("west_x"),
                    fg.get("east_x"),
                    fg.get("south_y"),
                    fg.get("north_y"),
                    status,
                    cg.get("length_m"),
                    cg.get("width_m"),
                    bounds.get("is_doubles"),
                    bd.get("near_baseline_to_fence_m"),
                    bd.get("far_baseline_to_fence_m"),
                    bd.get("left_sideline_to_fence_m"),
                    bd.get("right_sideline_to_fence_m"),
                    net_pos.get("x_m"),
                    net_pos.get("y_m"),
                    bounds.get("survey_type"),
                    json.dumps(canonical) if canonical else None,
                    json.dumps(bounds),
                ],
            )
        return True

    # ── Obstacle survey runs ───────────────────────────────────────────────────

    def save_obstacle_run(self, telemetry: dict) -> bool:
        """Persist one ObstacleSurvey run. Returns True if inserted (new run)."""
        started_at = float(telemetry.get("elapsed_s") or 0)
        result = telemetry.get("result") or {}
        surveyed_at = float(result.get("surveyed_at") or 0)
        if not surveyed_at:
            return False
        with self._lock:
            if self._conn.execute(
                "SELECT COUNT(*) FROM obstacle_survey_runs WHERE started_at=?", [surveyed_at]
            ).fetchone()[0]:
                return False
            next_id = self._conn.execute(
                "SELECT COALESCE(MAX(id), 0) + 1 FROM obstacle_survey_runs"
            ).fetchone()[0]
            pose = result.get("robot_pose_at_stop") or {}
            self._conn.execute(
                """
                INSERT INTO obstacle_survey_runs
                  (id, started_at, elapsed_s, status, failure_reason,
                   obstacle_type, obstacle_dist_m, obstacle_conf,
                   distance_traveled_m, scan_count, robot_x_m, robot_y_m, debug_log)
                VALUES (?,?,?,?,?,?,?,?,?,?,?,?,?)
                """,
                [
                    next_id,
                    surveyed_at,
                    result.get("elapsed_s"),
                    result.get("status"),
                    result.get("failure_reason"),
                    result.get("obstacle_type"),
                    result.get("obstacle_distance_m"),
                    result.get("obstacle_confidence"),
                    result.get("distance_traveled_m"),
                    telemetry.get("scan_count"),
                    pose.get("x_m"),
                    pose.get("y_m"),
                    json.dumps(telemetry.get("debug_log") or []),
                ],
            )
        return True

    def obstacle_runs(self, limit: int = 50) -> list[dict]:
        with self._lock:
            rows = self._conn.execute(
                """
                SELECT id, started_at, elapsed_s, status, failure_reason,
                       obstacle_type, obstacle_dist_m, obstacle_conf,
                       distance_traveled_m, scan_count, robot_x_m, robot_y_m, debug_log
                FROM   obstacle_survey_runs
                ORDER  BY started_at DESC
                LIMIT  ?
                """,
                [limit],
            ).fetchall()
        cols = [
            "id", "started_at", "elapsed_s", "status", "failure_reason",
            "obstacle_type", "obstacle_dist_m", "obstacle_conf",
            "distance_traveled_m", "scan_count", "robot_x_m", "robot_y_m", "debug_log",
        ]
        results = []
        for r in rows:
            row = dict(zip(cols, r))
            try:
                row["debug_log"] = json.loads(row["debug_log"] or "[]")
            except Exception:
                row["debug_log"] = []
            results.append(row)
        return results

    # ── Full read/write (used by /api/vendors) ─────────────────────────────────

    def read_all(self) -> dict:
        return {
            "vendors": self.vendors(),
            "courts": self.courts(),
            "active": self.active_session(),
        }

    def write_all(self, data: dict) -> None:
        """Replace vendors/courts/active from the Vendors page payload."""
        existing_vids = {v["id"] for v in self.vendors()}
        new_vids = {v["id"] for v in data.get("vendors", [])}
        for vid in existing_vids - new_vids:
            self.delete_vendor(vid)
        for v in data.get("vendors", []):
            self.upsert_vendor(v["id"], v.get("name", ""), v.get("address", ""))

        existing_cids = {c["id"] for c in self.courts()}
        new_cids = {c["id"] for c in data.get("courts", [])}
        for cid in existing_cids - new_cids:
            self.delete_court(cid)
        for c in data.get("courts", []):
            self.upsert_court(
                c["id"], c["vendor_id"], c.get("name", ""),
                c.get("surface", "clay"), c.get("notes", ""),
            )

        active = data.get("active") or {}
        self.set_active(active.get("vendor_id"), active.get("court_id"))

    def _sync_vendors_json(self) -> None:
        """Mirror current DuckDB state to vendors.json for the Webots controller."""
        try:
            state = self.read_all()
            tmp = _LEGACY_VENDORS_JSON.with_suffix(".tmp.json")
            tmp.write_text(json.dumps(state, indent=2), encoding="utf-8")
            tmp.replace(_LEGACY_VENDORS_JSON)
        except Exception as exc:
            print(f"db: could not write vendors.json: {exc}")
