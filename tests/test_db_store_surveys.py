from __future__ import annotations

import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from db_store import SURVEY_ARCHIVE_RETENTION_S, TennisRobotDB  # noqa: E402


def _survey(surveyed_at: float, status: str = "OK") -> dict:
    return {
        "schema": "court_knowledge_model/v2",
        "status": status,
        "surveyed_at": surveyed_at,
        "court": {"length_m": 23.77, "width_m": 10.97, "is_doubles": True},
        "distances_to_fence_m": {
            "near_baseline": 4.5,
            "far_baseline": 4.6,
            "left_sideline": 3.0,
            "right_sideline": 3.1,
        },
        "net": {"center": {"x_m": 0.0, "y_m": 0.0}},
        "occupancy": {"point_count": 1000 + int(surveyed_at)},
        "obstacles": [],
    }


def test_import_keeps_latest_survey_per_court_and_archives_old_rows() -> None:
    with tempfile.TemporaryDirectory() as tmp:
        db = TennisRobotDB(Path(tmp) / "robot.db")
        db.upsert_vendor("v1", "Vendor")
        db.upsert_court("c1", "v1", "Court 1")

        assert db.import_survey(_survey(1000.0), court_id="c1", vendor_id="v1")
        assert db.import_survey(_survey(2000.0), court_id="c1", vendor_id="v1")
        assert db.import_survey(_survey(3000.0), court_id="c1", vendor_id="v1")
        assert not db.import_survey(_survey(3000.0), court_id="c1", vendor_id="v1")

        current = db.surveys()
        archive = db.survey_archive()
        assert [row["surveyed_at"] for row in current] == [3000.0]
        assert sorted(row["surveyed_at"] for row in archive) == [1000.0, 2000.0]
        assert db.current_survey("c1")["surveyed_at"] == 3000.0

        old_archived_at = time.time() - SURVEY_ARCHIVE_RETENTION_S - 60
        db._conn.execute("UPDATE survey_audit_archive SET archived_at=?", [old_archived_at])
        assert db.import_survey(_survey(4000.0), court_id="c1", vendor_id="v1")

        current = db.surveys()
        archive = db.survey_archive()
        assert [row["surveyed_at"] for row in current] == [4000.0]
        assert [row["surveyed_at"] for row in archive] == [3000.0]

        db.close()


if __name__ == "__main__":
    test_import_keeps_latest_survey_per_court_and_archives_old_rows()
    print("survey DB archive retention OK")
