"""DatabaseService — façade over the DuckDB store (TennisRobotDB).

Keeps DB access behind one capability boundary so the rest of the console
never touches the DB handle directly. The underlying TennisRobotDB instance is
injected (constructor DI), so tests can pass a fake.
"""

from __future__ import annotations


class DatabaseService:
    def __init__(self, db) -> None:
        self._db = db

    # vendors / sessions
    def read_all(self) -> dict:
        return self._db.read_all()

    def write_all(self, data: dict) -> None:
        self._db.write_all(data)

    def active_session(self) -> dict:
        return self._db.active_session()

    # surveys
    def surveys(self) -> list:
        return self._db.surveys()

    def survey_archive(self) -> list:
        return self._db.survey_archive()

    def import_survey(self, bounds: dict, *, court_id=None, vendor_id=None) -> None:
        self._db.import_survey(bounds, court_id=court_id, vendor_id=vendor_id)

    # obstacle runs
    def obstacle_runs(self, limit: int = 20) -> list:
        return self._db.obstacle_runs(limit)

    def save_obstacle_run(self, obstacle_survey: dict) -> None:
        self._db.save_obstacle_run(obstacle_survey)
