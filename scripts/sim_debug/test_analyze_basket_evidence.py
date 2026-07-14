#!/usr/bin/env python3
"""Deterministic tests for the basket evidence gate."""

from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

from analyze_basket_evidence import analyze


def _row(t: float, *, target: bool = True, stored_x: float = 0.20) -> dict:
    poses = [
        {"n": "tennis_robot", "x": 0.0, "y": 0.0, "z": 0.0, "q": [0, 0, 0, 1]},
        {"n": "stored_ball_00", "x": stored_x, "y": 0.05, "z": 0.058},
    ]
    if target:
        poses.append({"n": "ball_02", "x": 0.30, "y": 0.0, "z": 0.058})
    return {"t_sim": t, "poses": poses}


class BasketEvidenceTest(unittest.TestCase):
    def _analyze(self, rows: list[dict], *, expected_stored_count: int = 1) -> dict:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "poses.jsonl"
            path.write_text(
                "".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8"
            )
            return analyze(
                path,
                target_name="ball_02",
                expected_stored_count=expected_stored_count,
                stored_prefix="stored_ball_",
                x_min=0.02,
                x_max=0.42,
                half_width=0.14,
                z_min=0.045,
                z_max=0.25,
                dwell_s=0.75,
                settle_s=0.50,
                settle_speed_m_s=0.08,
                max_pitch_deg=8.0,
                max_roll_deg=8.0,
            )

    def test_passes_when_target_settles_and_load_is_retained(self) -> None:
        result = self._analyze([_row(index * 0.1) for index in range(11)])
        self.assertTrue(result["pass"])
        self.assertEqual(result["required_pass"], "8/8")

    def test_fails_when_target_is_removed_after_entry(self) -> None:
        rows = [_row(index * 0.1) for index in range(10)]
        rows.append(_row(1.0, target=False))
        result = self._analyze(rows)
        self.assertFalse(result["pass"])
        self.assertFalse(result["required"]["target_retained_at_end"])

    def test_fails_when_a_stored_ball_escapes(self) -> None:
        rows = [_row(index * 0.1) for index in range(10)]
        rows.append(_row(1.0, stored_x=0.50))
        result = self._analyze(rows)
        self.assertFalse(result["pass"])
        self.assertEqual(result["measurements"]["stored_escaped"], ["stored_ball_00"])


if __name__ == "__main__":
    unittest.main()

