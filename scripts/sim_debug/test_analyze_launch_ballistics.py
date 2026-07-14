import json
import math
import tempfile
import unittest
from pathlib import Path

from analyze_launch_ballistics import GRAVITY_M_S2, analyze


class LaunchBallisticsTest(unittest.TestCase):
    def _analyze_rows(
        self,
        rows: list[dict],
        *,
        fit_x_min: float,
        landing_x: float,
        landing_z: float,
        target_apex_z: float,
    ) -> dict:
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "poses.jsonl"
            path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")
            return analyze(
                path,
                ball_name="ball_02",
                fit_x_min=fit_x_min,
                fit_x_max=0.51,
                fit_z_min=0.05,
                landing_z=landing_z,
                front_row_x=0.35,
                target_landing_x=landing_x,
                target_apex_z=target_apex_z,
                target_front_clearance_z=0.124,
            )

    def test_recovers_known_high_arc_release(self) -> None:
        x0 = 0.50
        z0 = 0.063
        landing_x = 0.28
        landing_z = 0.058
        target_apex_z = 0.135
        vz = math.sqrt(2.0 * GRAVITY_M_S2 * (target_apex_z - z0))
        flight_t = (vz + math.sqrt(vz * vz + 2.0 * GRAVITY_M_S2 * (z0 - landing_z))) / GRAVITY_M_S2
        inward_vx = (x0 - landing_x) / flight_t
        rows = []
        for index, t in enumerate((0.0, 0.02, 0.04, 0.06, 0.08)):
            x = x0 - inward_vx * t
            z = z0 + vz * t - 0.5 * GRAVITY_M_S2 * t * t
            rows.append(
                {
                    "t_sim": t,
                    "poses": [
                        {"n": "tennis_robot", "x": 0.0, "y": 0.0, "z": 0.0, "q": [0, 0, 0, 1]},
                        {"n": "ball_02", "x": x, "y": 0.0, "z": z},
                    ],
                }
            )
        result = self._analyze_rows(
            rows,
            fit_x_min=0.43,
            landing_x=landing_x,
            landing_z=landing_z,
            target_apex_z=target_apex_z,
        )
        measured = result["measurements"]
        self.assertAlmostEqual(measured["inward_velocity_m_s"], inward_vx, places=6)
        self.assertAlmostEqual(measured["vertical_velocity_m_s"], vz, places=6)
        self.assertAlmostEqual(measured["release_angle_deg"], math.degrees(math.atan2(vz, inward_vx)), places=6)
        self.assertAlmostEqual(measured["target_release_velocity_robot_m_s"][0], -inward_vx, places=6)
        self.assertAlmostEqual(measured["target_release_velocity_robot_m_s"][2], vz, places=6)
        self.assertTrue(result["pass"])

    def test_expands_sparse_window_and_excludes_rolling_samples(self) -> None:
        x0 = 0.50
        z0 = 0.062
        inward_vx = 1.09
        vz = 0.76
        landing_z = 0.058
        rows = []
        for index, t in enumerate((0.0, 0.032, 0.064, 0.096, 0.128, 0.160)):
            rows.append(
                {
                    "t_sim": t,
                    "poses": [
                        {"n": "tennis_robot", "x": 0.0, "y": 0.0, "z": 0.0, "q": [0, 0, 0, 1]},
                        {
                            "n": "ball_02",
                            "x": x0 - inward_vx * t,
                            "y": 0.0,
                            "z": z0 + vz * t - 0.5 * GRAVITY_M_S2 * t * t,
                        },
                    ],
                }
            )
        for index, x in enumerate((0.30, 0.28, 0.26), start=1):
            rows.append(
                {
                    "t_sim": 0.160 + 0.032 * index,
                    "poses": [
                        {"n": "tennis_robot", "x": 0.0, "y": 0.0, "z": 0.0, "q": [0, 0, 0, 1]},
                        {"n": "ball_02", "x": x, "y": 0.0, "z": landing_z},
                    ],
                }
            )

        result = self._analyze_rows(
            rows,
            fit_x_min=0.45,
            landing_x=0.28,
            landing_z=landing_z,
            target_apex_z=0.135,
        )
        measured = result["measurements"]
        self.assertEqual(measured["fit_window"], "adaptive_to_landing")
        self.assertGreaterEqual(measured["fit_samples"], 3)
        self.assertAlmostEqual(measured["inward_velocity_m_s"], inward_vx, places=6)
        self.assertAlmostEqual(measured["vertical_velocity_m_s"], vz, places=6)


if __name__ == "__main__":
    unittest.main()
