import unittest
from pathlib import Path

from scripts.analyze_survey_replay_canonical import (
    canonical_model,
    extract_semantic_events,
    render_svg,
)


ROOT = Path(__file__).resolve().parents[1]


class SurveyReplayCanonicalAnalyzerTest(unittest.TestCase):
    def test_runtime_event_suffixes_map_to_canonical_route(self) -> None:
        ticks = [
            {"survey_event": "net_found_turning_180_for_baseline", "x_m": 0.0, "y_m": 0.0},
            {"survey_event": "baseline_fence_reached_turning_90_left", "x_m": 0.0, "y_m": -5.6},
            {"survey_event": "side_fence_reached_turning_90_left", "x_m": -5.5, "y_m": -5.6},
            {"survey_event": "far_baseline_fence_reached_turning_90_left", "x_m": -5.5, "y_m": 18.0},
            {"survey_event": "far_side_fence_reached_turning_90_left", "x_m": 5.5, "y_m": 18.0},
            {"survey_event": "return_baseline_fence_reached", "x_m": 5.5, "y_m": -5.6},
        ]

        events = extract_semantic_events(ticks)
        model = canonical_model(events)

        self.assertEqual(model["status"], "VALID")
        self.assertEqual(set(model["corners"]), {"near_left", "far_left", "far_right", "near_right"})
        svg = render_svg(events, model, "synthetic")
        self.assertIn("<svg", svg)
        self.assertIn("playable outer lines", svg)
        self.assertIn("net", svg)
        self.assertIn("near_left", svg)

    def test_known_long_side_fixture_is_partial_before_far_corners(self) -> None:
        fixture = ROOT / "fixtures" / "navigation" / "survey" / "long_side_inside_court_timeout_2026-06-05.jsonl"
        ticks = [line for line in fixture.read_text(encoding="utf-8").splitlines() if line.strip()]
        self.assertGreater(len(ticks), 100)

        import json

        events = extract_semantic_events([json.loads(line) for line in ticks])
        model = canonical_model(events)

        self.assertEqual(model["status"], "PARTIAL")
        self.assertIn("near_left_fence_corner", [event.label for event in events])
        self.assertIn("far_left_fence_corner", model["errors"][0])


if __name__ == "__main__":
    unittest.main()
