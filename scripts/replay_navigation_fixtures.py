#!/usr/bin/env python3
"""Run recorded navigation replay fixtures."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parents[1]
FIXTURE_ROOT = ROOT / "fixtures" / "navigation" / "survey"
REPLAY_SCRIPT = ROOT / "scripts" / "replay_ros2_lidar_survey.py"


def _load_manifest(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _run_replay(fixture: Path) -> dict[str, Any]:
    result = subprocess.run(
        [sys.executable, str(REPLAY_SCRIPT), str(fixture)],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(result.stderr.strip() or result.stdout.strip())
    return json.loads(result.stdout)


def _matches_expected(summary: dict[str, Any], expected: dict[str, Any]) -> bool:
    for key, value in expected.items():
        if summary.get(key) != value:
            return False
    return True


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixtures-dir", type=Path, default=FIXTURE_ROOT)
    args = parser.parse_args()

    manifests = sorted(args.fixtures_dir.glob("*.json"))
    if not manifests:
        raise SystemExit(f"no fixture manifests found in {args.fixtures_dir}")

    failures: list[str] = []
    for manifest_path in manifests:
        manifest = _load_manifest(manifest_path)
        fixture = manifest_path.parent / manifest["fixture"]
        summary = _run_replay(fixture)
        expected = manifest.get("expected", {})
        ok = _matches_expected(summary, expected)
        status = "PASS" if ok else "FAIL"
        print(f"{status} {manifest['name']}: {summary['final_state']} / {summary.get('last_event')}")
        if not ok:
            failures.append(manifest["name"])

    if failures:
        raise SystemExit(f"fixture mismatches: {', '.join(failures)}")


if __name__ == "__main__":
    main()
