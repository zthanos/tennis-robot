"""Shared evaluation utilities for route benchmark scripts."""

from __future__ import annotations

import csv
import dataclasses
from pathlib import Path


def avg_field(rows: list, name: str) -> float:
    return sum(float(getattr(row, name)) for row in rows) / max(1, len(rows))


def write_dataclass_csv(path: Path | str, rows: list) -> None:
    if not rows:
        return
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [f.name for f in dataclasses.fields(rows[0])]
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(dataclasses.asdict(row))
