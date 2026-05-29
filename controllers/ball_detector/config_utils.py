"""Shared configuration helpers for controller modules."""

from __future__ import annotations

import os


def _env_float(name: str, default: float) -> float:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        print(f"invalid {name}={value!r}; using {default}")
        return default
