"""Configuration contract for bounded-rate simulation time."""

import os
import sys

import pytest

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), "..", "ros2_ws", "src", "tennis_robot"
    ),
)

from tennis_robot.sim_clock_relay_node import configured_publish_hz


def test_clock_rate_defaults_to_navigation_safe_50_hz(monkeypatch):
    monkeypatch.delenv("SIM_CLOCK_PUBLISH_HZ", raising=False)
    assert configured_publish_hz() == 50.0


@pytest.mark.parametrize("value", ["0", "201", "nan", "invalid"])
def test_invalid_clock_rate_fails_loudly(monkeypatch, value):
    monkeypatch.setenv("SIM_CLOCK_PUBLISH_HZ", value)
    with pytest.raises(ValueError, match="SIM_CLOCK_PUBLISH_HZ"):
        configured_publish_hz()
