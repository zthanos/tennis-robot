"""Basket lift supervisor: endpoint classification against the 100 mm travel.

RosService closes the position loop from measured /joint_states, so these tests
drive it with synthetic joint feedback instead of a simulator. The travel comes
from BASKET_LIFT_TRAVEL_M, the same variable the model generator reads, so a
mismatch between the model's joint limit and the supervisor shows up here.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from tennis_robot.console.config import ConsoleConfig  # noqa: E402
from tennis_robot.console.ros_service import RosService  # noqa: E402

TRAVEL_M = 0.100


@pytest.fixture
def service(tmp_path, monkeypatch):
    monkeypatch.setenv("TENNIS_ROBOT_RUNTIME", "simulation")
    monkeypatch.delenv("BASKET_LIFT_TRAVEL_M", raising=False)
    return RosService(ConsoleConfig(root=tmp_path))


def _feedback(service, position: float, velocity: float = 0.0) -> None:
    service._joint_cache = {"basket_joint": (position, velocity)}
    service._joint_cache_at = float("inf")  # never expire during a test


def test_default_travel_matches_the_generated_joint_limit(service):
    assert service._basket_lift_travel() == pytest.approx(TRAVEL_M)


def test_lowered_endpoint_is_classified(service):
    _feedback(service, 0.0)
    assert service.basket_position_state() == "LOWERED"


def test_raised_endpoint_is_classified_at_one_hundred_millimetres(service):
    _feedback(service, TRAVEL_M)
    assert service.basket_position_state() == "RAISED"


def test_endpoint_tolerance_admits_a_settled_move(service):
    """set_basket_position settles to 5 mm, so classification must accept it."""
    _feedback(service, TRAVEL_M - RosService.BASKET_POSITION_TOLERANCE_M)
    assert service.basket_position_state() == "RAISED"
    _feedback(service, RosService.BASKET_POSITION_TOLERANCE_M)
    assert service.basket_position_state() == "LOWERED"


def test_mid_travel_is_not_an_endpoint(service):
    _feedback(service, TRAVEL_M / 2.0)
    assert service.basket_position_state() == "UNKNOWN"


@pytest.mark.parametrize("velocity,expected", [(0.06, "RAISING"), (-0.06, "LOWERING")])
def test_motion_is_reported_over_endpoint_position(service, velocity, expected):
    _feedback(service, TRAVEL_M / 2.0, velocity)
    assert service.basket_position_state() == expected


def test_stale_450_mm_travel_would_break_endpoint_classification(service, monkeypatch):
    """Guards the specific regression: supervisor and model disagreeing.

    With the model built at 100 mm, a supervisor still on 450 mm never sees the
    raised endpoint, so Throwing Mode would hang in RAISING_BASKET forever.
    """
    monkeypatch.setenv("BASKET_LIFT_TRAVEL_M", "0.45")
    _feedback(service, TRAVEL_M)
    assert service.basket_position_state() == "UNKNOWN"
    monkeypatch.setenv("BASKET_LIFT_TRAVEL_M", str(TRAVEL_M))
    assert service.basket_position_state() == "RAISED"


def test_hardware_runtime_refuses_the_simulation_only_actuator(service, monkeypatch):
    monkeypatch.setenv("TENNIS_ROBOT_RUNTIME", "hardware")
    _feedback(service, TRAVEL_M)
    assert service.basket_position_state() == "UNKNOWN"
    assert service.set_basket_position(True) is False
