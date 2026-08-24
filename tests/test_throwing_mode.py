from __future__ import annotations

import json
import sys
import time
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from tennis_robot.console.throwing_service import ThrowingService, resolve_throwing_pose
from tennis_robot.throwing_mode import (
    PROGRAMS,
    BasketState,
    FlywheelState,
    PerceptionEvent,
    ProgramId,
    RobotReadiness,
    SessionState,
    ThrowConfiguration,
    ThrowingError,
    ThrowingSession,
)


def readiness(**overrides) -> RobotReadiness:
    values = dict(
        positioned=True, stationary=True, orientation_ok=True,
        basket_state=BasketState.RAISED, flywheel_state=FlywheelState.READY,
        motor_power_available=True, estop_clear=True,
    )
    values.update(overrides)
    return RobotReadiness(**values)


def armed_session(program=ProgramId.FOREHAND) -> ThrowingSession:
    session = ThrowingSession(ThrowConfiguration.defaults(program))
    session.start(readiness(positioned=False, orientation_ok=False))
    session.navigation_confirmed(
        reached=True, stationary=True, orientation_ok=True,
        basket_state=BasketState.RAISED,
    )
    session.flywheel_confirmed_ready(readiness())
    return session


def test_training_program_selection_and_mixed_is_deterministic():
    assert PROGRAMS[ProgramId.FOREHAND].target_for_index(3).value == "forehand_zone"
    assert PROGRAMS[ProgramId.BACKHAND].target_for_index(3).value == "backhand_zone"
    mixed = PROGRAMS[ProgramId.MIXED]
    assert [mixed.target_for_index(i).value for i in range(4)] == [
        "forehand_zone", "backhand_zone", "forehand_zone", "backhand_zone"
    ]


def test_positioning_and_stationary_confirmation_are_required():
    session = ThrowingSession(ThrowConfiguration.defaults(ProgramId.FOREHAND))
    session.start(readiness(positioned=False, orientation_ok=False))
    with pytest.raises(ThrowingError):
        session.record_successful_throw(readiness())
    with pytest.raises(ThrowingError):
        session.navigation_confirmed(
            reached=True, stationary=False, orientation_ok=True,
            basket_state=BasketState.RAISED,
        )


@pytest.mark.parametrize("basket", [
    BasketState.LOWERED, BasketState.RAISING, BasketState.LOWERING,
    BasketState.UNKNOWN, BasketState.FAULT,
])
def test_session_cannot_become_ready_with_unconfirmed_basket(basket):
    session = ThrowingSession(ThrowConfiguration.defaults(ProgramId.FOREHAND))
    session.start(readiness(positioned=False, orientation_ok=False, basket_state=basket))
    session.navigation_confirmed(
        reached=True, stationary=True, orientation_ok=True, basket_state=basket,
    )
    assert session.state == SessionState.RAISING_BASKET
    with pytest.raises(ThrowingError):
        session.basket_confirmed(basket)


def test_pause_resume_stop_and_test_throw_cardinality():
    session = armed_session()
    session.pause()
    assert session.state == SessionState.PAUSED
    session.resume()
    throw_id, _ = session.record_successful_throw(readiness())
    assert session.statistics.balls_thrown == 1 and throw_id
    session.complete()
    assert session.state == SessionState.COMPLETED


@pytest.mark.parametrize("gate", [
    {"estop_clear": False},
    {"blocking_fault": "motor_driver"},
    {"hopper_ball_count": 0},
])
def test_start_and_throw_safety_gates(gate):
    session = ThrowingSession(ThrowConfiguration.defaults(ProgramId.FOREHAND))
    with pytest.raises(ThrowingError):
        session.start(readiness(**gate))


def test_unknown_ball_count_is_allowed_and_remains_unknown():
    unknown = readiness(hopper_ball_count=None, hopper_count_source="UNKNOWN")
    session = ThrowingSession(ThrowConfiguration.defaults(ProgramId.FOREHAND))
    session.start(unknown)
    assert unknown.hopper_ball_count is None


def test_throw_id_correlation_ignores_unrelated_perception():
    session = armed_session()
    throw_id, _ = session.record_successful_throw(readiness())
    assert not session.add_perception_event(PerceptionEvent("player_contact", "unrelated"))
    assert session.statistics.balls_hit is None
    assert session.add_perception_event(PerceptionEvent("player_contact", throw_id))
    assert session.statistics.balls_hit == 1
    assert session.statistics.hit_rate == 1.0


def test_unavailable_analytics_are_not_zeroes():
    session = armed_session()
    session.record_successful_throw(readiness())
    stats = session.statistics
    assert stats.balls_hit is None
    assert stats.hit_rate is None
    assert stats.balls_in is None and stats.balls_out is None
    assert stats.average_distance_m is None


def test_semantic_throwing_position_uses_measured_court_frame():
    boundary = json.loads((ROOT / "runtime/court_boundary.json").read_text())
    pose = resolve_throwing_pose(boundary, -8.0, 0.0)
    assert pose.semantic_name == "THROWING_POSITION"
    # Near-side baseline in this map is around x=-3.8; inset puts it toward net.
    assert -4.5 < pose.x_m < -2.5
    assert abs(pose.y_m) < 0.2
    assert abs(pose.yaw_rad) < 0.1


class _StatusStore:
    def __init__(self):
        self.status = {
            "robot_x_m": -8.0, "robot_y_m": 0.0,
            "robot_yaw_rad": 0.0, "measured_speed_mps": 0.0,
        }

    def read(self):
        return dict(self.status)


class _Camera:
    available = False


class _Sensors:
    def read(self): return {"front_camera": None}


class _FakeRos:
    def __init__(self, status_store=None, collector_running=False):
        self.basket_commands = []
        self.basket_stops = 0
        self.feed_requests = []
        self.flywheel_commands = []
        self.status_store = status_store
        self.collector_running = collector_running
        self.rotations = []
        self.heading_correction_works = True
        self.nav_lands_yaw_off_by = 0.0

    def runtime_kind(self): return "simulation"
    def flywheel_available(self): return True
    def basket_position_state(self): return "UNKNOWN"
    def collector_status(self): return {"ok": True, "running": self.collector_running}
    def stop_basket(self):
        self.basket_stops += 1
        return True
    def navigate_to_pose(self, x, y, yaw):
        if self.status_store is not None:
            # Nav2 reports SUCCEEDED on position; nav_lands_yaw_off_by models
            # the shared general_goal_checker ignoring final heading.
            self.status_store.status.update(
                robot_x_m=x, robot_y_m=y,
                robot_yaw_rad=yaw + self.nav_lands_yaw_off_by,
                measured_speed_mps=0.0,
            )
        return {"succeeded": True}
    def set_basket_position(self, raised):
        self.basket_commands.append(raised)
        return True
    def set_flywheel_speed(self, speed):
        self.flywheel_commands.append(speed)
        return True
    def wait_flywheel_ready(self, speed): return True
    def request_ball_feed(self, **request):
        self.feed_requests.append(request)
        return True
    def nav_cancel(self): return True
    def align_heading(self, target_yaw_rad, tolerance_rad, timeout_s=45.0):
        current = self.status_store.status["robot_yaw_rad"] if self.status_store else 0.0
        self.rotations.append(target_yaw_rad - current)
        if self.status_store is not None and self.heading_correction_works:
            self.status_store.status["robot_yaw_rad"] = target_yaw_rad
        return self.heading_correction_works


def test_service_test_throw_runs_complete_slice_exactly_once(tmp_path):
    boundary = tmp_path / "court.json"
    boundary.write_text((ROOT / "runtime/court_boundary.json").read_text())
    status_store = _StatusStore()
    ros = _FakeRos(status_store)
    service = ThrowingService(
        ros=ros, status_store=status_store, sensor_store=_Sensors(), boundary_path=boundary, camera=_Camera()
    )
    result = service.start({"program": "backhand", "ball_count": 50}, test_throw=True)
    assert result["ok"]
    deadline = time.time() + 2
    while service.status()["session"]["state"] not in {"COMPLETED", "FAULT"} and time.time() < deadline:
        time.sleep(0.01)
    status = service.status()
    assert status["session"]["state"] == "COMPLETED"
    assert status["session"]["statistics"]["balls_thrown"] == 1
    assert len(status["session"]["throw_ids"]) == 1
    assert ros.basket_commands == [True]
    assert len(ros.feed_requests) == 1
    assert ros.feed_requests[0]["throw_id"] == status["session"]["throw_ids"][0]
    assert ros.feed_requests[0]["target_zone"] == "backhand_zone"
    assert ros.flywheel_commands[-1] == 0.0


def test_manual_basket_raise_and_lower_are_confirmed(tmp_path):
    boundary = tmp_path / "court.json"
    boundary.write_text("{}")
    ros = _FakeRos()
    service = ThrowingService(
        ros=ros, status_store=_StatusStore(), sensor_store=_Sensors(), boundary_path=boundary, camera=_Camera()
    )
    assert service.basket_command(True)["ok"]
    deadline = time.time() + 1
    while service.status()["readiness"]["basket_state"] == "RAISING" and time.time() < deadline:
        time.sleep(0.01)
    assert service.status()["readiness"]["basket_state"] == "RAISED"
    assert service.basket_command(False)["ok"]
    deadline = time.time() + 1
    while service.status()["readiness"]["basket_state"] == "LOWERING" and time.time() < deadline:
        time.sleep(0.01)
    assert service.status()["readiness"]["basket_state"] == "LOWERED"
    assert ros.basket_commands == [True, False]


def test_service_rejects_stop_from_idle_and_test_throw_while_faulted(tmp_path):
    boundary = tmp_path / "court.json"
    boundary.write_text("{}")
    service = ThrowingService(
        ros=_FakeRos(), status_store=_StatusStore(), sensor_store=_Sensors(),
        boundary_path=boundary, camera=_Camera(),
    )
    assert not service.stop()["ok"]
    service._session.fault("interlock")
    assert not service.start({"program": "forehand"}, test_throw=True)["ok"]


def test_control_panel_page_and_sim_joint_integration_are_present():
    html = (ROOT / "scripts/control_panel.html").read_text()
    view = (ROOT / "scripts/control_panel/views/throwing.html").read_text()
    basket = (ROOT / "ros2_ws/src/tennis_robot/urdf/components/basket.urdf.xacro").read_text()
    controllers = (ROOT / "ros2_ws/src/tennis_robot/config/controllers.yaml").read_text()
    launch = (ROOT / "ros2_ws/src/tennis_robot/launch/sim.launch.py").read_text()
    pc_bridge = (ROOT / "config/network/pc41_lan42_domain_bridge.yaml").read_text()
    pi_bridge = (ROOT / "config/network/pc42_pi43_domain_bridge.yaml").read_text()
    extras = (ROOT / "ros2_ws/src/tennis_robot/tennis_robot/gazebo_extras_node.py").read_text()
    assert 'data-view="throwing"' in html and 'data-partial="throwing"' in html
    assert "Forehand Training" in view and "Backhand Training" in view and "Mixed Training" in view
    assert '<joint name="basket_joint" type="prismatic">' in basket
    assert 'interface_name: velocity' in controllers
    assert '_spawner("basket_velocity_controller")' in launch
    for bridge in (pc_bridge, pi_bridge):
        assert "/joint_states:" in bridge
        assert "/basket_velocity_controller/commands:" in bridge
        assert "/flywheel_velocity_controller/commands:" in bridge
        assert "/throwing/feed_request:" in bridge
    assert '"/throwing/feed_request"' in extras


# ---------------------------------------------------------------------------
# Machine-ownership interlocks (docs/mechanism/flywheel-launcher-exploration-el.md:
# "COLLECT: launcher inhibited" / "LAUNCH: intake disabled")
# ---------------------------------------------------------------------------

def _service(tmp_path, ros) -> ThrowingService:
    boundary = tmp_path / "court.json"
    boundary.write_text((ROOT / "runtime/court_boundary.json").read_text())
    return ThrowingService(
        ros=ros, status_store=_StatusStore(), sensor_store=_Sensors(),
        boundary_path=boundary, camera=_Camera(),
    )


def test_launcher_cannot_arm_while_the_collector_runs(tmp_path):
    service = _service(tmp_path, _FakeRos(collector_running=True))
    result = service.start({"program": "forehand"}, test_throw=True)
    assert not result["ok"]
    assert "collector is running" in result["message"]


def test_raising_the_basket_is_blocked_by_a_running_collector(tmp_path):
    ros = _FakeRos(collector_running=True)
    service = _service(tmp_path, ros)
    assert not service.basket_command(True)["ok"]
    # Lowering is the safe direction and stays available.
    assert service.basket_command(False)["ok"]


def test_basket_will_not_move_while_the_flywheels_are_not_idle(tmp_path):
    service = _service(tmp_path, _FakeRos())
    service._flywheel_state = FlywheelState.READY
    result = service.basket_command(True)
    assert not result["ok"] and "flywheels are READY" in result["message"]
    service._flywheel_state = FlywheelState.IDLE
    assert service.basket_command(True)["ok"]


def test_idle_session_does_not_own_the_machine(tmp_path):
    service = _service(tmp_path, _FakeRos())
    assert service.machine_owner_conflict() is None
    service._session.state = SessionState.READY
    assert "owns the robot" in service.machine_owner_conflict()


def test_completed_session_stops_the_basket_as_well_as_the_flywheels(tmp_path):
    status_store = _StatusStore()
    ros = _FakeRos(status_store)
    service = ThrowingService(
        ros=ros, status_store=status_store, sensor_store=_Sensors(),
        boundary_path=tmp_path / "court.json", camera=_Camera(),
    )
    (tmp_path / "court.json").write_text((ROOT / "runtime/court_boundary.json").read_text())
    assert service.start({"program": "forehand"}, test_throw=True)["ok"]
    deadline = time.time() + 2
    while service.status()["session"]["state"] not in {"COMPLETED", "FAULT"} and time.time() < deadline:
        time.sleep(0.01)
    assert ros.flywheel_commands[-1] == 0.0
    assert ros.basket_stops >= 1


# ---------------------------------------------------------------------------
# Basket lift/tilt readiness. RAISED is the LIFT endpoint only; the mechanical
# launch pose is 100 mm lift PLUS ~12 deg tilt, which is not yet validated.
# ---------------------------------------------------------------------------

def test_throwing_pose_is_lift_and_tilt_even_while_tilt_is_unmodelled():
    from tennis_robot.throwing_mode import TiltState
    raised = readiness()
    assert raised.lift_confirmed
    assert raised.tilt_confirmed
    assert raised.throwing_pose_confirmed
    # Lift alone is not enough once a tilt axis actually reports.
    lowered = readiness(basket_state=BasketState.LOWERED)
    assert not lowered.lift_confirmed and not lowered.throwing_pose_confirmed


def test_a_future_tilt_axis_blocks_throwing_without_touching_the_state_machine():
    from tennis_robot.throwing_mode import TiltState
    faulted = readiness(basket_tilt_state=TiltState.FAULT)
    assert faulted.lift_confirmed
    assert not faulted.tilt_confirmed
    assert not faulted.throwing_pose_confirmed
    assert "basket_tilt_not_confirmed" in faulted.throw_blockers
    confirmed = readiness(basket_tilt_state=TiltState.CONFIRMED)
    assert confirmed.throwing_pose_confirmed
    assert "basket_tilt_not_confirmed" not in confirmed.throw_blockers


def test_tilt_defaults_to_mechanical_validation_pending_and_is_reported(tmp_path):
    from tennis_robot.throwing_mode import TiltState
    service = _service(tmp_path, _FakeRos())
    status = service.status()
    assert status["capability"]["basket_tilt"] == "MECHANICAL_VALIDATION_PENDING"
    assert status["readiness"]["basket_tilt_state"] == "MECHANICAL_VALIDATION_PENDING"
    assert TiltState.MECHANICAL_VALIDATION_PENDING.value == "MECHANICAL_VALIDATION_PENDING"


def test_basket_readiness_is_travel_agnostic_so_100_mm_changes_nothing():
    """Readiness keys off the confirmed endpoint, never off a travel constant."""
    import inspect

    from tennis_robot import throwing_mode
    source = inspect.getsource(throwing_mode)
    assert "0.45" not in source and "0.100" not in source, (
        "the domain model must not encode a lift travel; the endpoint is "
        "confirmed by the actuator port from measured joint state"
    )
    assert readiness(basket_state=BasketState.RAISED).lift_confirmed


# ---------------------------------------------------------------------------
# Live defect (2026-08-24): Nav2's shared general_goal_checker uses
# yaw_goal_tolerance 3.14 so the collection lanes do not spin at each lane end,
# and Regulated Pure Pursuit performs no final rotation to the goal heading.
# NavigateToPose therefore reports SUCCEEDED at the right XY facing anywhere.
# The first live E2E run landed 159 deg off and faulted in POSITIONING.
# ---------------------------------------------------------------------------

def test_navigation_success_with_wrong_heading_is_corrected_then_confirmed(tmp_path):
    ros = _FakeRos(_StatusStore())
    ros.nav_lands_yaw_off_by = 2.77  # what Nav2 actually delivered live
    service = ThrowingService(
        ros=ros, status_store=ros.status_store, sensor_store=_Sensors(),
        boundary_path=tmp_path / "court.json", camera=_Camera(),
    )
    (tmp_path / "court.json").write_text((ROOT / "runtime/court_boundary.json").read_text())
    assert service.start({"program": "forehand"}, test_throw=True)["ok"]
    # Two confirmation windows plus the correction; do not race the thread.
    deadline = time.time() + 15
    while service.status()["session"]["state"] not in {"COMPLETED", "FAULT"} and time.time() < deadline:
        time.sleep(0.05)
    status = service.status()
    assert ros.rotations, "no heading correction was attempted"
    assert abs(ros.rotations[0] + 2.77) < 1e-6, "correction must close the measured error"
    assert status["session"]["state"] == "COMPLETED"


def test_a_failed_heading_correction_faults_instead_of_arming(tmp_path):
    """The gate decides, not the correction: never arm facing the wrong way."""
    ros = _FakeRos(_StatusStore())
    ros.nav_lands_yaw_off_by = 2.77
    ros.heading_correction_works = False
    service = ThrowingService(
        ros=ros, status_store=ros.status_store, sensor_store=_Sensors(),
        boundary_path=tmp_path / "court.json", camera=_Camera(),
    )
    (tmp_path / "court.json").write_text((ROOT / "runtime/court_boundary.json").read_text())
    assert service.start({"program": "forehand"}, test_throw=True)["ok"]
    # A rejected correction costs two confirmation windows before the fault.
    deadline = time.time() + 15
    while service.status()["session"]["state"] not in {"COMPLETED", "FAULT"} and time.time() < deadline:
        time.sleep(0.05)
    status = service.status()
    assert status["session"]["state"] == "FAULT"
    assert ros.basket_commands == [], "basket must not move with an unconfirmed heading"
    # Zero is the fault-path safety stop; any NON-zero command would mean the
    # launcher spun up behind an unconfirmed heading.
    assert all(speed == 0.0 for speed in ros.flywheel_commands), (
        f"flywheels armed with an unconfirmed heading: {ros.flywheel_commands}"
    )
    assert ros.feed_requests == []


def test_heading_already_correct_needs_no_rotation(tmp_path):
    ros = _FakeRos(_StatusStore())
    service = ThrowingService(
        ros=ros, status_store=ros.status_store, sensor_store=_Sensors(),
        boundary_path=tmp_path / "court.json", camera=_Camera(),
    )
    (tmp_path / "court.json").write_text((ROOT / "runtime/court_boundary.json").read_text())
    assert service.start({"program": "forehand"}, test_throw=True)["ok"]
    deadline = time.time() + 15
    while service.status()["session"]["state"] not in {"COMPLETED", "FAULT"} and time.time() < deadline:
        time.sleep(0.05)
    assert ros.rotations == [], "must not spin when Nav2 already delivered the heading"
    assert service.status()["session"]["state"] == "COMPLETED"
