"""Actuator commands must outlive their own delivery.

The console publishes actuator setpoints from a short-lived `ros2 topic pub`.
`--times` already waits for a matching subscription, so the burst is written —
but the process then exits immediately, and when the controller is on ANOTHER
machine the reliable-delivery handshake has not completed yet, so every sample
is dropped and the command silently never arrives.

Measured Pi -> PC on the distributed bench, against a live
flywheel_velocity_controller:

    --times 5 -r 10                  flywheel stayed 0.0 rad/s   (nothing arrived)
    --times 5 -r 10 --keep-alive 3   flywheel reached 55.0 rad/s
    keep-alive sweep: 0.25 s dropped commands; 0.5 s and 1.0 s delivered 3/3

The live symptom was a Throwing Mode session faulting with "flywheel readiness
was not confirmed" while the flywheels never turned. Raising the readiness
timeout would not have helped — the command was never delivered at all.

These tests pin the argv, because the failure is invisible from the return code:
`ros2 topic pub` exits 0 whether or not the samples were delivered.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from tennis_robot.console.config import ConsoleConfig  # noqa: E402
from tennis_robot.console.ros_service import RosService  # noqa: E402


@pytest.fixture
def service(tmp_path, monkeypatch):
    monkeypatch.setenv("TENNIS_ROBOT_RUNTIME", "simulation")
    return RosService(ConsoleConfig(root=tmp_path))


@pytest.fixture
def recorded(service, monkeypatch):
    """Capture the argv of every publish instead of running ros2."""
    calls: list[list[str]] = []

    class _Result:
        returncode = 0
        stdout = ""

    def _run(argv, **_kwargs):
        calls.append(list(argv))
        return _Result()

    monkeypatch.setattr("tennis_robot.console.ros_service.subprocess.run", _run)
    return calls


def _flag_value(argv: list[str], flag: str) -> str:
    return argv[argv.index(flag) + 1]


def test_publish_keeps_the_writer_alive_after_the_last_message(service, recorded):
    service._publish_command("/t", "std_msgs/msg/String", "data: 'x'")
    argv = recorded[0]
    assert "--keep-alive" in argv, (
        "without --keep-alive the writer is destroyed before the reliable "
        "handshake completes and a cross-machine command is silently dropped"
    )
    assert float(_flag_value(argv, "--keep-alive")) >= 0.5, (
        "0.25 s was measured to still drop commands Pi -> PC"
    )


def test_keep_alive_exceeds_the_measured_drop_threshold():
    assert RosService.COMMAND_PUBLISH_KEEP_ALIVE_S >= 0.5


def test_publish_still_repeats_the_idempotent_setpoint(service, recorded):
    service._publish_command("/t", "std_msgs/msg/String", "data: 'x'")
    argv = recorded[0]
    assert int(_flag_value(argv, "--times")) == RosService.COMMAND_PUBLISH_TIMES
    assert RosService.COMMAND_PUBLISH_TIMES > 1


def test_publish_timeout_still_covers_the_whole_publish(service, recorded):
    """The subprocess budget must exceed burst + keep-alive, or a delivered
    command is reported as a failure."""
    burst_s = RosService.COMMAND_PUBLISH_TIMES / RosService.COMMAND_PUBLISH_RATE_HZ
    assert 12.0 > burst_s + RosService.COMMAND_PUBLISH_KEEP_ALIVE_S


def test_every_actuator_path_goes_through_the_lingering_publisher(service, recorded):
    """Flywheel, basket stop and collector control share one delivery guarantee."""
    service.set_flywheel_speed(18.0)
    service.stop_basket()
    # Readiness probes are service calls, not publishes; only publishes carry
    # the delivery guarantee.
    publishes = [argv for argv in recorded if argv[1:3] == ["topic", "pub"]]
    topics = [next(a for a in argv if a.startswith("/")) for argv in publishes]
    # set_flywheel_speed only publishes when the controller is reachable; the
    # basket stop is unconditional and must always be on the wire.
    assert "/basket_velocity_controller/commands" in topics
    for argv in publishes:
        assert "--keep-alive" in argv
