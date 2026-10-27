from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "scripts" / "intake_bench_panel.py"
SPEC = importlib.util.spec_from_file_location("intake_bench_panel", MODULE_PATH)
assert SPEC and SPEC.loader
panel = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = panel
SPEC.loader.exec_module(panel)


def test_physical_motor_mapping_and_limits_are_fixed() -> None:
    scenarios = panel.SCENARIOS
    assert scenarios["physical_right_inward"].command == "PULSE L -60 250"
    assert scenarios["physical_left_inward"].command == "PULSE R 60 250"
    assert scenarios["swapped_motor_isolation"].command == "PULSE R 60 250"
    assert scenarios["both_inward_duration"].command == "RUN_BENCH 60 4000"
    assert all(item.duration_ms <= 4000 for item in scenarios.values())
    assert all(
        item.command is None or not item.command.startswith(("RUN 90", "PULSE L 90", "PULSE R 90"))
        for item in scenarios.values()
    )


def test_every_motion_requires_an_explicit_power_instruction() -> None:
    for scenario in panel.SCENARIOS.values():
        if scenario.motion:
            assert scenario.power in {"OUT ON", "OUT OFF"}


def test_data_parser_tracks_physical_channel_telemetry() -> None:
    result = panel.parse_data_line("DATA,1700,2,-60,0,12,-4,21.5,-3.0,1,0")
    assert result == {
        "mega_ms": 1700,
        "state": 2,
        "state_name": "RUNNING",
        "left_pwm": -60,
        "right_pwm": 0,
        "left_count": 12,
        "right_count": -4,
        "left_rpm": 21.5,
        "right_rpm": -3.0,
        "entry_broken": True,
        "exit_broken": False,
    }
    assert panel.parse_data_line("READY,NO_DATA") is None


def test_html_has_emergency_stop_and_no_arbitrary_command_input() -> None:
    html = (ROOT / "scripts" / "intake_bench_panel.html").read_text()
    assert "STOP + DISARM" in html
    assert "/api/stop" in html
    assert "/api/action" in html
    assert "serial command" not in html.lower()


def test_systemd_service_is_boot_persistent_and_has_serial_group() -> None:
    service = (
        ROOT / "config" / "systemd" / "tennis-robot-intake-bench.service.in"
    ).read_text()
    assert "WantedBy=multi-user.target" in service
    assert "SupplementaryGroups=dialout" in service
    assert "--port 8082" in service
    assert "--serial-port /dev/ttyACM0" in service


def test_serial_port_is_opened_exclusively() -> None:
    source = MODULE_PATH.read_text()
    assert "exclusive=True" in source
    assert "Cancels any older AUTO watchdog" in source
