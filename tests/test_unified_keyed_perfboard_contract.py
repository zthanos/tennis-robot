"""Mechanical grid and firmware checks for the proposed Rev C logic netlist.

These checks do not certify routing, solder joints, power or physical fit.
"""

from __future__ import annotations

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DOC = ROOT / "docs/hardware/unified-keyed-perfboard-wiring-el.md"
SKETCH = ROOT / "arduino/motion/motion_intake_mega/motion_intake_mega.ino"

DRIVER_PIN_NAMES = {
    "ML": ("DRIVE_LEFT_RPWM_PIN", "DRIVE_LEFT_LPWM_PIN", "DRIVE_LEFT_EN_PIN"),
    "MR": ("DRIVE_RIGHT_RPWM_PIN", "DRIVE_RIGHT_LPWM_PIN", "DRIVE_RIGHT_EN_PIN"),
    "IL": ("INTAKE_LEFT_RPWM_PIN", "INTAKE_LEFT_LPWM_PIN", "INTAKE_LEFT_EN_PIN"),
    "IR": ("INTAKE_RIGHT_RPWM_PIN", "INTAKE_RIGHT_LPWM_PIN", "INTAKE_RIGHT_EN_PIN"),
}
ENCODER_PIN_NAMES = {
    "LF": ("DRIVE_LF_ENC_A_PIN", "DRIVE_LF_ENC_B_PIN"),
    "LR": ("DRIVE_LR_ENC_A_PIN", "DRIVE_LR_ENC_B_PIN"),
    "RF": ("DRIVE_RF_ENC_A_PIN", "DRIVE_RF_ENC_B_PIN"),
    "RR": ("DRIVE_RR_ENC_A_PIN", "DRIVE_RR_ENC_B_PIN"),
    "IL": ("INTAKE_LEFT_ENC_A_PIN", "INTAKE_LEFT_ENC_B_PIN"),
    "IR": ("INTAKE_RIGHT_ENC_A_PIN", "INTAKE_RIGHT_ENC_B_PIN"),
}


def _firmware_pins() -> dict[str, str]:
    return dict(
        re.findall(
            r"const uint8_t ([A-Z][A-Z0-9_]*) = (A\d+|\d+);",
            SKETCH.read_text(encoding="utf-8"),
        )
    )


def test_rev_c_driver_and_encoder_nets_match_firmware() -> None:
    text = DOC.read_text(encoding="utf-8")
    firmware = _firmware_pins()
    drivers = re.findall(r"^J_(ML|MR|IL|IR)_CTRL\s+R\d+\s+(.+)$", text, re.M)
    assert len(drivers) == 4
    assert {name for name, _ in drivers} == set(DRIVER_PIN_NAMES)

    for name, contacts in drivers:
        actual = re.findall(r"C\d+=D(\d+)/(?:RPWM|LPWM|R_EN|L_EN)", contacts)
        rpwm, lpwm, enable = DRIVER_PIN_NAMES[name]
        assert actual == [firmware[rpwm], firmware[lpwm], firmware[enable], firmware[enable]]

    encoders = re.findall(
        r"^J_ENC_(LF|LR|RF|RR|IL|IR)\s+C\d+–C\d+\s+A=(D\d+|A\d+)\s+B=(D\d+|A\d+)$",
        text,
        re.M,
    )
    assert len(encoders) == 6
    assert {name for name, _, _ in encoders} == set(ENCODER_PIN_NAMES)
    for name, actual_a, actual_b in encoders:
        expected_a, expected_b = ENCODER_PIN_NAMES[name]
        for actual, expected in ((actual_a, expected_a), (actual_b, expected_b)):
            pin = firmware[expected]
            assert actual == (pin if pin.startswith("A") else f"D{pin}")


def test_rev_c_footprints_occupy_107_distinct_board_holes() -> None:
    text = DOC.read_text(encoding="utf-8")
    occupied: set[tuple[int, int]] = set()

    def claim(column: int, row: int) -> None:
        assert 1 <= column <= 42 and 1 <= row <= 30
        location = column, row
        assert location not in occupied, f"header overlap at C{column}/R{row}"
        occupied.add(location)

    for row in range(6, 23):
        for column in (3, 4):
            claim(column, row)

    drivers = re.findall(
        r"^J_(ML|MR|IL|IR)_(PWR|CTRL)\s+R(\d+)\s+(.+)$", text, re.M
    )
    assert len(drivers) == 8
    for _, kind, row, contacts in drivers:
        columns = [int(column) for column in re.findall(r"C(\d+)=", contacts)]
        assert len(columns) == (2 if kind == "PWR" else 4)
        for column in columns:
            claim(column, int(row))

    encoders = re.findall(
        r"^J_ENC_(?:LF|LR|RF|RR|IL|IR)\s+C(\d+)–C(\d+)\s+", text, re.M
    )
    assert len(encoders) == 6
    for start, end in encoders:
        assert int(end) - int(start) == 3
        for column in range(int(start), int(end) + 1):
            claim(column, 26)

    for label, expected_count in (("J_IR", 10), ("J_USER", 6), ("J_IMU", 4)):
        match = re.search(
            rf"^{label}\s+IDC 2×\d+\s+C(\d+)–C(\d+)/R(\d+)–R(\d+)",
            text,
            re.M,
        )
        assert match, label
        first_col, last_col, first_row, last_row = map(int, match.groups())
        assert (last_col - first_col + 1) * (last_row - first_row + 1) == expected_count
        for column in range(first_col, last_col + 1):
            for row in range(first_row, last_row + 1):
                claim(column, row)

    for label, expected_count in (("JP_PWR", 3), ("PWR_IN", 2)):
        match = re.search(rf"^{label}\s+1×\d+\s+C(\d+)–C(\d+)/R(\d+)", text, re.M)
        assert match, label
        first_col, last_col, row = map(int, match.groups())
        assert last_col - first_col + 1 == expected_count
        for column in range(first_col, last_col + 1):
            claim(column, row)

    assert len(occupied) == 107
